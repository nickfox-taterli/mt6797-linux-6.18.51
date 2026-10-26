// SPDX-License-Identifier: GPL-2.0-only
/* MT6797 BSP algorithm bring-up: bounded DMA and engine resource access. */
#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/iommu.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>

#define LAB_ALLOC _IOWR('X', 1, struct lab_mem)
#define LAB_FREE _IOW('X', 2, struct lab_mem)
#define LAB_WAIT _IOWR('X', 3, struct lab_wait)
#define LAB_LOCK _IO('X', 4)
#define LAB_UNLOCK _IO('X', 5)
#define LAB_POWERCYCLE _IO('X', 6)
#define LAB_DMA_BASE 0x60000000UL
#define LAB_MAX_BYTES SZ_128M
static bool diagnostics;
module_param(diagnostics, bool, 0644);
MODULE_PARM_DESC(diagnostics, "Read-only engine/SMI status on open and timeout");

struct lab_mem { __u32 size, id; __u64 dma; };
struct lab_wait { __u32 timeout_ms, status; };
struct lab_buffer {
	struct list_head list;
	void *cpu;
	dma_addr_t dma;
	u32 size, id;
	unsigned int maps;
};
struct lab_file;
struct lab_engine {
	struct device *dev, *larb;
	struct miscdevice misc;
	void __iomem *regs;
	void __iomem *gcon;
	void __iomem *larb_regs;
	resource_size_t base, size;
	struct clk_bulk_data *clks;
	int num_clks, irq;
	bool encoder;
	atomic_t in_use;
	struct completion done;
	u32 status;
	struct mutex lifecycle;
	struct lab_file *context;
	bool powered, sleeping, irq_live, system_sleep;
};
struct lab_file {
	struct lab_engine *engine;
	struct mutex mutex;
	struct list_head buffers;
	u32 next_id;
	size_t bytes;
	bool locked;
};
static void lab_diagnose(struct lab_engine *e)
{
	if (!diagnostics)
		return;
	dev_info(e->dev, "SMI sta=%08x mmu=%08x secure=%08x domain=%08x engine=%08x\n",
		 readl(e->larb_regs), readl(e->larb_regs + 0xfc0),
		 readl(e->larb_regs + 0xfc4), readl(e->larb_regs + 0xfd0),
		 readl(e->regs));
}

static irqreturn_t lab_irq(int irq, void *data)
{
	struct lab_engine *e = data;
	u32 status;

	if (e->encoder) {
		status = readl(e->regs + 0x5c);
		if (!status)
			return IRQ_NONE;
		/* BSP acknowledges each cause individually. */
		for (u32 bit = 1; bit <= 0x80; bit <<= 1)
			if (status & bit)
				writel(bit, e->regs + 0x60);
	} else {
		status = readl(e->regs + 0xa4);
		if (!(status & BIT(16)))
			return IRQ_NONE;
		writel(status | 0x11, e->regs + 0xa4);
		writel((status | 0x11) & ~0x10, e->regs + 0xa4);
	}
	WRITE_ONCE(e->status, status);
	complete(&e->done);
	return IRQ_HANDLED;
}

static int lab_power_on(struct lab_engine *e)
{
	int ret = pm_runtime_resume_and_get(e->dev);

	if (ret < 0)
		return ret;
	if (e->gcon) {
		writel(1, e->gcon + 0x10);
		readl(e->gcon + 0x10);
	}
	ret = pm_runtime_resume_and_get(e->larb);
	if (ret < 0)
		goto put;
	ret = clk_bulk_prepare_enable(e->num_clks, e->clks);
	if (ret)
		goto larb;
	if (e->encoder) {
		/* A restored domain can retain the previous HEVC MODE/RCPU
		 * enable. No bus master may run before the new owner maps DMA. */
		writel(0, e->regs);
		writel(0, e->regs + 0xa8);
		readl(e->regs + 0xa8);
	}
	WRITE_ONCE(e->status, 0);
	reinit_completion(&e->done);
	e->powered = true;
	enable_irq(e->irq);
	e->irq_live = true;
	return 0;
larb:
	pm_runtime_put_sync(e->larb);
put:
	pm_runtime_put_sync(e->dev);
	return ret;
}

static void lab_stop(struct lab_engine *e)
{
	if (!e->powered)
		return;
	if (e->irq_live) {
		disable_irq(e->irq);
		e->irq_live = false;
	}
	/* Complete the reset and retire the IRQ before clocks or DMA go away. */
	if (e->encoder) {
		/* RESETB alone does not retire a retained RCPU enable. Clear
		 * MODE before reset/free, including process-death and PM paths. */
		writel(0, e->regs);
		readl(e->regs);
		writel(0, e->regs + 0xa8);
		readl(e->regs + 0xa8);
	} else {
		u32 idle;

		/* Successful userspace submissions stop the decoder bus before
		 * reading output. Timeout and process-death paths must do it too:
		 * resetting GCON first can leave the VLD fetcher requesting IOVA 0. */
		writel(readl(e->regs + 0x100) | 1, e->regs + 0x100);
		if (readl_poll_timeout(e->regs + 0x104, idle,
				       (idle & 0x11) == 0x11, 1, 2000))
			dev_warn(e->dev, "decoder bus stop timed out: %08x\n", idle);
		writel(0x101, e->regs + 0x1108);
		readl(e->regs + 0x1108);
		writel(0, e->gcon + 0x10);
		readl(e->gcon + 0x10);
	}
}

static void lab_power_off(struct lab_engine *e)
{
	if (!e->powered)
		return;
	lab_stop(e);
	clk_bulk_disable_unprepare(e->num_clks, e->clks);
	pm_runtime_put_sync(e->larb);
	pm_runtime_put_sync(e->dev);
	e->powered = false;
}

static int lab_open(struct inode *inode, struct file *file)
{
	struct lab_engine *e = container_of(file->private_data, struct lab_engine, misc);
	struct lab_file *ctx;
	int ret;

	if (atomic_cmpxchg(&e->in_use, 0, 1))
		return -EBUSY;
	mutex_lock(&e->lifecycle);
	if (e->system_sleep) { ret = -EBUSY; goto unlock; }
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) { ret = -ENOMEM; goto unlock; }
	ret = lab_power_on(e);
	if (ret)
		goto free;
	lab_diagnose(e);
	ctx->engine = e;
	mutex_init(&ctx->mutex);
	INIT_LIST_HEAD(&ctx->buffers);
	file->private_data = ctx;
	e->context = ctx;
	mutex_unlock(&e->lifecycle);
	return 0;
free:
	kfree(ctx);
unlock:
	atomic_set(&e->in_use, 0);
	mutex_unlock(&e->lifecycle);
	return ret;
}

static void lab_dma_free(struct lab_file *ctx, struct lab_buffer *buf)
{
	dma_free_coherent(ctx->engine->dev, buf->size, buf->cpu, buf->dma);
	ctx->bytes -= buf->size;
	list_del(&buf->list);
	kfree(buf);
}

static int lab_release(struct inode *inode, struct file *file)
{
	struct lab_file *ctx = file->private_data;
	struct lab_engine *e = ctx->engine;
	struct lab_buffer *buf, *next;

	mutex_lock(&e->lifecycle);
	/* Reset, retire the IRQ and gate the engine before unmapping DMA.
	 * VENC can issue an RCPU request after RESETB has been lowered;
	 * keeping clocks live across dma_free_coherent leaves a use-after-
	 * unmap window even when the completed image/IRQ was correct. */
	lab_power_off(e);
	list_for_each_entry_safe(buf, next, &ctx->buffers, list)
		lab_dma_free(ctx, buf);
	e->context = NULL;
	e->sleeping = false;
	atomic_set(&e->in_use, 0);
	mutex_unlock(&e->lifecycle);
	kfree(ctx);
	return 0;
}

static long lab_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct lab_file *ctx = file->private_data;
	struct lab_engine *e = ctx->engine;
	struct lab_mem req;
	struct lab_buffer *buf;
	struct lab_wait wait;
	long ret = 0;
	void __user *user = (void __user *)arg;

	if (cmd == LAB_WAIT) {
		if (copy_from_user(&wait, user, sizeof(wait)))
			return -EFAULT;
		if (!wait.timeout_ms || wait.timeout_ms > 2000 || !ctx->locked)
			return -EINVAL;
		ret = wait_for_completion_interruptible_timeout(&e->done,
				msecs_to_jiffies(wait.timeout_ms));
		if (ret <= 0) {
			lab_diagnose(e);
			return ret ? ret : -ETIMEDOUT;
		}
		wait.status = READ_ONCE(e->status);
		return copy_to_user(user, &wait, sizeof(wait)) ? -EFAULT : 0;
	}
	mutex_lock(&e->lifecycle);
	mutex_lock(&ctx->mutex);
	if (!e->powered || e->sleeping) { ret = -EHOSTDOWN; goto out; }
	if (cmd == LAB_LOCK) {
		if (ctx->locked) { ret = -EBUSY; goto out; }
		reinit_completion(&e->done);
		ctx->locked = true;
		goto out;
	}
	if (cmd == LAB_UNLOCK) {
		if (!ctx->locked) { ret = -EINVAL; goto out; }
		ctx->locked = false;
		goto out;
	}
	/* The product flow powers the VDEC domain off/on between frames;
	 * replicate it while the coherent mappings stay attached. */
	if (cmd == LAB_POWERCYCLE) {
		if (ctx->locked) { ret = -EBUSY; goto out; }
		lab_power_off(e);
		ret = lab_power_on(e);
		goto out;
	}
	if (cmd != LAB_ALLOC && cmd != LAB_FREE) { ret = -ENOTTY; goto out; }
	if (copy_from_user(&req, user, sizeof(req))) { ret = -EFAULT; goto out; }
	if (cmd == LAB_FREE) {
		ret = -ENOENT;
		list_for_each_entry(buf, &ctx->buffers, list) {
			if (buf->id != req.id)
				continue;
			if (buf->maps || ctx->locked) { ret = -EBUSY; break; }
			lab_dma_free(ctx, buf);
			ret = 0;
			break;
		}
		goto out;
	}
	if (!req.size || req.size > SZ_64M || ctx->bytes + PAGE_ALIGN(req.size) > LAB_MAX_BYTES ||
	    ctx->next_id >= 0xffff) { ret = -EINVAL; goto out; }
	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) { ret = -ENOMEM; goto out; }
	buf->size = PAGE_ALIGN(req.size);
	buf->cpu = dma_alloc_coherent(e->dev, buf->size, &buf->dma, GFP_KERNEL);
	if (!buf->cpu) { kfree(buf); ret = -ENOMEM; goto out; }
	if (upper_32_bits(buf->dma) || buf->dma + buf->size > BIT_ULL(32)) {
		dma_free_coherent(e->dev, buf->size, buf->cpu, buf->dma);
		kfree(buf); ret = -ERANGE; goto out;
	}
	buf->id = ++ctx->next_id;
	list_add_tail(&buf->list, &ctx->buffers);
	ctx->bytes += buf->size;
	req.id = buf->id; req.dma = buf->dma; req.size = buf->size;
	if (copy_to_user(user, &req, sizeof(req))) {
		lab_dma_free(ctx, buf); ret = -EFAULT; goto out;
	}
	dev_dbg(e->dev, "DMA id=%u iova=%pad size=%u\n", buf->id, &buf->dma, buf->size);
out:
	mutex_unlock(&ctx->mutex);
	mutex_unlock(&e->lifecycle);
	return ret;
}

static void lab_vma_open(struct vm_area_struct *vma)
{
	struct lab_file *ctx = vma->vm_file->private_data;
	struct lab_buffer *buf = vma->vm_private_data;

	mutex_lock(&ctx->mutex);
	buf->maps++;
	mutex_unlock(&ctx->mutex);
}
static void lab_vma_close(struct vm_area_struct *vma)
{
	struct lab_file *ctx = vma->vm_file->private_data;
	struct lab_buffer *buf = vma->vm_private_data;

	mutex_lock(&ctx->mutex);
	buf->maps--;
	mutex_unlock(&ctx->mutex);
}
static const struct vm_operations_struct lab_vm_ops = {
	.open = lab_vma_open, .close = lab_vma_close,
};

static int lab_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct lab_file *ctx = file->private_data;
	struct lab_engine *e = ctx->engine;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long size = vma->vm_end - vma->vm_start;
	struct lab_buffer *buf;
	int ret = -EINVAL;

	if (!(vma->vm_flags & VM_SHARED))
		return -EINVAL;
	if (!e->encoder && offset == 0x16000000 && size == PAGE_SIZE) {
		vma->vm_pgoff = 0;
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		return vm_iomap_memory(vma, 0x16000000, PAGE_SIZE);
	}
	/* Engine registers only, no arbitrary physical mappings. */
	if (offset >= e->base && offset + size >= offset && offset + size <= e->base + e->size) {
		vma->vm_pgoff -= e->base >> PAGE_SHIFT;
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		return vm_iomap_memory(vma, e->base, e->size);
	}
	mutex_lock(&ctx->mutex);
	list_for_each_entry(buf, &ctx->buffers, list) {
		if (offset != LAB_DMA_BASE + ((unsigned long)buf->id << PAGE_SHIFT) || size != buf->size)
			continue;
		vma->vm_pgoff = 0;
		ret = dma_mmap_coherent(e->dev, vma, buf->cpu, buf->dma, buf->size);
		if (!ret) {
			vma->vm_ops = &lab_vm_ops;
			vma->vm_private_data = buf;
			buf->maps++;
		}
		break;
	}
	mutex_unlock(&ctx->mutex);
	return ret;
}

static const struct file_operations lab_fops = {
	.owner = THIS_MODULE, .open = lab_open, .release = lab_release,
	.unlocked_ioctl = lab_ioctl, .compat_ioctl = compat_ptr_ioctl,
	.mmap = lab_mmap,
};

static int lab_probe(struct platform_device *pdev)
{
	struct lab_engine *e;
	struct device_node *node;
	struct platform_device *larb;
	struct resource *res;
	int ret;

	e = devm_kzalloc(&pdev->dev, sizeof(*e), GFP_KERNEL);
	if (!e)
		return -ENOMEM;
	e->dev = &pdev->dev;
	e->encoder = of_device_is_compatible(pdev->dev.of_node, "mediatek,mt6797-venc-lab");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	e->regs = devm_ioremap_resource(e->dev, res);
	if (IS_ERR(e->regs))
		return PTR_ERR(e->regs);
	e->base = res->start; e->size = resource_size(res);
	if (!e->encoder) {
		e->gcon = devm_ioremap(e->dev, 0x16000000, PAGE_SIZE);
		if (!e->gcon)
			return -ENOMEM;
	}
	e->num_clks = devm_clk_bulk_get_all(e->dev, &e->clks);
	if (e->num_clks < 0)
		return e->num_clks;
	node = of_parse_phandle(pdev->dev.of_node, "mediatek,larb", 0);
	if (!node)
		return -EINVAL;
	larb = of_find_device_by_node(node);
	of_node_put(node);
	if (!larb)
		return -EPROBE_DEFER;
	res = platform_get_resource(larb, IORESOURCE_MEM, 0);
	if (!res) { put_device(&larb->dev); return -EINVAL; }
	e->larb_regs = devm_ioremap(e->dev, res->start, resource_size(res));
	if (!e->larb_regs) { put_device(&larb->dev); return -ENOMEM; }
	e->larb = &larb->dev;
	put_device(e->larb);
	if (!device_link_add(e->dev, e->larb, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	ret = dma_set_mask_and_coherent(e->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	if (!device_iommu_mapped(e->dev))
		return dev_err_probe(e->dev, -ENODEV, "translated M4U domain required\n");
	e->irq = platform_get_irq(pdev, 0);
	if (e->irq < 0)
		return e->irq;
	init_completion(&e->done);
	atomic_set(&e->in_use, 0);
	mutex_init(&e->lifecycle);
	ret = devm_request_irq(e->dev, e->irq, lab_irq, IRQF_NO_AUTOEN, dev_name(e->dev), e);
	if (ret)
		return ret;
	e->misc.minor = MISC_DYNAMIC_MINOR;
	e->misc.name = e->encoder ? "x20-venc" : "x20-vdec";
	e->misc.fops = &lab_fops;
	e->misc.parent = e->dev;
	e->misc.mode = 0600;
	pm_runtime_enable(e->dev);
	ret = misc_register(&e->misc);
	if (ret) { pm_runtime_disable(e->dev); return ret; }
	platform_set_drvdata(pdev, e);
	dev_info(e->dev, "%s ready with 32-bit DMA/M4U and IRQ%d\n", e->misc.name, e->irq);
	return 0;
}
static void lab_remove(struct platform_device *pdev)
{
	struct lab_engine *e = platform_get_drvdata(pdev);

	misc_deregister(&e->misc);
	pm_runtime_disable(e->dev);
}
static int lab_suspend(struct device *dev)
{
	struct lab_engine *e = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&e->lifecycle);
	if (e->context && READ_ONCE(e->context->locked)) {
		ret = -EBUSY;
	} else {
		e->system_sleep = true;
		if (e->context && e->powered) {
			/* Idle mappings stay owned by their file across sleep. Each
			 * new job programs fresh registers after LAB_LOCK, so no MMIO
			 * image needs restoring; coherent pixels stay in RAM. */
			lab_power_off(e);
			e->sleeping = true;
		}
	}
	mutex_unlock(&e->lifecycle);
	return ret;
}
static int lab_resume(struct device *dev)
{
	struct lab_engine *e = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&e->lifecycle);
	if (e->sleeping) {
		ret = lab_power_on(e);
		if (!ret)
			e->sleeping = false;
	}
	e->system_sleep = false;
	mutex_unlock(&e->lifecycle);
	return ret;
}
static DEFINE_SIMPLE_DEV_PM_OPS(lab_pm, lab_suspend, lab_resume);
static const struct of_device_id lab_match[] = {
	{ .compatible = "mediatek,mt6797-venc-lab" },
	{ .compatible = "mediatek,mt6797-vdec-lab" }, {},
};
MODULE_DEVICE_TABLE(of, lab_match);
static struct platform_driver lab_driver = {
	.probe = lab_probe, .remove = lab_remove,
	.driver = { .name = "mt6797-codec-lab", .of_match_table = lab_match,
		.suppress_bind_attrs = true, .pm = pm_sleep_ptr(&lab_pm) },
};
module_platform_driver(lab_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT6797 BSP codec algorithm resource bridge");
