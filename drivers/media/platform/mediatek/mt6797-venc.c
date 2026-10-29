// SPDX-License-Identifier: GPL-2.0-only
/* MT6797 firmware-free AVC/HEVC V4L2 encoder.
 * Hardware protocol and generated event program from x20-sbc/tools/vcodec/open/venc.c.
 * The kernel owns all register access, DMA, reconstruction and QP feedback.
 */
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/dma-buf.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#define GUARD 4096U
#define MAX_BYTES SZ_1M
struct buffer { void *cpu; dma_addr_t dma; unsigned size, used; };
struct venc_ctx;
struct venc_dev {
 struct device *dev, *larb;
 void __iomem *regs;
 struct clk_bulk_data *clks;
 int num_clks, irq;
 struct mutex lock;
 struct v4l2_device v4l2;
 struct video_device video;
 struct v4l2_m2m_dev *m2m;
 struct work_struct work;
 struct completion done;
 struct venc_ctx *job;
 u32 status;
 bool powered;
};
struct venc_ctx {
 struct v4l2_fh fh;
 struct venc_dev *dev;
 struct v4l2_ctrl_handler controls;
 struct mutex state_lock; /* STOP versus the worker completing its last source. */
 struct buffer buffers[10];
 unsigned width, height, codec, coded_width, coded_height;
 unsigned gop_length, frame_rate, avc_profile, frame_qp, qp_i, qp_p, target_bps;
 unsigned frame, gop_frame;
 s64 bit_debt, ema_bits;
 bool rate_control, headers, aborting;
 atomic_t force_key;
};
struct bitstream { u8 *data; unsigned size, used; };
static inline struct venc_ctx *venc_fh(void *priv) { return container_of(priv, struct venc_ctx, fh); }
static void wr(struct venc_ctx *ctx, unsigned off, u32 val) { writel(val, ctx->dev->regs + off); }
static u32 rd(struct venc_ctx *ctx, unsigned off) { return readl(ctx->dev->regs + off); }
static int start_job(struct venc_ctx *ctx, u32 command, u32 expected)
{
 struct venc_dev *dev = ctx->dev;
 reinit_completion(&dev->done);
 WRITE_ONCE(dev->status, 0);
 wr(ctx, 0xec, 1);
 dma_wmb();
 wr(ctx, 0x58, command);
 if (!wait_for_completion_timeout(&dev->done, msecs_to_jiffies(500))) {
  wr(ctx, 0xec, 0);
  return -ETIMEDOUT;
 }
 wr(ctx, 0xec, 0);
 return (READ_ONCE(dev->status) & expected) ? 0 : -EIO;
}
/* RCPU task ABI: LI uses a 13-bit unsigned literal and a 7-bit GPR;
 * OUT sends a GPR to an internal control port, RET completes a task.
 * Five event vectors share the same CPU-chosen QP. 0x212 packs both
 * six-bit QP lanes; 0x276 commits the new frame state, including chroma.
 * This small program is generated from those instructions and parameters.
 * It contains no product rate-control algorithm, code image or table. */
static void frame_controller(struct buffer *buffer, unsigned qp, int hevc)
{
	u32 *program = (void *)buffer->cpu;
	static const unsigned vector[] = {0, 1, 5, 6, 7};
	memset(program, 0, 64 + 5 * 10 * sizeof(*program));
	for (unsigned task = 0; task < 5; task++) {
		unsigned pc = task * 10;
		u32 *code = program + 16 + pc;
		program[vector[task]] = pc;
        if (!hevc) {
            /* AVC uses register QP. Its shared workspace only acknowledges
             * an event and returns, without HEVC-specific control writes. */
            code[0] = 0x08000002U | ((1U << (task ? task-1 : 0)) << 9);
            code[1] = 0x0b002211U;
            code[2] = 0x0a800000U;
            continue;
        }
		code[0] = 0x08000002U | (qp << 9);       /* LI r2, QP */
		code[1] = 0x0b002213U;                  /* OUT QP */
		code[2] = 0x08000002U | ((qp | (qp << 6)) << 9);
		code[3] = 0x0b002212U;                  /* OUT QP lanes */
		code[4] = 0x08000000U;                  /* LI r0, 0 */
		code[5] = 0x0b000276U;                  /* Commit frame state */
		code[6] = 0x08000002U | ((1U << (task ? task-1 : 0)) << 9);
		code[7] = 0x0b002211U;                  /* Event completion */
		code[8] = 0x0a800000U;                  /* RET */
	}
}
struct bits {
	u8 bytes[128];
	unsigned pos;
	bool error;
};
static void bits_u(struct bits *b, unsigned n, u32 v)
{
	if (n > 32 || b->pos + n > sizeof(b->bytes) * 8) { b->error = true; return; }
	for (unsigned i = n; i; i--) {
		if ((v >> (i - 1)) & 1)
			b->bytes[b->pos / 8] |= 1 << (7 - b->pos % 8);
		b->pos++;
	}
}
static void bits_ue(struct bits *b, u32 v)
{
	unsigned n = 0;
	for (u32 x = ++v; x >>= 1;)
		n++;
	bits_u(b, n, 0);
	bits_u(b, n + 1, v);
}
static void bits_se(struct bits *b, int v) { bits_ue(b, v <= 0 ? -v * 2 : v * 2 - 1); }
static int put_byte(struct bitstream *out, u8 byte)
{
 if (out->used >= out->size) return -ENOSPC;
 out->data[out->used++] = byte;
 return 0;
}
static int write_nal(struct bitstream *out, struct bits *b, unsigned header)
{
 unsigned zeros = 0;
 bits_u(b, 1, 1);
 while (b->pos % 8 && !b->error) bits_u(b, 1, 0);
 if (b->error) return -EINVAL;
 if (put_byte(out, 0) || put_byte(out, 0) || put_byte(out, 0) || put_byte(out, 1) || put_byte(out, header)) return -ENOSPC;
 for (unsigned i=0; i<b->pos/8; i++) {
  u8 v = b->bytes[i];
  if (zeros == 2 && v <= 3) { if (put_byte(out, 3)) return -ENOSPC; zeros=0; }
  if (put_byte(out, v)) return -ENOSPC;
  zeros = v == 0 ? zeros+1 : 0;
 }
 return 0;
}
static int avc_headers(struct venc_ctx *ctx, struct bitstream *out)
{
	struct bits b = {0};
	bits_u(&b, 8, ctx->avc_profile);
	bits_u(&b, 8, 0);
	bits_u(&b, 8, 42); /* Supports the admitted grid through 1920x1088 at 60 fps. */
	bits_ue(&b, 0);
	if (ctx->avc_profile == 100) {
		bits_ue(&b, 1); /* 4:2:0 */
		bits_ue(&b, 0); bits_ue(&b, 0); /* 8-bit */
		bits_u(&b, 1, 0); bits_u(&b, 1, 0); /* no bypass/scaling lists */
	}
	bits_ue(&b, 12);
	bits_ue(&b, 0);
	bits_ue(&b, 12);
	bits_ue(&b, 1);
	bits_u(&b, 1, 0);
	bits_ue(&b, ctx->coded_width / 16 - 1);
	bits_ue(&b, ctx->coded_height / 16 - 1);
	bits_u(&b, 1, 1);
	bits_u(&b, 1, 1);
	bits_u(&b, 1, ctx->coded_width != ctx->width || ctx->coded_height != ctx->height);
	if (ctx->coded_width != ctx->width || ctx->coded_height != ctx->height) {
		bits_ue(&b, 0);
		bits_ue(&b, (ctx->coded_width - ctx->width) / 2);
		bits_ue(&b, 0);
		bits_ue(&b, (ctx->coded_height - ctx->height) / 2);
	}
	bits_u(&b, 1, 1); /* vui_parameters_present_flag */
	bits_u(&b, 1, 0); /* aspect_ratio_info_present_flag */
	bits_u(&b, 1, 0); /* overscan_info_present_flag */
	bits_u(&b, 1, 0); /* video_signal_type_present_flag */
	bits_u(&b, 1, 0); /* chroma_loc_info_present_flag */
	bits_u(&b, 1, 1); /* timing_info_present_flag */
	bits_u(&b, 32, 1);
	bits_u(&b, 32, ctx->frame_rate * 2);
	bits_u(&b, 1, 1); /* fixed_frame_rate_flag */
	bits_u(&b, 1, 0); /* nal_hrd_parameters_present_flag */
	bits_u(&b, 1, 0); /* vcl_hrd_parameters_present_flag */
	bits_u(&b, 1, 0); /* pic_struct_present_flag */
	bits_u(&b, 1, 1); /* bitstream_restriction_flag */
	bits_u(&b, 1, 1); /* motion_vectors_over_pic_boundaries_flag */
	bits_ue(&b, 2);	  /* max_bytes_per_pic_denom */
	bits_ue(&b, 1);	  /* max_bits_per_mb_denom */
	bits_ue(&b, 16);
	bits_ue(&b, 16); /* max MV lengths */
	bits_ue(&b, 0);	 /* no frame reordering */
	bits_ue(&b, 1);	 /* max_dec_frame_buffering */
	if (write_nal(out, &b, 0x67))
		return -1;
	memset(&b, 0, sizeof(b));
	bits_ue(&b, 0);
	bits_ue(&b, 0);
	bits_u(&b, 1, ctx->avc_profile != 66);
	bits_u(&b, 1, 0);
	bits_ue(&b, 0);
	bits_ue(&b, 1);
	bits_ue(&b, 1);
	bits_u(&b, 1, 0);
	bits_u(&b, 2, 0);
	bits_se(&b, 0);
	bits_se(&b, 0);
	bits_se(&b, 0);
	bits_u(&b, 1, 1);
	bits_u(&b, 1, 0);
	bits_u(&b, 1, 0);
	if (ctx->avc_profile == 100) {
		bits_u(&b, 1, 1); /* transform_8x8_mode_flag */
		bits_u(&b, 1, 0); bits_se(&b, 0);
	}
	return write_nal(out, &b, 0x68);
}
static void hevc_ptl(struct bits *b)
{
	bits_u(b, 8, 1);	   /* Main, main tier */
	bits_u(b, 32, 0x60000000); /* Main/Main10 compatibility */
	bits_u(b, 16, 0x9000);
	bits_u(b, 32, 0);  /* progressive, frame-only */
	bits_u(b, 8, 153); /* Main tier level 5.1, including 1080p60. */
}
static int hevc_headers(struct venc_ctx *ctx, struct bitstream *out)
{
	struct bits b = {0};
	bits_u(&b, 8, 1); /* nuh_layer_id=0, temporal_id_plus1=1 */
	bits_u(&b, 4, 0);
	bits_u(&b, 1, 1);
	bits_u(&b, 1, 1);
	bits_u(&b, 6, 0);
	bits_u(&b, 3, 0);
	bits_u(&b, 1, 1);
	bits_u(&b, 16, 0xffff);
	hevc_ptl(&b);
	bits_u(&b, 1, 1);
	bits_ue(&b, 1);
	bits_ue(&b, 0);
	bits_ue(&b, 0);
	bits_u(&b, 6, 0);
	bits_ue(&b, 0);
	bits_u(&b, 1, 0);
	bits_u(&b, 1, 0);
	if (write_nal(out, &b, 0x40))
		return -1;
	memset(&b, 0, sizeof(b));
	bits_u(&b, 8, 1);
	bits_u(&b, 4, 0);
	bits_u(&b, 3, 0);
	bits_u(&b, 1, 1);
	hevc_ptl(&b);
	bits_ue(&b, 0);
	bits_ue(&b, 1);
	bits_ue(&b, ctx->coded_width);
	bits_ue(&b, ctx->coded_height);
	bits_u(&b, 1, ctx->coded_width != ctx->width || ctx->coded_height != ctx->height);
	if (ctx->coded_width != ctx->width || ctx->coded_height != ctx->height) {
		bits_ue(&b, 0);
		bits_ue(&b, (ctx->coded_width - ctx->width) / 2);
		bits_ue(&b, 0);
		bits_ue(&b, (ctx->coded_height - ctx->height) / 2);
	}
	bits_ue(&b, 0);
	bits_ue(&b, 0);
	bits_ue(&b, 4);
	bits_u(&b, 1, 1);
	bits_ue(&b, 1);
	bits_ue(&b, 0);
	bits_ue(&b, 0);
	bits_ue(&b, 1);
	bits_ue(&b, 1);
	bits_ue(&b, 1);
	bits_ue(&b, 1);
	bits_ue(&b, 1);
	bits_ue(&b, 0);
	bits_u(&b, 1, 0);
	bits_u(&b, 1, 0);
	bits_u(&b, 1, 1);
	bits_u(&b, 1, 0);
	bits_ue(&b, 1);
	bits_ue(&b, 1);
	bits_ue(&b, 0);
	bits_ue(&b, 0);
	bits_u(&b, 1, 1); /* one prior short-term reference */
	bits_u(&b, 1, 0);
	bits_u(&b, 1, 1);
	bits_u(&b, 1, 1);
	bits_u(&b, 1, 1); /* VUI */
	bits_u(&b, 8, 0); /* aspect, overscan, video, chroma, neutral, field,
			   * frame-field and default-display flags */
	bits_u(&b, 1, 1); /* timing info */
	bits_u(&b, 32, 1);
	bits_u(&b, 32, ctx->frame_rate);
	bits_u(&b, 1, 0); /* POC proportional to timing */
	bits_u(&b, 1, 0); /* HRD parameters */
	bits_u(&b, 1, 0); /* bitstream restrictions */
	bits_u(&b, 1, 0);
	return write_nal(out, &b, 0x42);
}
static void configure(struct venc_ctx *ctx, int hevc)
{
	/* MODE bit4 enables the RCPU. Clear the previous codec's mode before
	 * releasing RESETB: its old code/scratch IOVAs may already be unmapped.
	 * The new mode is enabled only after this job's DMA addresses are set. */
	wr(ctx, 0, 0);
	wr(ctx, 0xa8, 0);
	wr(ctx, 0xa8, 0);
	wr(ctx, 0xa8, 1);
	wr(ctx, 0, 0);
	wr(ctx, 0x04, (ctx->frame_qp << 26) | (ctx->gop_length << 14) | ctx->gop_length);
	wr(ctx, 0x08, hevc ? (ctx->frame_qp << 10) | 0x3a0 : (0x626e5fae & ~(63U << 10)) | (ctx->frame_qp << 10));
	/* Baseline uses the BSP's CAVLC value. Selecting High's 0x924e
	 * emits CABAC residuals behind a CAVLC slice header. */
	wr(ctx, 0x0c, hevc ? 0 : ctx->avc_profile == 100 ? 0x924e0000 : 0x104e0000);
	wr(ctx, 0x24, (ctx->coded_height / 16) << 16 | (ctx->coded_width / 16));
	wr(ctx, 0x28, (ctx->coded_width / 16) << 20 | (ctx->coded_width / 16) << 11);
	wr(ctx, 0x2c, ctx->coded_width * ctx->coded_height / (hevc ? 1024 : 256));
	wr(ctx, 0x30, hevc ? 0x20000000 : ctx->avc_profile == 100 ? 0xc7e00000 : ctx->avc_profile == 77 ? 0xa7c00000 : 0x07c00000);
	if (hevc)
		wr(ctx, 0x38, 0x3db3);
	wr(ctx, 0x48, hevc ? 0x00134240 : 0x000403e8);
	wr(ctx, 0x4c, hevc ? 0x306 : (ctx->frame_rate << 16) | 0x303);
	wr(ctx, 0x50, 0x2608);
	if (!hevc) {
		wr(ctx, 0x54, 0x800);
		wr(ctx, 0xf4, 0);
		wr(ctx, 0xfc, 0);
	}
	wr(ctx, 0x1c0, 0x10);
	wr(ctx, 0x1c4, 0);
	if (hevc) {
		wr(ctx, 0x12c, 0x600);
		wr(ctx, 0x1f0, 0);
	}
	wr(ctx, 0x128, !hevc && ctx->avc_profile != 66 ? 0x10000 : 0);
}

static void venc_free_buffers(struct venc_ctx *ctx)
{
 for (unsigned i=0; i<ARRAY_SIZE(ctx->buffers); i++) {
  struct buffer *b=&ctx->buffers[i];
  if (b->cpu) dma_free_coherent(ctx->dev->dev, b->size, b->cpu, b->dma);
  memset(b, 0, sizeof(*b));
 }
}
static int venc_alloc_buffers(struct venc_ctx *ctx)
{
 bool hevc=ctx->codec == V4L2_PIX_FMT_HEVC;
 unsigned w=ctx->coded_width=ALIGN(ctx->width, hevc ? 32 : 16);
 unsigned h=ctx->coded_height=ALIGN(ctx->height, hevc ? 32 : 16);
 unsigned recon, chroma, mv, used[10];
 if (hevc) { w=ctx->coded_width=max(w,128U); h=ctx->coded_height=max(h,128U); }
 recon=ALIGN(w*ALIGN(h,32)+4096,4096);
 chroma=ALIGN(w*ALIGN(h,32)/2+4096,4096);
 mv=hevc ? ALIGN(DIV_ROUND_UP(w,64)*DIV_ROUND_UP(h,32)*32,4096) :
  ALIGN(ALIGN(DIV_ROUND_UP(w,128)*(h/16)*32,64)*2,4096);
 used[0]=w*h*3/2+4096; used[1]=MAX_BYTES;
 used[2]=used[3]=recon; used[4]=used[5]=chroma;
 used[6]=used[7]=mv; used[8]=20480; used[9]=4096;
 for (unsigned i=0; i<10; i++) {
  struct buffer *b=&ctx->buffers[i];
  b->used=used[i]; b->size=PAGE_ALIGN(used[i]+GUARD);
  b->cpu=dma_alloc_coherent(ctx->dev->dev,b->size,&b->dma,GFP_KERNEL);
  if (!b->cpu) { venc_free_buffers(ctx); return -ENOMEM; }
  memset(b->cpu,0,b->used);
  memset(b->cpu+b->used,0xd3,b->size-b->used);
 }
 return 0;
}
static void venc_retire(struct venc_ctx *ctx)
{
 /* MODE must latch while RESETB is high, before any DMA can disappear. */
 wr(ctx,0,0); rd(ctx,0);
 wr(ctx,0xa8,0); rd(ctx,0xa8);
}
static int venc_power_on(struct venc_ctx *ctx)
{
 struct venc_dev *d=ctx->dev;
 int ret=pm_runtime_resume_and_get(d->dev);
 if (ret<0) return ret;
 ret=pm_runtime_resume_and_get(d->larb);
 if (ret<0) goto put;
 ret=clk_bulk_prepare_enable(d->num_clks,d->clks);
 if (ret) goto larb;
 venc_retire(ctx);
 for (u32 bit=1; bit<=0x80; bit<<=1) if (rd(ctx,0x5c)&bit) wr(ctx,0x60,bit);
 enable_irq(d->irq); d->powered=true;
 return 0;
larb:
 pm_runtime_put_sync(d->larb);
put:
 pm_runtime_put_sync(d->dev);
 return ret;
}
static void venc_power_off(struct venc_ctx *ctx)
{
 struct venc_dev *d=ctx->dev;
 if (!d->powered) return;
 venc_retire(ctx);
 disable_irq(d->irq);
 clk_bulk_disable_unprepare(d->num_clks,d->clks);
 pm_runtime_put_sync(d->larb); pm_runtime_put_sync(d->dev);
 d->powered=false;
}
static int venc_encode(struct venc_ctx *ctx, struct vb2_v4l2_buffer *src,
                       struct vb2_v4l2_buffer *dst, unsigned *bytes, bool *keyframe)
{
 bool hevc=ctx->codec==V4L2_PIX_FMT_HEVC;
 struct buffer *b=ctx->buffers;
 const u8 *pixels=vb2_plane_vaddr(&src->vb2_buf,0);
 u8 *capture=vb2_plane_vaddr(&dst->vb2_buf,0);
 struct bitstream stream={.data=capture,.size=vb2_plane_size(&dst->vb2_buf,0)};
 unsigned prefix=0,cur=ctx->frame%2,ref=cur^1;
 int ret;
 if (!pixels || !capture) return -EFAULT;
 bool forced=atomic_xchg(&ctx->force_key,0);
 *keyframe=!ctx->frame || ctx->gop_frame>=ctx->gop_length || forced;
 if (*keyframe) ctx->gop_frame=0;
 if (!ctx->rate_control) ctx->frame_qp=*keyframe ? ctx->qp_i : ctx->qp_p;
 for (unsigned plane=0;plane<2;plane++) {
  unsigned rows=ctx->height>>plane, padded=ctx->coded_height>>plane;
  const u8 *source=pixels+(plane ? ctx->width*ctx->height : 0);
  u8 *dest=b[0].cpu+(plane ? ctx->coded_width*ctx->coded_height : 0);
  for (unsigned y=0;y<rows;y++) {
   u8 *row=dest+y*ctx->coded_width;
   memcpy(row,source+y*ctx->width,ctx->width);
   for (unsigned x=ctx->width;x<ctx->coded_width;x++)
    row[x]=row[ctx->width-(plane?2:1)+(plane?x%2:0)];
  }
  for (unsigned y=rows;y<padded;y++)
   memcpy(dest+y*ctx->coded_width,dest+(rows-1)*ctx->coded_width,ctx->coded_width);
 }
 frame_controller(&b[8],ctx->frame_qp,hevc);
 ret=venc_power_on(ctx);
 if (ret) return ret;
 if (!ctx->headers) {
  if (hevc) {
   configure(ctx,1);
   wr(ctx,0x8c,b[8].dma>>4); wr(ctx,0x90,b[9].dma>>4);
   wr(ctx,0x38,0x3d23); wr(ctx,0x48,0x154240);
   wr(ctx,0x68,MAX_BYTES>>7); wr(ctx,0x64,b[1].dma>>4);
   wr(ctx,0,0x0208fff5); wr(ctx,0xa4,0xbf);
   ret=start_job(ctx,0x20,0x80);
   if (ret || !rd(ctx,0x98) || rd(ctx,0x98)>MAX_BYTES) { ret=ret ?: -EIO; goto stop; }
   ret=hevc_headers(ctx,&stream);
   if (ret) goto stop;
   wr(ctx,0x38,0); wr(ctx,0x68,MAX_BYTES>>7); wr(ctx,0x64,b[1].dma>>4);
   wr(ctx,0,0x0200fff5); wr(ctx,0xa4,0xbf);
   ret=start_job(ctx,2,2);
   if (ret) goto stop;
   unsigned n=rd(ctx,0x98);
   if (!n || n>MAX_BYTES || n>stream.size-stream.used) { ret=-EOVERFLOW; goto stop; }
   dma_rmb(); memcpy(capture+stream.used,b[1].cpu,n); stream.used+=n;
   wr(ctx,0x38,0);
  } else {
   ret=avc_headers(ctx,&stream);
   if (ret) goto stop;
  }
  prefix=stream.used;
 }
 memset(b[1].cpu,0,MAX_BYTES);
 configure(ctx,hevc);
 wr(ctx,0x10,hevc ? (ctx->gop_frame ? (ctx->gop_frame==1?0x8008:0x8108):0x800e) :
    (ctx->gop_frame ? (ctx->gop_frame==1?0x8400:0x8500):0xb406));
 wr(ctx,0x14,ctx->gop_frame); wr(ctx,0x18,ctx->gop_frame); wr(ctx,0x1c,0);
 if (!hevc) wr(ctx,0x124,0);
 wr(ctx,0x68,MAX_BYTES>>7); wr(ctx,0x64,b[1].dma>>4);
 wr(ctx,0x6c,b[0].dma>>4); wr(ctx,0x70,(b[0].dma+ctx->coded_width*ctx->coded_height)>>4);
 wr(ctx,0x74,b[2+ref].dma>>4); wr(ctx,0x78,b[4+ref].dma>>4);
 wr(ctx,0x7c,b[2+cur].dma>>4); wr(ctx,0x80,b[4+cur].dma>>4);
 wr(ctx,0x84,b[6+cur].dma>>4); wr(ctx,0x88,b[6+ref].dma>>4);
 wr(ctx,0x8c,b[8].dma>>4); wr(ctx,0x90,b[9].dma>>4);
 wr(ctx,0xa4,hevc?0xbf:0x3f);
 wr(ctx,0,(ctx->gop_frame?0x0fa00000:0x0f600000)|(hevc?0xfff5:0));
 ret=start_job(ctx,4,4);
 if (ret) goto stop;
 *bytes=rd(ctx,0x98);
 if (!*bytes || *bytes>MAX_BYTES || *bytes>stream.size-prefix) { ret=-EOVERFLOW; goto stop; }
stop:
 venc_power_off(ctx);
 dma_rmb();
 if (ret) return ret;
 for (unsigned i=0;i<10;i++)
  if (memchr_inv(b[i].cpu+b[i].used,0xd3,b[i].size-b[i].used)) return -EIO;
 memcpy(capture+prefix,b[1].cpu,*bytes);
 if (ctx->rate_control) {
  s64 target=ctx->target_bps/ctx->frame_rate,bits=(s64)*bytes*8,demand;
  int delta=0;
  ctx->bit_debt=clamp(ctx->bit_debt+bits-target,-target*8,target*8);
  ctx->ema_bits=ctx->ema_bits ? (ctx->ema_bits*3+bits)/4 : bits;
  demand=ctx->ema_bits+ctx->bit_debt/12;
  if (demand>target*112/100) delta=demand>target*2?3:demand>target*3/2?2:1;
  if (demand<target*88/100) delta=demand<target/2?-3:demand<target*2/3?-2:-1;
  ctx->frame_qp=clamp((int)ctx->frame_qp+delta,0,51);
 }
 *bytes+=prefix; ctx->headers=true; ctx->gop_frame++;
 return 0;
}

static bool venc_busy(struct venc_ctx *ctx)
{
 return ctx->fh.m2m_ctx && (vb2_is_busy(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx)) ||
                           vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)));
}
static int venc_ctrl(struct v4l2_ctrl *ctrl)
{
 struct venc_ctx *ctx=container_of(ctrl->handler,struct venc_ctx,controls);
 if (ctrl->id==V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME) { atomic_set(&ctx->force_key,1); return 0; }
 if (venc_busy(ctx)) return -EBUSY;
 switch (ctrl->id) {
 case V4L2_CID_MPEG_VIDEO_GOP_SIZE: ctx->gop_length=ctrl->val; break;
 case V4L2_CID_MPEG_VIDEO_BITRATE: ctx->target_bps=ctrl->val; break;
 case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE: ctx->rate_control=ctrl->val; break;
 case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
 case V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP: ctx->qp_i=ctrl->val; break;
 case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
 case V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP: ctx->qp_p=ctrl->val; break;
 case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
  ctx->avc_profile=ctrl->val==V4L2_MPEG_VIDEO_H264_PROFILE_HIGH?100:
                   ctrl->val==V4L2_MPEG_VIDEO_H264_PROFILE_MAIN?77:66; break;
 }
 return 0;
}
static const struct v4l2_ctrl_ops venc_ctrl_ops={.s_ctrl=venc_ctrl};
static int venc_controls(struct venc_ctx *ctx)
{
 struct v4l2_ctrl_handler *h=&ctx->controls;
 v4l2_ctrl_handler_init(h,12);
 v4l2_ctrl_new_std(h,&venc_ctrl_ops,V4L2_CID_MPEG_VIDEO_GOP_SIZE,1,120,1,120);
 v4l2_ctrl_new_std(h,&venc_ctrl_ops,V4L2_CID_MPEG_VIDEO_BITRATE,30000,100000000,1,1000000);
 v4l2_ctrl_new_std(h,&venc_ctrl_ops,V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE,0,1,1,0);
 v4l2_ctrl_new_std(h,&venc_ctrl_ops,V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,0,51,1,22);
 v4l2_ctrl_new_std(h,&venc_ctrl_ops,V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,0,51,1,22);
 v4l2_ctrl_new_std(h,&venc_ctrl_ops,V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP,0,51,1,22);
 v4l2_ctrl_new_std(h,&venc_ctrl_ops,V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP,0,51,1,22);
 v4l2_ctrl_new_std_menu(h,&venc_ctrl_ops,V4L2_CID_MPEG_VIDEO_H264_PROFILE,
   V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,BIT(V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE)|BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED),V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE);
 v4l2_ctrl_new_std_menu(h,&venc_ctrl_ops,V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
   V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,0,V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN);
 v4l2_ctrl_new_std(h,&venc_ctrl_ops,V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,0,0,0,0);
 if (h->error) return h->error;
 ctx->fh.ctrl_handler=h;
 return v4l2_ctrl_handler_setup(h);
}
static int venc_querycap(struct file *f,void *priv,struct v4l2_capability *cap)
{
 strscpy(cap->driver,"mt6797-venc",sizeof(cap->driver));
 strscpy(cap->card,"MT6797 AVC/HEVC encoder",sizeof(cap->card));
 strscpy(cap->bus_info,"platform:mt6797-venc",sizeof(cap->bus_info));
 return 0;
}
static int venc_enum_fmt(struct file *f,void *priv,struct v4l2_fmtdesc *fmt)
{
 if (V4L2_TYPE_IS_OUTPUT(fmt->type)) {
  if (fmt->index) return -EINVAL;
  fmt->pixelformat=V4L2_PIX_FMT_NV12;
 } else {
  if (fmt->index>1) return -EINVAL;
  fmt->pixelformat=fmt->index?V4L2_PIX_FMT_HEVC:V4L2_PIX_FMT_H264;
  fmt->flags=V4L2_FMT_FLAG_COMPRESSED;
 }
 return 0;
}
static int venc_try_fmt(struct file *f,void *priv,struct v4l2_format *fmt)
{
 struct venc_ctx *ctx=venc_fh(file_to_v4l2_fh(f));
 struct v4l2_pix_format_mplane *p=&fmt->fmt.pix_mp;
 bool output=V4L2_TYPE_IS_OUTPUT(fmt->type);
 if (output) {
  p->width=clamp(ALIGN(p->width,16),64U,1920U);
  p->height=clamp(ALIGN(p->height,2),64U,1088U);
  p->pixelformat=V4L2_PIX_FMT_NV12;
 } else {
  p->width=ctx->width; p->height=ctx->height;
  if (p->pixelformat!=V4L2_PIX_FMT_HEVC) p->pixelformat=V4L2_PIX_FMT_H264;
 }
 p->field=V4L2_FIELD_NONE; p->num_planes=1;
 p->colorspace=V4L2_COLORSPACE_REC709;
 p->ycbcr_enc=V4L2_YCBCR_ENC_DEFAULT; p->quantization=V4L2_QUANTIZATION_DEFAULT;
 p->xfer_func=V4L2_XFER_FUNC_DEFAULT;
 memset(p->plane_fmt,0,sizeof(p->plane_fmt));
 p->plane_fmt[0].sizeimage=output?p->width*p->height*3/2:MAX_BYTES+4096;
 p->plane_fmt[0].bytesperline=output?p->width:0;
 return 0;
}
static int venc_get_fmt(struct file *f,void *priv,struct v4l2_format *fmt)
{
 struct venc_ctx *ctx=venc_fh(file_to_v4l2_fh(f));
 fmt->fmt.pix_mp.width=ctx->width; fmt->fmt.pix_mp.height=ctx->height;
 fmt->fmt.pix_mp.pixelformat=V4L2_TYPE_IS_OUTPUT(fmt->type)?V4L2_PIX_FMT_NV12:ctx->codec;
 return venc_try_fmt(f,priv,fmt);
}
static int venc_set_fmt(struct file *f,void *priv,struct v4l2_format *fmt)
{
 struct venc_ctx *ctx=venc_fh(file_to_v4l2_fh(f));
 if (venc_busy(ctx)) return -EBUSY;
 venc_try_fmt(f,priv,fmt);
 if (V4L2_TYPE_IS_OUTPUT(fmt->type)) { ctx->width=fmt->fmt.pix_mp.width; ctx->height=fmt->fmt.pix_mp.height; }
 else ctx->codec=fmt->fmt.pix_mp.pixelformat;
 return 0;
}
static int venc_framesizes(struct file *f,void *priv,struct v4l2_frmsizeenum *fs)
{
 if (fs->index || (fs->pixel_format!=V4L2_PIX_FMT_NV12 && fs->pixel_format!=V4L2_PIX_FMT_H264 && fs->pixel_format!=V4L2_PIX_FMT_HEVC)) return -EINVAL;
 fs->type=V4L2_FRMSIZE_TYPE_STEPWISE;
 fs->stepwise=(struct v4l2_frmsize_stepwise){64,1920,16,64,1088,2};
 return 0;
}
static int venc_get_parm(struct file *f,void *priv,struct v4l2_streamparm *p)
{
 struct venc_ctx *ctx=venc_fh(file_to_v4l2_fh(f));
 if (p->type!=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) return -EINVAL;
 p->parm.output=(struct v4l2_outputparm){.capability=V4L2_CAP_TIMEPERFRAME,.timeperframe={1,ctx->frame_rate}};
 return 0;
}
static int venc_set_parm(struct file *f,void *priv,struct v4l2_streamparm *p)
{
 struct venc_ctx *ctx=venc_fh(file_to_v4l2_fh(f));
 struct v4l2_fract t=p->parm.output.timeperframe;
 if (p->type!=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE || !t.numerator || !t.denominator ||
     t.denominator%t.numerator || t.denominator/t.numerator>60 || t.denominator/t.numerator<1) return -EINVAL;
 if (venc_busy(ctx)) return -EBUSY;
 ctx->frame_rate=t.denominator/t.numerator;
 return venc_get_parm(f,priv,p);
}
static int venc_queue_setup(struct vb2_queue *q,unsigned *count,unsigned *planes,
                             unsigned sizes[],struct device *alloc_devs[])
{
 struct venc_ctx *ctx=vb2_get_drv_priv(q);
 unsigned size=V4L2_TYPE_IS_OUTPUT(q->type)?ctx->width*ctx->height*3/2:MAX_BYTES+4096;
 if (*planes) return *planes==1 && sizes[0]>=size?0:-EINVAL;
 *planes=1; sizes[0]=size;
 return 0;
}
static int venc_buf_prepare(struct vb2_buffer *vb)
{
 struct venc_ctx *ctx=vb2_get_drv_priv(vb->vb2_queue);
 unsigned need=V4L2_TYPE_IS_OUTPUT(vb->type)?ctx->width*ctx->height*3/2:MAX_BYTES+4096;
 if (vb2_plane_size(vb,0)<need) return -EINVAL;
 if (V4L2_TYPE_IS_OUTPUT(vb->type) && (vb2_get_plane_payload(vb,0)!=need || vb->planes[0].data_offset)) return -EINVAL;
 return 0;
}
static void venc_buf_queue(struct vb2_buffer *vb)
{
 struct venc_ctx *ctx=vb2_get_drv_priv(vb->vb2_queue);
 struct vb2_v4l2_buffer *buf=to_vb2_v4l2_buffer(vb);
 static const struct v4l2_event eos={.type=V4L2_EVENT_EOS};
 mutex_lock(&ctx->state_lock);
 if (!V4L2_TYPE_IS_OUTPUT(vb->type)) {
  vb2_set_plane_payload(vb,0,0);
  if (v4l2_m2m_dst_buf_is_last(ctx->fh.m2m_ctx)) {
   buf->sequence=ctx->frame;
   v4l2_m2m_last_buffer_done(ctx->fh.m2m_ctx,buf);
   v4l2_event_queue_fh(&ctx->fh,&eos);
   mutex_unlock(&ctx->state_lock);
   return;
  }
 }
 v4l2_m2m_buf_queue(ctx->fh.m2m_ctx,to_vb2_v4l2_buffer(vb));
 mutex_unlock(&ctx->state_lock);
}
static int venc_start_streaming(struct vb2_queue *q,unsigned count)
{
 struct venc_ctx *ctx=vb2_get_drv_priv(q);
 int ret=0;
 ctx->aborting=false;
 if (!ctx->buffers[0].cpu) {
  ctx->frame=ctx->gop_frame=0; ctx->headers=false;
  ctx->frame_qp=ctx->qp_i; ctx->bit_debt=ctx->ema_bits=0;
  ret=venc_alloc_buffers(ctx);
 }
 if (ret) {
  struct vb2_v4l2_buffer *b;
  while ((b=V4L2_TYPE_IS_OUTPUT(q->type)?v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx):v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
   v4l2_m2m_buf_done(b,VB2_BUF_STATE_QUEUED);
 }
 if (!ret) {
  mutex_lock(&ctx->state_lock);
  v4l2_m2m_update_start_streaming_state(ctx->fh.m2m_ctx,q);
  if (V4L2_TYPE_IS_OUTPUT(q->type) && v4l2_m2m_has_stopped(ctx->fh.m2m_ctx)) {
   v4l2_m2m_clear_state(ctx->fh.m2m_ctx);
   vb2_clear_last_buffer_dequeued(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
  }
  mutex_unlock(&ctx->state_lock);
 }
 return ret;
}
static void venc_stop_streaming(struct vb2_queue *q)
{
 struct venc_ctx *ctx=vb2_get_drv_priv(q);
 struct vb2_v4l2_buffer *b;
 struct vb2_queue *other=V4L2_TYPE_IS_OUTPUT(q->type)?v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx):v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx);
 ctx->aborting=true;
 flush_work(&ctx->dev->work);
 while ((b=V4L2_TYPE_IS_OUTPUT(q->type)?v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx):v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
  v4l2_m2m_buf_done(b,VB2_BUF_STATE_ERROR);
 /* CAPTURE STREAMOFF resets encoding. OUTPUT STREAMOFF only discards
  * queued source pictures and must retain reconstruction/GOP state. */
 if (!V4L2_TYPE_IS_OUTPUT(q->type)) {
  ctx->frame=ctx->gop_frame=0; ctx->headers=false;
  ctx->frame_qp=ctx->qp_i; ctx->bit_debt=ctx->ema_bits=0;
 }
 mutex_lock(&ctx->state_lock);
 bool stopped=v4l2_m2m_has_stopped(ctx->fh.m2m_ctx);
 v4l2_m2m_update_stop_streaming_state(ctx->fh.m2m_ctx,q);
 if (!stopped && v4l2_m2m_has_stopped(ctx->fh.m2m_ctx)) {
  static const struct v4l2_event eos={.type=V4L2_EVENT_EOS};
  v4l2_event_queue_fh(&ctx->fh,&eos);
 }
 mutex_unlock(&ctx->state_lock);
 if (!vb2_is_streaming(other)) venc_free_buffers(ctx);
}
static const struct vb2_ops venc_vb2_ops={.queue_setup=venc_queue_setup,.buf_prepare=venc_buf_prepare,
 .buf_queue=venc_buf_queue,.start_streaming=venc_start_streaming,.stop_streaming=venc_stop_streaming,
 .wait_prepare=vb2_ops_wait_prepare,.wait_finish=vb2_ops_wait_finish};
static int venc_queue_init(void *priv,struct vb2_queue *src,struct vb2_queue *dst)
{
 struct venc_ctx *ctx=priv;
 struct vb2_queue *q[2]={src,dst};
 for (unsigned i=0;i<2;i++) {
  q[i]->type=i?V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  q[i]->io_modes=VB2_MMAP|VB2_DMABUF;
  q[i]->drv_priv=ctx; q[i]->buf_struct_size=sizeof(struct v4l2_m2m_buffer);
  q[i]->ops=&venc_vb2_ops; q[i]->mem_ops=&vb2_dma_contig_memops;
  q[i]->timestamp_flags=V4L2_BUF_FLAG_TIMESTAMP_COPY; q[i]->lock=&ctx->dev->lock;
  q[i]->dev=ctx->dev->dev;
  int ret=vb2_queue_init(q[i]); if (ret) return ret;
 }
 return 0;
}
static void venc_work(struct work_struct *work)
{
 struct venc_dev *d=container_of(work,struct venc_dev,work);
 struct venc_ctx *ctx=READ_ONCE(d->job);
 struct vb2_v4l2_buffer *src=v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx),*dst=v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
 unsigned bytes=0; bool key=false;
 int ret=ctx->aborting?-ECANCELED:0;
 bool src_cpu=false,dst_cpu=false;
 if (!ret && src->vb2_buf.planes[0].dbuf) {
  ret=dma_buf_begin_cpu_access(src->vb2_buf.planes[0].dbuf,DMA_BIDIRECTIONAL); src_cpu=!ret;
 }
 if (!ret && dst->vb2_buf.planes[0].dbuf) {
  ret=dma_buf_begin_cpu_access(dst->vb2_buf.planes[0].dbuf,DMA_BIDIRECTIONAL); dst_cpu=!ret;
 }
 if (!ret) ret=venc_encode(ctx,src,dst,&bytes,&key);
 if (dst_cpu) dma_buf_end_cpu_access(dst->vb2_buf.planes[0].dbuf,DMA_BIDIRECTIONAL);
 if (src_cpu) dma_buf_end_cpu_access(src->vb2_buf.planes[0].dbuf,DMA_BIDIRECTIONAL);
 if (READ_ONCE(ctx->aborting)) ret=-ECANCELED;
 mutex_lock(&ctx->state_lock);
 bool last=v4l2_m2m_is_last_draining_src_buf(ctx->fh.m2m_ctx,src);
 src=v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx); dst=v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
 v4l2_m2m_buf_copy_metadata(src,dst,true);
 dst->sequence=src->sequence=ctx->frame;
 vb2_set_plane_payload(&dst->vb2_buf,0,ret?0:bytes);
 dst->flags &= ~(V4L2_BUF_FLAG_KEYFRAME|V4L2_BUF_FLAG_PFRAME|V4L2_BUF_FLAG_BFRAME);
 if (!ret) { dst->flags |= key?V4L2_BUF_FLAG_KEYFRAME:V4L2_BUF_FLAG_PFRAME; ctx->frame++; }
 else { ctx->frame=ctx->gop_frame=0; ctx->headers=false; }
 if (last) {
  static const struct v4l2_event eos={.type=V4L2_EVENT_EOS};
  dst->flags |= V4L2_BUF_FLAG_LAST;
  v4l2_m2m_mark_stopped(ctx->fh.m2m_ctx);
  v4l2_event_queue_fh(&ctx->fh,&eos);
 }
 v4l2_m2m_buf_done(src,ret?VB2_BUF_STATE_ERROR:VB2_BUF_STATE_DONE);
 v4l2_m2m_buf_done(dst,ret?VB2_BUF_STATE_ERROR:VB2_BUF_STATE_DONE);
 mutex_unlock(&ctx->state_lock);
 v4l2_m2m_job_finish(d->m2m,ctx->fh.m2m_ctx);
}
static void venc_device_run(void *priv)
{
 struct venc_ctx *ctx=priv;
 WRITE_ONCE(ctx->dev->job,ctx);
 schedule_work(&ctx->dev->work);
}
static void venc_job_abort(void *priv)
{
 struct venc_ctx *ctx=priv;
 WRITE_ONCE(ctx->aborting,true);
 flush_work(&ctx->dev->work);
}
static const struct v4l2_m2m_ops venc_m2m_ops={.device_run=venc_device_run,.job_abort=venc_job_abort};
static int venc_open(struct file *f)
{
 struct venc_dev *d=video_drvdata(f);
 struct venc_ctx *ctx=kzalloc(sizeof(*ctx),GFP_KERNEL);
 int ret;
 if (!ctx) return -ENOMEM;
 ctx->dev=d; ctx->width=320; ctx->height=240; ctx->codec=V4L2_PIX_FMT_H264;
 ctx->frame_rate=30;
 mutex_init(&ctx->state_lock);
 mutex_lock(&d->lock);
 v4l2_fh_init(&ctx->fh,&d->video); f->private_data=&ctx->fh;
 ret=venc_controls(ctx);
 if (ret) goto fail;
 ctx->fh.m2m_ctx=v4l2_m2m_ctx_init(d->m2m,ctx,venc_queue_init);
 if (IS_ERR(ctx->fh.m2m_ctx)) { ret=PTR_ERR(ctx->fh.m2m_ctx); goto fail; }
 v4l2_fh_add(&ctx->fh,f); mutex_unlock(&d->lock);
 return 0;
fail:
 v4l2_ctrl_handler_free(&ctx->controls); v4l2_fh_exit(&ctx->fh);
 mutex_unlock(&d->lock); kfree(ctx);
 return ret;
}
static int venc_release(struct file *f)
{
 struct venc_ctx *ctx=venc_fh(f->private_data);
 mutex_lock(&ctx->dev->lock);
 v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
 venc_free_buffers(ctx);
 v4l2_ctrl_handler_free(&ctx->controls); v4l2_fh_del(&ctx->fh,f); v4l2_fh_exit(&ctx->fh);
 mutex_unlock(&ctx->dev->lock); kfree(ctx);
 return 0;
}
static int venc_encoder_cmd(struct file *f,void *priv,struct v4l2_encoder_cmd *cmd)
{
 struct venc_ctx *ctx=venc_fh(file_to_v4l2_fh(f));
 struct v4l2_m2m_ctx *m2m=ctx->fh.m2m_ctx;
 static const struct v4l2_event eos={.type=V4L2_EVENT_EOS};
 int ret=v4l2_m2m_ioctl_try_encoder_cmd(f,priv,cmd);
 bool stopped;
 if (ret) return ret;
 if (!vb2_is_streaming(v4l2_m2m_get_src_vq(m2m)) ||
     !vb2_is_streaming(v4l2_m2m_get_dst_vq(m2m))) return 0;
 mutex_lock(&ctx->state_lock);
 stopped=v4l2_m2m_has_stopped(m2m);
 ret=v4l2_m2m_encoder_cmd(f,m2m,cmd);
 if (!ret && cmd->cmd==V4L2_ENC_CMD_STOP && !stopped && v4l2_m2m_has_stopped(m2m))
  v4l2_event_queue_fh(&ctx->fh,&eos);
 if (!ret && cmd->cmd==V4L2_ENC_CMD_START && stopped) {
  vb2_clear_last_buffer_dequeued(v4l2_m2m_get_dst_vq(m2m));
 }
 mutex_unlock(&ctx->state_lock);
 if (!ret && cmd->cmd==V4L2_ENC_CMD_START) v4l2_m2m_try_schedule(m2m);
 return ret;
}
static int venc_subscribe(struct v4l2_fh *fh,const struct v4l2_event_subscription *sub)
{
 if (sub->type==V4L2_EVENT_EOS) return v4l2_event_subscribe(fh,sub,0,NULL);
 return v4l2_ctrl_subscribe_event(fh,sub);
}
static const struct v4l2_ioctl_ops venc_ioctl_ops={
 .vidioc_querycap=venc_querycap,.vidioc_enum_fmt_vid_cap=venc_enum_fmt,.vidioc_enum_fmt_vid_out=venc_enum_fmt,
 .vidioc_g_fmt_vid_cap_mplane=venc_get_fmt,.vidioc_g_fmt_vid_out_mplane=venc_get_fmt,
 .vidioc_s_fmt_vid_cap_mplane=venc_set_fmt,.vidioc_s_fmt_vid_out_mplane=venc_set_fmt,
 .vidioc_try_fmt_vid_cap_mplane=venc_try_fmt,.vidioc_try_fmt_vid_out_mplane=venc_try_fmt,
 .vidioc_enum_framesizes=venc_framesizes,.vidioc_g_parm=venc_get_parm,.vidioc_s_parm=venc_set_parm,
 .vidioc_reqbufs=v4l2_m2m_ioctl_reqbufs,.vidioc_create_bufs=v4l2_m2m_ioctl_create_bufs,
 .vidioc_querybuf=v4l2_m2m_ioctl_querybuf,.vidioc_prepare_buf=v4l2_m2m_ioctl_prepare_buf,
 .vidioc_qbuf=v4l2_m2m_ioctl_qbuf,.vidioc_dqbuf=v4l2_m2m_ioctl_dqbuf,
 .vidioc_expbuf=v4l2_m2m_ioctl_expbuf,.vidioc_streamon=v4l2_m2m_ioctl_streamon,
 .vidioc_streamoff=v4l2_m2m_ioctl_streamoff,.vidioc_subscribe_event=venc_subscribe,
 .vidioc_try_encoder_cmd=v4l2_m2m_ioctl_try_encoder_cmd,.vidioc_encoder_cmd=venc_encoder_cmd,
 .vidioc_unsubscribe_event=v4l2_event_unsubscribe};
static const struct v4l2_file_operations venc_fops={.owner=THIS_MODULE,.open=venc_open,.release=venc_release,
 .unlocked_ioctl=video_ioctl2,.poll=v4l2_m2m_fop_poll,.mmap=v4l2_m2m_fop_mmap};
static irqreturn_t venc_irq(int irq,void *data)
{
 struct venc_dev *d=data;
 u32 status=readl(d->regs+0x5c);
 if (!status) return IRQ_NONE;
 for (u32 bit=1;bit<=0x80;bit<<=1) if (status&bit) writel(bit,d->regs+0x60);
 WRITE_ONCE(d->status,status); complete(&d->done);
 return IRQ_HANDLED;
}
static int venc_probe(struct platform_device *pdev)
{
 struct venc_dev *d=devm_kzalloc(&pdev->dev,sizeof(*d),GFP_KERNEL);
 struct device_node *node; struct platform_device *larb;
 int ret;
 if (!d) return -ENOMEM;
 d->dev=&pdev->dev;
 d->regs=devm_platform_ioremap_resource(pdev,0);
 if (IS_ERR(d->regs)) return PTR_ERR(d->regs);
 d->num_clks=devm_clk_bulk_get_all(d->dev,&d->clks);
 if (d->num_clks<0) return d->num_clks;
 node=of_parse_phandle(d->dev->of_node,"mediatek,larb",0);
 if (!node) return -EINVAL;
 larb=of_find_device_by_node(node); of_node_put(node);
 if (!larb) return -EPROBE_DEFER;
 d->larb=&larb->dev;
 if (!device_link_add(d->dev,d->larb,DL_FLAG_AUTOREMOVE_CONSUMER)) { put_device(d->larb); return -ENOMEM; }
 put_device(d->larb);
 ret=dma_set_mask_and_coherent(d->dev,DMA_BIT_MASK(32));
 if (ret) return ret;
 if (!device_iommu_mapped(d->dev)) return -ENODEV;
 d->irq=platform_get_irq(pdev,0);
 if (d->irq<0) return d->irq;
 mutex_init(&d->lock); init_completion(&d->done); INIT_WORK(&d->work,venc_work);
 ret=devm_request_irq(d->dev,d->irq,venc_irq,IRQF_NO_AUTOEN,dev_name(d->dev),d);
 if (ret) return ret;
 ret=v4l2_device_register(d->dev,&d->v4l2);
 if (ret) return ret;
 d->m2m=v4l2_m2m_init(&venc_m2m_ops);
 if (IS_ERR(d->m2m)) { ret=PTR_ERR(d->m2m); goto unregister; }
 strscpy(d->video.name,"mt6797-venc",sizeof(d->video.name));
 d->video.fops=&venc_fops; d->video.ioctl_ops=&venc_ioctl_ops;
 d->video.release=video_device_release_empty; d->video.v4l2_dev=&d->v4l2;
 d->video.lock=&d->lock; d->video.vfl_dir=VFL_DIR_M2M;
 d->video.device_caps=V4L2_CAP_VIDEO_M2M_MPLANE|V4L2_CAP_STREAMING;
 video_set_drvdata(&d->video,d); platform_set_drvdata(pdev,d);
 pm_runtime_enable(d->dev);
 ret=video_register_device(&d->video,VFL_TYPE_VIDEO,-1);
 if (ret) { pm_runtime_disable(d->dev); v4l2_m2m_release(d->m2m); goto unregister; }
 return 0;
unregister:
 v4l2_device_unregister(&d->v4l2);
 return ret;
}
static void venc_remove(struct platform_device *pdev)
{
 struct venc_dev *d=platform_get_drvdata(pdev);
 video_unregister_device(&d->video); flush_work(&d->work);
 v4l2_m2m_release(d->m2m); v4l2_device_unregister(&d->v4l2); pm_runtime_disable(d->dev);
}
static int venc_suspend(struct device *dev)
{
 struct venc_dev *d=dev_get_drvdata(dev);
 /* Pause admission before waiting for the current bounded hardware job. */
 v4l2_m2m_suspend(d->m2m);
 flush_work(&d->work);
 return 0;
}
static int venc_resume(struct device *dev)
{
 struct venc_dev *d=dev_get_drvdata(dev);
 v4l2_m2m_resume(d->m2m);
 return 0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(venc_pm,venc_suspend,venc_resume);
static const struct of_device_id venc_match[]={
 {.compatible="mediatek,mt6797-venc"},{}};
MODULE_DEVICE_TABLE(of,venc_match);
static struct platform_driver venc_driver={.probe=venc_probe,.remove=venc_remove,
 .driver={.name="mt6797-venc",.of_match_table=venc_match,.suppress_bind_attrs=true,.pm=pm_sleep_ptr(&venc_pm)}};
module_platform_driver(venc_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT6797 native V4L2 AVC/HEVC encoder");

MODULE_IMPORT_NS("DMA_BUF");
