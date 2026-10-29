// SPDX-License-Identifier: GPL-2.0-only
/* Stateless H.264 slice requests; hardware access stays inside this driver.
 * The first prototype admits progressive pictures and uniform slice headers.
 * PAFF/MBAFF and per-slice parameter changes remain on the existing decoder.
 */
#include <linux/interrupt.h>
#include <linux/dma-buf.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include "mt6797-vdec.h"
static inline struct vdec_ctx *vdec_fh(void *fh) { return container_of(fh,struct vdec_ctx,fh); }
static void vdec_free_buffers(struct vdec_ctx *ctx)
{
 for (unsigned i=0;i<ARRAY_SIZE(ctx->buffers);i++) {
  struct buffer *b=&ctx->buffers[i];
  if (b->cpu) dma_free_coherent(ctx->dev->dev,b->size,b->cpu,b->dma);
  memset(b,0,sizeof(*b));
 }
 memset(ctx->cache,0,sizeof(ctx->cache));
}
int vdec_buffer(struct vdec_ctx *ctx,struct buffer *b,unsigned size)
{
 b->size=PAGE_ALIGN(size); b->used=size-GUARD_SIZE;
 b->cpu=dma_alloc_coherent(ctx->dev->dev,b->size,&b->dma,GFP_KERNEL);
 if (!b->cpu) return -ENOMEM;
 memset(b->cpu,0,b->used); memset(b->cpu+b->used,0xd3,b->size-b->used);
 return 0;
}
static int vdec_alloc_buffers(struct vdec_ctx *ctx)
{
 for (unsigned i=0;i<4;i++) {
  int ret=vdec_buffer(ctx,&ctx->buffers[i],(i?WORK_SIZE:INPUT_SIZE)+GUARD_SIZE);
  if (ret) { vdec_free_buffers(ctx); return ret; }
 }
 return 0;
}
static int vdec_power_on(struct vdec_ctx *ctx)
{
 struct vdec_dev *d=ctx->dev;
 int ret=pm_runtime_resume_and_get(d->dev);
 if (ret<0) return ret;
 writel(1,d->gcon+0x10); readl(d->gcon+0x10);
 ret=pm_runtime_resume_and_get(d->larb);
 if (ret<0) goto put;
 ret=clk_bulk_prepare_enable(d->num_clks,d->clks);
 if (ret) goto larb;
 d->powered=true; enable_irq(d->irq);
 return 0;
larb:
 pm_runtime_put_sync(d->larb);
put:
 pm_runtime_put_sync(d->dev);
 return ret;
}
static void vdec_power_off(struct vdec_ctx *ctx)
{
 struct vdec_dev *d=ctx->dev; u32 idle;
 if (!d->powered) return;
 disable_irq(d->irq);
 writel(readl(d->regs+0x100)|1,d->regs+0x100);
 if (readl_poll_timeout(d->regs+0x104,idle,(idle&0x11)==0x11,1,2000))
  dev_err(d->dev,"decoder bus stop timed out\n");
 writel(0x101,d->regs+0x1108); readl(d->regs+0x1108);
 writel(0,d->gcon+0x10); readl(d->gcon+0x10);
 clk_bulk_disable_unprepare(d->num_clks,d->clks);
 pm_runtime_put_sync(d->larb); pm_runtime_put_sync(d->dev); d->powered=false;
}
static const struct v4l2_ctrl_config vdec_control_config[]={
 {.id=V4L2_CID_STATELESS_H264_SPS},{.id=V4L2_CID_STATELESS_H264_PPS},
 {.id=V4L2_CID_STATELESS_H264_SCALING_MATRIX},{.id=V4L2_CID_STATELESS_H264_DECODE_PARAMS},
 {.id=V4L2_CID_STATELESS_H264_SLICE_PARAMS},{.id=V4L2_CID_STATELESS_H264_PRED_WEIGHTS},
 {.id=V4L2_CID_STATELESS_H264_DECODE_MODE,.min=V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
 .max=V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,.def=V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED},
 {.id=V4L2_CID_STATELESS_H264_START_CODE,.min=V4L2_STATELESS_H264_START_CODE_NONE,
 .max=V4L2_STATELESS_H264_START_CODE_NONE,.def=V4L2_STATELESS_H264_START_CODE_NONE},
};
static int vdec_controls(struct vdec_ctx *ctx)
{
 v4l2_ctrl_handler_init(&ctx->controls,ARRAY_SIZE(vdec_control_config));
 for (unsigned i=0;i<ARRAY_SIZE(vdec_control_config);i++)
  ctx->control[i]=v4l2_ctrl_new_custom(&ctx->controls,&vdec_control_config[i],NULL);
 ctx->fh.ctrl_handler=&ctx->controls;
 return ctx->controls.error;
}
static void vdec_read_control(struct vdec_ctx *ctx,unsigned id,void *dest,unsigned size)
{
 for (unsigned i=0;i<ARRAY_SIZE(ctx->control);i++)
  if (ctx->control[i]->id==id) { memcpy(dest,ctx->control[i]->p_cur.p,size); return; }
 WARN_ON_ONCE(1);
}
static int vdec_stage_slice(struct vdec_ctx *ctx,struct vb2_v4l2_buffer *src)
{
 struct control *p=&ctx->picture;
 struct v4l2_ctrl_h264_slice_params sl;
 struct v4l2_ctrl_h264_decode_params dec;
 struct v4l2_ctrl_h264_sps sps;
 struct v4l2_ctrl_h264_pps pps;
 const u8 *data=vb2_plane_vaddr(&src->vb2_buf,0);
 unsigned len=vb2_get_plane_payload(&src->vb2_buf,0);
 int ret;
 vdec_read_control(ctx,V4L2_CID_STATELESS_H264_SLICE_PARAMS,&sl,sizeof(sl));
 vdec_read_control(ctx,V4L2_CID_STATELESS_H264_DECODE_PARAMS,&dec,sizeof(dec));
 vdec_read_control(ctx,V4L2_CID_STATELESS_H264_SPS,&sps,sizeof(sps));
 vdec_read_control(ctx,V4L2_CID_STATELESS_H264_PPS,&pps,sizeof(pps));
 if (!data || len<4 || len>INPUT_SIZE-512 || ctx->bytes+len+4>INPUT_SIZE-512 || ctx->slices>=64) return -EINVAL;
 if (!(sps.flags&V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) ||
     (dec.flags&V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC)) return -EOPNOTSUPP;
 if (mt6797_h264_cabac_alignment(&pps,&sl,data,len)) return -EINVAL;
 if (!ctx->bytes) {
  if (sl.first_mb_in_slice || sps.pic_width_in_mbs_minus1+1!=ctx->width/16 ||
      sps.pic_height_in_map_units_minus1+1!=DIV_ROUND_UP(ctx->height,16)) return -EINVAL;
  ctx->native_decode=dec;
  memset(p,0,sizeof(*p)); p->sps=sps; p->pps=pps; p->slice=sl;
  memcpy(p->decode.dpb,dec.dpb,sizeof(dec.dpb));
  static_assert(sizeof(p->decode)-offsetof(struct mt6797_h264_decode_params,nal_ref_idc)==
                sizeof(dec)-offsetof(struct v4l2_ctrl_h264_decode_params,nal_ref_idc));
  memcpy(&p->decode.nal_ref_idc,&dec.nal_ref_idc,sizeof(dec)-offsetof(struct v4l2_ctrl_h264_decode_params,nal_ref_idc));
  p->width=ctx->width; p->height=ctx->height; p->reference_ts=src->vb2_buf.timestamp;
  ctx->timestamp=src->vb2_buf.timestamp; ctx->first_bytes=len;
  vdec_read_control(ctx,V4L2_CID_STATELESS_H264_PRED_WEIGHTS,&p->weights,sizeof(p->weights));
  vdec_read_control(ctx,V4L2_CID_STATELESS_H264_SCALING_MATRIX,&p->scaling,sizeof(p->scaling));
  if ((data[0]&0x80) || ((data[0]>>5)&3)!=dec.nal_ref_idc ||
      ((data[0]&31)!=1 && (data[0]&31)!=5) ||
      (!!(dec.flags&V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC)!=((data[0]&31)==5))) return -EINVAL;
  if (V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(&pps,&sl)) {
   ret=mt6797_h264_weight_range(&sps,&pps,&sl,&p->decode,&p->weights,data,len,&p->weight_range);
   if (ret) return ret;
  }
  memset(ctx->buffers[0].cpu,0,INPUT_SIZE);
 } else {
  if (ctx->timestamp!=src->vb2_buf.timestamp || memcmp(&ctx->native_decode,&dec,sizeof(dec)) || memcmp(&p->sps,&sps,sizeof(sps)) ||
      memcmp(&p->pps,&pps,sizeof(pps)) || sl.first_mb_in_slice<=ctx->last_mb ||
      sl.first_mb_in_slice>=ctx->width*DIV_ROUND_UP(ctx->height,16)/16) return -EINVAL;
  ret=mt6797_h264_slice_headers_match(&p->slice,ctx->buffers[0].cpu,ctx->first_bytes,&sl,data,len);
  if (ret) return ret;
  memcpy(ctx->buffers[0].cpu+ctx->bytes,"\0\0\0\1",4); ctx->bytes+=4;
 }
 memcpy(ctx->buffers[0].cpu+ctx->bytes,data,len); ctx->bytes+=len; ctx->slices++; ctx->last_mb=sl.first_mb_in_slice;
 return 0;
}
static int vdec_querycap(struct file *f,void *priv,struct v4l2_capability *cap)
{
 strscpy(cap->driver,"mt6797-vdec",sizeof(cap->driver));
 strscpy(cap->card,"MT6797 stateless H264 decoder",sizeof(cap->card));
 strscpy(cap->bus_info,"platform:mt6797-vdec",sizeof(cap->bus_info));
 return 0;
}
static int vdec_enum_fmt(struct file *f,void *priv,struct v4l2_fmtdesc *fmt)
{
 if (V4L2_TYPE_IS_OUTPUT(fmt->type)) {
  if (fmt->index) return -EINVAL;
  fmt->pixelformat=V4L2_PIX_FMT_H264_SLICE;
 fmt->flags=V4L2_FMT_FLAG_COMPRESSED;
 } else {
  if (fmt->index) return -EINVAL;
  fmt->pixelformat=V4L2_PIX_FMT_MM21;

 }
 return 0;
}
static void vdec_buf_queue(struct vb2_buffer *vb)
{
 struct vdec_ctx *ctx=vb2_get_drv_priv(vb->vb2_queue);
 v4l2_m2m_buf_queue(ctx->fh.m2m_ctx,to_vb2_v4l2_buffer(vb));
}
static int vdec_try_fmt(struct file *f,void *priv,struct v4l2_format *fmt)
{
 struct v4l2_pix_format_mplane *p=&fmt->fmt.pix_mp;
 p->width=clamp(ALIGN(p->width,16),64U,1920U);
 p->height=clamp(ALIGN(p->height,16),64U,1088U);
 p->field=V4L2_FIELD_NONE; p->num_planes=1; p->colorspace=V4L2_COLORSPACE_REC709;
 p->pixelformat=V4L2_TYPE_IS_OUTPUT(fmt->type)?V4L2_PIX_FMT_H264_SLICE:V4L2_PIX_FMT_MM21;
 memset(p->plane_fmt,0,sizeof(p->plane_fmt));
 p->plane_fmt[0].sizeimage=V4L2_TYPE_IS_OUTPUT(fmt->type)?INPUT_SIZE:p->width*ALIGN(p->height,32)*3/2;
 p->plane_fmt[0].bytesperline=V4L2_TYPE_IS_OUTPUT(fmt->type)?0:p->width;
 return 0;
}
static int vdec_get_fmt(struct file *f,void *priv,struct v4l2_format *fmt)
{
 struct vdec_ctx *ctx=vdec_fh(file_to_v4l2_fh(f));
 fmt->fmt.pix_mp.width=ctx->width; fmt->fmt.pix_mp.height=ctx->height;
 return vdec_try_fmt(f,priv,fmt);
}
static int vdec_set_fmt(struct file *f,void *priv,struct v4l2_format *fmt)
{
 struct vdec_ctx *ctx=vdec_fh(file_to_v4l2_fh(f));
 if (vb2_is_busy(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx)) || vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx))) return -EBUSY;
 vdec_try_fmt(f,priv,fmt);
 ctx->width=fmt->fmt.pix_mp.width; ctx->height=fmt->fmt.pix_mp.height;
 return 0;
}
static int vdec_framesizes(struct file *f,void *priv,struct v4l2_frmsizeenum *fs)
{
 if (fs->index || (fs->pixel_format!=V4L2_PIX_FMT_H264_SLICE && fs->pixel_format!=V4L2_PIX_FMT_MM21)) return -EINVAL;
 fs->type=V4L2_FRMSIZE_TYPE_STEPWISE;
 fs->stepwise=(struct v4l2_frmsize_stepwise){64,1920,16,64,1088,16}; return 0;
}
static int vdec_queue_setup(struct vb2_queue *q,unsigned *count,unsigned *planes,unsigned sizes[],struct device *devs[])
{
 struct vdec_ctx *ctx=vb2_get_drv_priv(q);
 unsigned size=V4L2_TYPE_IS_OUTPUT(q->type)?INPUT_SIZE:ctx->width*ALIGN(ctx->height,32)*3/2;
 if (*planes) return *planes==1 && sizes[0]>=size?0:-EINVAL;
 *planes=1; sizes[0]=size; return 0;
}
static int vdec_buf_prepare(struct vb2_buffer *vb)
{
 struct vdec_ctx *ctx=vb2_get_drv_priv(vb->vb2_queue);
 unsigned size=V4L2_TYPE_IS_OUTPUT(vb->type)?INPUT_SIZE:ctx->width*ALIGN(ctx->height,32)*3/2;
 if (vb2_plane_size(vb,0)<size || vb->planes[0].data_offset) return -EINVAL;
 if (V4L2_TYPE_IS_OUTPUT(vb->type) && (vb2_get_plane_payload(vb,0)<4 || vb2_get_plane_payload(vb,0)>INPUT_SIZE-512)) return -EINVAL;
 return 0;
}
static void vdec_buf_request_complete(struct vb2_buffer *vb)
{
 struct vdec_ctx *ctx=vb2_get_drv_priv(vb->vb2_queue);
 v4l2_ctrl_request_complete(vb->req_obj.req,&ctx->controls);
}
static int vdec_start_streaming(struct vb2_queue *q,unsigned count)
{
 struct vdec_ctx *ctx=vb2_get_drv_priv(q);
 int ret=0;
 ctx->aborting=false;
 if (!ctx->buffers[0].cpu) {
  ctx->bytes=ctx->slices=ctx->sequence=0;

  ret=vdec_alloc_buffers(ctx);
 }
 if (ret) {
  struct vb2_v4l2_buffer *b;
  while ((b=V4L2_TYPE_IS_OUTPUT(q->type)?v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx):v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
   v4l2_m2m_buf_done(b,VB2_BUF_STATE_QUEUED);
 }
 return ret;
}
static void vdec_stop_streaming(struct vb2_queue *q)
{
 struct vdec_ctx *ctx=vb2_get_drv_priv(q);
 struct vb2_v4l2_buffer *b;
 struct vb2_queue *other=V4L2_TYPE_IS_OUTPUT(q->type)?v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx):v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx);
 ctx->aborting=true;
 flush_work(&ctx->dev->work);
 while ((b=V4L2_TYPE_IS_OUTPUT(q->type)?v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx):v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
  v4l2_m2m_buf_done(b,VB2_BUF_STATE_ERROR);
 ctx->bytes=ctx->slices=0;
 if (!vb2_is_streaming(other)) vdec_free_buffers(ctx);
}
static int vdec_buf_out_validate(struct vb2_buffer *vb)
{
 to_vb2_v4l2_buffer(vb)->field=V4L2_FIELD_NONE;
 return 0;
}
static const struct vb2_ops vdec_vb2_ops={.queue_setup=vdec_queue_setup,.buf_prepare=vdec_buf_prepare,
 .buf_queue=vdec_buf_queue,.start_streaming=vdec_start_streaming,.stop_streaming=vdec_stop_streaming,
 .buf_request_complete=vdec_buf_request_complete,.buf_out_validate=vdec_buf_out_validate,.wait_prepare=vb2_ops_wait_prepare,.wait_finish=vb2_ops_wait_finish};
static int vdec_queue_init(void *priv,struct vb2_queue *src,struct vb2_queue *dst)
{
 struct vdec_ctx *ctx=priv;
 struct vb2_queue *q[2]={src,dst};
 for (unsigned i=0;i<2;i++) {
  q[i]->type=i?V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  q[i]->io_modes=VB2_MMAP|VB2_DMABUF;
  q[i]->drv_priv=ctx; q[i]->buf_struct_size=sizeof(struct v4l2_m2m_buffer);
  q[i]->ops=&vdec_vb2_ops; q[i]->mem_ops=&vb2_dma_contig_memops;
  q[i]->timestamp_flags=V4L2_BUF_FLAG_TIMESTAMP_COPY; q[i]->lock=&ctx->dev->lock;
  q[i]->dev=ctx->dev->dev;
  if (!i) { q[i]->supports_requests=true; q[i]->requires_requests=true;
    q[i]->subsystem_flags=VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF; }
  int ret=vb2_queue_init(q[i]); if (ret) return ret;
 }
 return 0;
}
static void vdec_work(struct work_struct *work)
{
 struct vdec_dev *d=container_of(work,struct vdec_dev,work);
 struct vdec_ctx *ctx=READ_ONCE(d->job);
 struct vb2_v4l2_buffer *src=v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx),*dst=v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
 struct media_request *request=src->vb2_buf.req_obj.req;
 int ret=ctx->aborting?-ECANCELED:v4l2_ctrl_request_setup(request,&ctx->controls);
 bool src_cpu=false,dst_cpu=false;
 if (!ret && src->vb2_buf.planes[0].dbuf) {
  ret=dma_buf_begin_cpu_access(src->vb2_buf.planes[0].dbuf,DMA_BIDIRECTIONAL); src_cpu=!ret;
 }
 if (!ret && dst->vb2_buf.planes[0].dbuf) {
  ret=dma_buf_begin_cpu_access(dst->vb2_buf.planes[0].dbuf,DMA_BIDIRECTIONAL); dst_cpu=!ret;
 }
 if (!ret) {
  mutex_lock(ctx->controls.lock);
  ret=vdec_stage_slice(ctx,src);
  mutex_unlock(ctx->controls.lock);
 }
 v4l2_m2m_buf_copy_metadata(src,dst,true);
 dst->sequence=ctx->sequence;
 if (!ret && !(src->flags&V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF)) {
  ret=vdec_power_on(ctx);
  if (!ret) ret=mt6797_h264_run(ctx,dst);
  vdec_power_off(ctx);
  ctx->bytes=ctx->slices=0; ctx->sequence++;
 }
 if (dst_cpu) dma_buf_end_cpu_access(dst->vb2_buf.planes[0].dbuf,DMA_BIDIRECTIONAL);
 if (src_cpu) dma_buf_end_cpu_access(src->vb2_buf.planes[0].dbuf,DMA_BIDIRECTIONAL);
 if (READ_ONCE(ctx->aborting)) ret=-ECANCELED;
 if (ret) {
  src->flags &= ~V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;
  ctx->bytes=ctx->slices=0;
  memset(ctx->cache,0,sizeof(ctx->cache)); vb2_set_plane_payload(&dst->vb2_buf,0,0);
 }
 v4l2_ctrl_request_complete(request,&ctx->controls);
 v4l2_m2m_buf_done_and_job_finish(d->m2m,ctx->fh.m2m_ctx,ret?VB2_BUF_STATE_ERROR:VB2_BUF_STATE_DONE);
}
static void vdec_device_run(void *priv)
{
 struct vdec_ctx *ctx=priv;
 WRITE_ONCE(ctx->dev->job,ctx); schedule_work(&ctx->dev->work);
}
static void vdec_job_abort(void *priv)
{
 struct vdec_ctx *ctx=priv;
 WRITE_ONCE(ctx->aborting,true); flush_work(&ctx->dev->work);
}
static const struct v4l2_m2m_ops vdec_m2m_ops={.device_run=vdec_device_run,.job_abort=vdec_job_abort};
static int vdec_request_validate(struct media_request *request)
{
 unsigned count=vb2_request_buffer_cnt(request);
 if (count!=1) return count?-EINVAL:-ENOENT;
 return vb2_request_validate(request);
}
static const struct media_device_ops vdec_media_ops={.req_validate=vdec_request_validate,.req_queue=v4l2_m2m_request_queue};
static int vdec_open(struct file *f)
{
 struct vdec_dev *d=video_drvdata(f);
 struct vdec_ctx *ctx=kzalloc(sizeof(*ctx),GFP_KERNEL);
 int ret;
 if (!ctx) return -ENOMEM;
 ctx->dev=d; ctx->width=320; ctx->height=240;

 mutex_lock(&d->lock);
 v4l2_fh_init(&ctx->fh,&d->video); f->private_data=&ctx->fh;
 ret=vdec_controls(ctx);
 if (ret) goto fail;
 ctx->fh.m2m_ctx=v4l2_m2m_ctx_init(d->m2m,ctx,vdec_queue_init);
 if (IS_ERR(ctx->fh.m2m_ctx)) { ret=PTR_ERR(ctx->fh.m2m_ctx); goto fail; }
 v4l2_fh_add(&ctx->fh,f); mutex_unlock(&d->lock);
 return 0;
fail:
 v4l2_ctrl_handler_free(&ctx->controls); v4l2_fh_exit(&ctx->fh);
 mutex_unlock(&d->lock); kfree(ctx);
 return ret;
}
static int vdec_release(struct file *f)
{
 struct vdec_ctx *ctx=vdec_fh(f->private_data);
 mutex_lock(&ctx->dev->lock);
 v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
 vdec_free_buffers(ctx);
 v4l2_ctrl_handler_free(&ctx->controls); v4l2_fh_del(&ctx->fh,f); v4l2_fh_exit(&ctx->fh);
 mutex_unlock(&ctx->dev->lock); kfree(ctx);
 return 0;
}
static const struct v4l2_ioctl_ops vdec_ioctl_ops={
 .vidioc_querycap=vdec_querycap,.vidioc_enum_fmt_vid_cap=vdec_enum_fmt,.vidioc_enum_fmt_vid_out=vdec_enum_fmt,
 .vidioc_g_fmt_vid_cap_mplane=vdec_get_fmt,.vidioc_g_fmt_vid_out_mplane=vdec_get_fmt,
 .vidioc_s_fmt_vid_cap_mplane=vdec_set_fmt,.vidioc_s_fmt_vid_out_mplane=vdec_set_fmt,
 .vidioc_try_fmt_vid_cap_mplane=vdec_try_fmt,.vidioc_try_fmt_vid_out_mplane=vdec_try_fmt,
 .vidioc_enum_framesizes=vdec_framesizes,
 .vidioc_reqbufs=v4l2_m2m_ioctl_reqbufs,.vidioc_create_bufs=v4l2_m2m_ioctl_create_bufs,
 .vidioc_querybuf=v4l2_m2m_ioctl_querybuf,.vidioc_prepare_buf=v4l2_m2m_ioctl_prepare_buf,
 .vidioc_qbuf=v4l2_m2m_ioctl_qbuf,.vidioc_dqbuf=v4l2_m2m_ioctl_dqbuf,
 .vidioc_expbuf=v4l2_m2m_ioctl_expbuf,.vidioc_streamon=v4l2_m2m_ioctl_streamon,
 .vidioc_streamoff=v4l2_m2m_ioctl_streamoff,.vidioc_subscribe_event=v4l2_ctrl_subscribe_event,
 .vidioc_unsubscribe_event=v4l2_event_unsubscribe};
static const struct v4l2_file_operations vdec_fops={.owner=THIS_MODULE,.open=vdec_open,.release=vdec_release,
 .unlocked_ioctl=video_ioctl2,.poll=v4l2_m2m_fop_poll,.mmap=v4l2_m2m_fop_mmap};
static irqreturn_t vdec_irq(int irq,void *data)
{
 struct vdec_dev *d=data;
 u32 status=readl(d->regs+0xa4);
 if (!(status&BIT(16))) return IRQ_NONE;
 writel(status|0x11,d->regs+0xa4); writel((status|0x11)&~0x10,d->regs+0xa4);
 WRITE_ONCE(d->status,status); complete(&d->done);
 return IRQ_HANDLED;
}
static int vdec_probe(struct platform_device *pdev)
{
 struct vdec_dev *d=devm_kzalloc(&pdev->dev,sizeof(*d),GFP_KERNEL);
 struct device_node *node; struct platform_device *larb;
 int ret;
 if (!d) return -ENOMEM;
 d->dev=&pdev->dev; d->regs=devm_platform_ioremap_resource(pdev,0);
 if (IS_ERR(d->regs)) return PTR_ERR(d->regs);
 d->gcon=devm_ioremap(d->dev,0x16000000,PAGE_SIZE);
 if (!d->gcon) return -ENOMEM;
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
 d->irq=platform_get_irq(pdev,0); if (d->irq<0) return d->irq;
 mutex_init(&d->lock); init_completion(&d->done); INIT_WORK(&d->work,vdec_work);
 ret=devm_request_irq(d->dev,d->irq,vdec_irq,IRQF_NO_AUTOEN,dev_name(d->dev),d);
 if (ret) return ret;
 ret=v4l2_device_register(d->dev,&d->v4l2); if (ret) return ret;
 d->m2m=v4l2_m2m_init(&vdec_m2m_ops);
 if (IS_ERR(d->m2m)) { ret=PTR_ERR(d->m2m); goto unregister; }
 d->media.dev=d->dev; strscpy(d->media.model,"mt6797-vdec",sizeof(d->media.model));
 strscpy(d->media.bus_info,"platform:mt6797-vdec",sizeof(d->media.bus_info));
 media_device_init(&d->media); d->media.ops=&vdec_media_ops; d->v4l2.mdev=&d->media;
 strscpy(d->video.name,"mt6797-vdec",sizeof(d->video.name));
 d->video.fops=&vdec_fops; d->video.ioctl_ops=&vdec_ioctl_ops;
 d->video.release=video_device_release_empty; d->video.v4l2_dev=&d->v4l2;
 d->video.lock=&d->lock; d->video.vfl_dir=VFL_DIR_M2M;
 d->video.device_caps=V4L2_CAP_VIDEO_M2M_MPLANE|V4L2_CAP_STREAMING;
 video_set_drvdata(&d->video,d); platform_set_drvdata(pdev,d);
 pm_runtime_enable(d->dev);
 ret=video_register_device(&d->video,VFL_TYPE_VIDEO,-1); if (ret) goto cleanup;
 ret=v4l2_m2m_register_media_controller(d->m2m,&d->video,MEDIA_ENT_F_PROC_VIDEO_DECODER);
 if (ret) goto video;
 ret=media_device_register(&d->media); if (ret) goto mc;
 return 0;
mc:
 v4l2_m2m_unregister_media_controller(d->m2m);
video:
 video_unregister_device(&d->video);
cleanup:
 pm_runtime_disable(d->dev); media_device_cleanup(&d->media); v4l2_m2m_release(d->m2m);
unregister:
 v4l2_device_unregister(&d->v4l2); return ret;
}
static void vdec_remove(struct platform_device *pdev)
{
 struct vdec_dev *d=platform_get_drvdata(pdev);
 media_device_unregister(&d->media); v4l2_m2m_unregister_media_controller(d->m2m);
 video_unregister_device(&d->video); flush_work(&d->work);
 v4l2_m2m_release(d->m2m); v4l2_device_unregister(&d->v4l2);
 media_device_cleanup(&d->media); pm_runtime_disable(d->dev);
}
static int vdec_suspend(struct device *dev)
{
 struct vdec_dev *d=dev_get_drvdata(dev);
 return work_busy(&d->work)||READ_ONCE(d->powered)?-EBUSY:0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(vdec_pm,vdec_suspend,NULL);
/* Explicit opt-in: production's legacy decoder keeps HEVC/VP9 and field coverage. */
static const struct of_device_id vdec_match[]={{.compatible="mediatek,mt6797-vdec"},{}};
MODULE_DEVICE_TABLE(of,vdec_match);
static struct platform_driver vdec_driver={.probe=vdec_probe,.remove=vdec_remove,
 .driver={.name="mt6797-vdec",.of_match_table=vdec_match,.suppress_bind_attrs=true,.pm=pm_sleep_ptr(&vdec_pm)}};
module_platform_driver(vdec_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT6797 stateless H264 slice Request prototype");

MODULE_IMPORT_NS("DMA_BUF");
