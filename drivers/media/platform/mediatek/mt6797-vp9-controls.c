// SPDX-License-Identifier: GPL-2.0-only
#include "mt6797-vp9.h"
int mt6797_vp9_stage(struct vdec_ctx *ctx)
{
 const struct v4l2_ctrl_vp9_frame *f=vdec_control(ctx,V4L2_CID_STATELESS_VP9_FRAME,NULL);
 unsigned w=f->frame_width_minus_1+1,h=f->frame_height_minus_1+1;
 u8 first=ctx->buffers[0].cpu[0];
 if((first&0xf8)!=0x80 || !!(first&4)==!!(f->flags&V4L2_VP9_FRAME_FLAG_KEY_FRAME) ||
    !!(first&2)!=!!(f->flags&V4L2_VP9_FRAME_FLAG_SHOW_FRAME) || !!(first&1)!=!!(f->flags&V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT))return -EINVAL;
 if(f->profile || f->bit_depth!=8 || (f->flags&(V4L2_VP9_FRAME_FLAG_X_SUBSAMPLING|V4L2_VP9_FRAME_FLAG_Y_SUBSAMPLING))!=(V4L2_VP9_FRAME_FLAG_X_SUBSAMPLING|V4L2_VP9_FRAME_FLAG_Y_SUBSAMPLING))return -EOPNOTSUPP;
 if(w>ctx->width || h>ctx->height || w>1920 || h>1088 || f->frame_context_idx>3 || f->reset_frame_context>2 || f->interpolation_filter>4 || f->reference_mode>2 || f->tile_cols_log2>6 || f->tile_rows_log2>2 ||
  !f->uncompressed_header_size || !f->compressed_header_size || f->uncompressed_header_size+f->compressed_header_size>=ctx->bytes || f->lf.level>63 || f->lf.sharpness>7 ||
  f->quant.delta_q_y_dc< -15 || f->quant.delta_q_y_dc>15 || f->quant.delta_q_uv_dc< -15 || f->quant.delta_q_uv_dc>15 || f->quant.delta_q_uv_ac< -15 || f->quant.delta_q_uv_ac>15)return -EINVAL;
 if(!ctx->vp9){
  struct mt6797_vp9_state *v=kzalloc(sizeof(*v),GFP_KERNEL);if(!v)return -ENOMEM;
  for(unsigned i=0;i<4;i++)v->probs[i]=v4l2_vp9_default_probs;
  ctx->vp9=v;
 }
 struct mt6797_vp9_state *v=ctx->vp9;struct mt6797_vp9_control *c=&v->control;memset(c,0,sizeof(*c));
 c->width=w;c->height=h;c->bytes=ctx->bytes;c->uncompressed_size=f->uncompressed_header_size;c->compressed_size=f->compressed_header_size;c->tile_offset=c->uncompressed_size+c->compressed_size;
 c->frame_type=!(f->flags&V4L2_VP9_FRAME_FLAG_KEY_FRAME);c->intra_only=!!(f->flags&V4L2_VP9_FRAME_FLAG_INTRA_ONLY);
 c->error_resilient=!!(f->flags&V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT);c->parallel=!!(f->flags&V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE);
 c->refresh_context=!!(f->flags&V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX);c->high_precision_mv=!!(f->flags&V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV);
 c->reset_context_idx=f->frame_context_idx;c->reset_frame_context=f->reset_frame_context==V4L2_VP9_RESET_FRAME_CTX_ALL?3:f->reset_frame_context==V4L2_VP9_RESET_FRAME_CTX_SPEC?2:0;
 c->context_idx=(!c->frame_type || c->intra_only || c->error_resilient)?0:f->frame_context_idx;
 c->filter_level=f->lf.level;c->sharpness=f->lf.sharpness;c->delta_enabled=!!(f->lf.flags&V4L2_VP9_LOOP_FILTER_FLAG_DELTA_ENABLED);
 for(unsigned i=0;i<4;i++)c->ref_deltas|=(u32)(u8)f->lf.ref_deltas[i]<<(i*8);
 for(unsigned i=0;i<2;i++)c->mode_deltas|=(u32)(u8)f->lf.mode_deltas[i]<<(i*8);
 c->qindex=f->quant.base_q_idx;c->y_dc_delta=f->quant.delta_q_y_dc;c->uv_dc_delta=f->quant.delta_q_uv_dc;c->uv_ac_delta=f->quant.delta_q_uv_ac;
 c->interp_filter=f->interpolation_filter;c->sign_bias=f->ref_frame_sign_bias<<1;c->tile_cols_log2=f->tile_cols_log2;c->tile_rows_log2=f->tile_rows_log2;
 c->seg_enabled=!!(f->seg.flags&V4L2_VP9_SEGMENTATION_FLAG_ENABLED);c->seg_update_map=!!(f->seg.flags&V4L2_VP9_SEGMENTATION_FLAG_UPDATE_MAP);c->seg_temporal=!!(f->seg.flags&V4L2_VP9_SEGMENTATION_FLAG_TEMPORAL_UPDATE);
 c->seg_update_data=!!(f->seg.flags&V4L2_VP9_SEGMENTATION_FLAG_UPDATE_DATA);c->seg_abs=!!(f->seg.flags&V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE);
 for(unsigned i=0;i<7;i++)if(i<4)c->seg_tree0|=(u32)f->seg.tree_probs[i]<<(i*8);else c->seg_tree1|=(u32)f->seg.tree_probs[i]<<((i-4)*8);
 for(unsigned i=0;i<3;i++)c->seg_pred|=(u32)f->seg.pred_probs[i]<<(i*8);
 for(unsigned i=0;i<8;i++){
  const s16 *data=f->seg.feature_data[i];unsigned mask=f->seg.feature_enabled[i];
  if(mask>15 || data[0]< -255 || data[0]>255 || data[1]< -63 || data[1]>63 || data[2]<0 || data[2]>3)return -EINVAL;
  c->seg_feature[i]=((u32)data[0]&511)|(((u32)data[1]&127)<<9)|((u32)data[2]<<16)|(mask<<24);
 }
 c->last_frame_type=v->valid?v->last_type:0;c->size_changed=v->valid && (w!=v->last_width || h!=v->last_height);
 bool inter=c->frame_type && !c->intra_only;
 c->use_prev_mi=inter && !c->error_resilient && !c->size_changed && v->valid && v->last_show && v->last_type && !v->last_intra;
 if(c->error_resilient && (!c->parallel || c->refresh_context))return -EINVAL;
 if(inter){
  u64 ts[3]={f->last_frame_ts,f->golden_frame_ts,f->alt_frame_ts};
  for(unsigned i=0;i<3;i++){
   unsigned slot;for(slot=0;slot<MAX_SLOTS;slot++)if(v->timestamps[slot]==ts[i] && ts[i])break;
   if(slot==MAX_SLOTS)return -ENOENT;
   c->ref_width[i]=v->width[slot];c->ref_height[i]=v->height[slot];
   if(c->ref_width[i]>2*w || c->ref_height[i]>2*h || w>16*c->ref_width[i] || h>16*c->ref_height[i])return -EINVAL;
  }
 }else for(unsigned i=0;i<3;i++){c->ref_width[i]=w;c->ref_height[i]=h;}
 return 0;
}
