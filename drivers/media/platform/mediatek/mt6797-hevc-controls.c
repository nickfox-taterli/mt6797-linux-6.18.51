// SPDX-License-Identifier: GPL-2.0-only
/* Standard HEVC syntax controls -> the MT6797 register algorithm. */
#include "mt6797-hevc.h"
struct bits {const u8 *data;unsigned bytes,source,remaining,pos,zeros;u8 byte;int error;};
static u32 get(struct bits *b,unsigned n)
{
 u32 v=0;if(n>32 || b->pos+n>32768){b->error=-EINVAL;return 0;}
 while(n-- && !b->error){
  if(!b->remaining){
   if(b->source>=b->bytes){b->error=-EINVAL;break;}
   u8 c=b->data[b->source++];
   if(b->zeros==2 && c==3){if(b->source>=b->bytes || b->data[b->source]>3){b->error=-EINVAL;break;}c=b->data[b->source++];b->zeros=0;}
   b->zeros=c==0?min(b->zeros+1,2U):0;b->byte=c;b->remaining=8;
  }
  v=(v<<1)|((b->byte>>--b->remaining)&1);b->pos++;
 }
 return v;
}
static u32 ue(struct bits *b){unsigned z=0;while(!b->error && !get(b,1))if(++z>30){b->error=-EINVAL;return 0;}return ((1U<<z)-1)+get(b,z);}
static s32 se(struct bits *b){u32 v=ue(b);return v&1?(s32)((v+1)/2):-(s32)(v/2);}
static unsigned ceil_log(unsigned n){return n>1?fls(n-1):0;}
static int locate(struct mt6797_hevc_control *p,const struct v4l2_ctrl_hevc_sps *s,
 const struct v4l2_ctrl_hevc_pps *pps,const struct v4l2_ctrl_hevc_decode_params *d,const u8 *nal,unsigned bytes)
{
 struct bits b={.data=nal,.bytes=bytes};
 unsigned h=get(&b,16),kind=(h>>9)&63;
 if((h&0x81f8) || !(h&7) || kind!=p->nal_type || !get(&b,1))return -EINVAL;
 if(kind>=16 && kind<=23)get(&b,1);
 if(ue(&b)!=pps->pic_parameter_set_id)return -EINVAL;
 get(&b,pps->num_extra_slice_header_bits);
 if(ue(&b)!=p->slice_type)return -EINVAL;
 if(pps->flags&V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT)get(&b,1);
 p->rps_neg=p->rps_pos=0;
 if(!p->is_idr){
  unsigned lsb=get(&b,s->log2_max_pic_order_cnt_lsb_minus4+4);
  if(lsb!=((u32)d->pic_order_cnt_val&((1U<<(s->log2_max_pic_order_cnt_lsb_minus4+4))-1)))return -EINVAL;
  p->pic_order_cnt_lsb=lsb;
  if(get(&b,1)){
   if(!s->num_short_term_ref_pic_sets || get(&b,ceil_log(s->num_short_term_ref_pic_sets))>=s->num_short_term_ref_pic_sets)return -EINVAL;
   for(unsigned i=0;i<d->num_active_dpb_entries;i++)if(!(d->dpb[i].flags&V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE))p->rps_neg++;
  }else if(s->num_short_term_ref_pic_sets && get(&b,1)){
   ue(&b);get(&b,1);ue(&b);
   if(d->num_delta_pocs_of_ref_rps_idx>16)return -EINVAL;
   for(unsigned i=0;i<=d->num_delta_pocs_of_ref_rps_idx;i++){
    bool used=get(&b,1);if(used || get(&b,1))p->rps_neg++;
   }
  }else{
   unsigned neg=ue(&b),pos=ue(&b);if(neg+pos>16)return -EINVAL;
   p->rps_neg=neg;p->rps_pos=pos;
   for(unsigned i=0;i<neg+pos;i++){ue(&b);get(&b,1);}
  }
  if(s->flags&V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT){
   unsigned nsps=s->num_long_term_ref_pics_sps?ue(&b):0,n=ue(&b);
   if(nsps>s->num_long_term_ref_pics_sps || n+nsps>16)return -EINVAL;
   p->num_long_term=n+nsps;p->num_long_term_sps=nsps;
   for(unsigned i=0;i<n+nsps;i++){
    if(i<nsps)get(&b,ceil_log(s->num_long_term_ref_pics_sps));
    else {get(&b,s->log2_max_pic_order_cnt_lsb_minus4+4);get(&b,1);}
    if(get(&b,1))ue(&b);
   }
  }
  if(s->flags&V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED)get(&b,1);
 }
 if(s->flags&V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET){get(&b,1);get(&b,1);}
 if(p->slice_type!=2){
  if(get(&b,1)){if(ue(&b)!=p->num_ref_idx_l0_active_minus1)return -EINVAL;if(p->slice_type==0 && ue(&b)!=p->num_ref_idx_l1_active_minus1)return -EINVAL;}
  else if(p->num_ref_idx_l0_active_minus1!=pps->num_ref_idx_l0_default_active_minus1 ||
   (p->slice_type==0 && p->num_ref_idx_l1_active_minus1!=pps->num_ref_idx_l1_default_active_minus1))return -EINVAL;
  p->rpl_start_bit=b.pos;
  if(p->lists_modification_present_flag && p->ref_table_count>1)
   for(unsigned list=0;list<(p->slice_type==0?2U:1U);list++)if(get(&b,1))
    for(unsigned i=0;i<=(list?p->num_ref_idx_l1_active_minus1:p->num_ref_idx_l0_active_minus1);i++)if(get(&b,ceil_log(p->ref_table_count))>=p->ref_table_count)return -EINVAL;
  p->rpl_end_bit=b.pos;
  if(p->slice_type==0)get(&b,1);
  if(pps->flags&V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT)get(&b,1);
  if(p->slice_temporal_mvp_enabled_flag){
   bool from0=p->slice_type==0?get(&b,1):true;
   if((from0?p->num_ref_idx_l0_active_minus1:p->num_ref_idx_l1_active_minus1)>0)ue(&b);
  }
  if((p->slice_type==1 && (pps->flags&V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED)) ||
     (p->slice_type==0 && (pps->flags&V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED))){
   p->weights_start_bit=b.pos;
   unsigned luma=ue(&b);s32 chroma=(s32)luma+se(&b);if(luma>7 || chroma<0 || chroma>7)return -EINVAL;
   for(unsigned list=0;list<(p->slice_type==0?2U:1U);list++){
    unsigned n=(list?p->num_ref_idx_l1_active_minus1:p->num_ref_idx_l0_active_minus1)+1;bool lf[16],cf[16];
    for(unsigned i=0;i<n;i++)lf[i]=get(&b,1);
    for(unsigned i=0;i<n;i++)cf[i]=get(&b,1);
    for(unsigned i=0;i<n;i++){
     if(lf[i]){s32 a=se(&b),c=se(&b);if(a< -128 || a>127 || c< -128 || c>127)return -EINVAL;}
     if(cf[i])for(unsigned j=0;j<2;j++){s32 a=se(&b),c=se(&b);if(a< -128 || a>127 || c< -512 || c>511)return -EINVAL;}
    }
   }
   p->weights_end_bit=b.pos;
  }
  if(ue(&b)>4)return -EINVAL;
 }
 se(&b);
 if(pps->flags&V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT){se(&b);se(&b);}
 bool disabled=pps->flags&V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER;
 if((pps->flags&V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED) && get(&b,1)){
  disabled=get(&b,1);if(!disabled){se(&b);se(&b);}
 }
 if((pps->flags&V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED) && (p->sao_luma || p->sao_chroma || !disabled))get(&b,1);
 if(pps->flags&(V4L2_HEVC_PPS_FLAG_TILES_ENABLED|V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED)){
  unsigned n=ue(&b);if(n>=p->lcu_w*p->lcu_h)return -EINVAL;
  if(n){unsigned bits=ue(&b)+1;if(bits>32)return -EINVAL;for(unsigned i=0;i<n;i++)get(&b,bits);}
 }
 if(pps->flags&V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT){unsigned n=ue(&b);if(n>256)return -EINVAL;for(unsigned i=0;i<n;i++)get(&b,8);}
 if(!get(&b,1))return -EINVAL;
 while(b.pos%8)if(get(&b,1))return -EINVAL;
 p->header_bit_size=b.pos;
 return b.error;
}
int mt6797_hevc_stage(struct vdec_ctx *ctx)
{
 const struct v4l2_ctrl_hevc_sps *s=vdec_control(ctx,V4L2_CID_STATELESS_HEVC_SPS,NULL);
 const struct v4l2_ctrl_hevc_pps *pps=vdec_control(ctx,V4L2_CID_STATELESS_HEVC_PPS,NULL);
 const struct v4l2_ctrl_hevc_decode_params *d=vdec_control(ctx,V4L2_CID_STATELESS_HEVC_DECODE_PARAMS,NULL);
 const struct v4l2_ctrl_hevc_scaling_matrix *sc=vdec_control(ctx,V4L2_CID_STATELESS_HEVC_SCALING_MATRIX,NULL);
 unsigned nslices;const struct v4l2_ctrl_hevc_slice_params *sl=vdec_control(ctx,V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,&nslices);
 if(s->chroma_format_idc!=1 || s->bit_depth_luma_minus8 || s->bit_depth_chroma_minus8 || s->log2_max_pic_order_cnt_lsb_minus4>12 ||
  s->pic_width_in_luma_samples!=ctx->width || s->pic_height_in_luma_samples!=ctx->height || !nslices || nslices>64 || d->num_active_dpb_entries>15)return -EINVAL;
 unsigned mincb=s->log2_min_luma_coding_block_size_minus3+3,ctb=mincb+s->log2_diff_max_min_luma_coding_block_size;
 unsigned mintb=s->log2_min_luma_transform_block_size_minus2+2,maxtb=mintb+s->log2_diff_max_min_luma_transform_block_size;
 if(mincb>ctb || ctb<4 || ctb>6 || mintb<2 || maxtb>5 || s->max_transform_hierarchy_depth_inter>7 || s->max_transform_hierarchy_depth_intra>7 || pps->diff_cu_qp_delta_depth>ctb)return -EINVAL;
 if(!ctx->hevc){ctx->hevc=kzalloc(sizeof(struct mt6797_hevc_state),GFP_KERNEL);if(!ctx->hevc)return -ENOMEM;}
 struct mt6797_hevc_control *p=&((struct mt6797_hevc_state *)ctx->hevc)->control;memset(p,0,sizeof(*p));
 p->width=ctx->width;p->height=ctx->height;p->log2_ctb=ctb;p->lcu_w=DIV_ROUND_UP(p->width,1U<<ctb);p->lcu_h=DIV_ROUND_UP(p->height,1U<<ctb);
 p->aligned_width=ALIGN(p->lcu_w<<ctb,64);p->aligned_height=ALIGN(p->lcu_h<<ctb,64);p->start_code_bits=32;p->control_version=3;
#define S(name) (!!(s->flags&V4L2_HEVC_SPS_FLAG_##name))
#define P(name) (!!(pps->flags&V4L2_HEVC_PPS_FLAG_##name))
 p->sps0=1|S(PCM_ENABLED)<<2|P(LISTS_MODIFICATION_PRESENT)<<3|S(AMP_ENABLED)<<4|S(SAMPLE_ADAPTIVE_OFFSET)<<5|S(PCM_LOOP_FILTER_DISABLED)<<6|S(SPS_TEMPORAL_MVP_ENABLED)<<7|
  (S(PCM_ENABLED)?s->pcm_sample_bit_depth_luma_minus1:7)<<8|(S(PCM_ENABLED)?s->pcm_sample_bit_depth_chroma_minus1:7)<<12|s->num_long_term_ref_pics_sps<<16|S(LONG_TERM_REF_PICS_PRESENT)<<22|s->num_short_term_ref_pic_sets<<24;
 p->sps1=(s->log2_max_pic_order_cnt_lsb_minus4+4)|(max(1U,ceil_log(s->num_short_term_ref_pic_sets))<<8)|(max(1U,ceil_log(s->num_long_term_ref_pics_sps))<<12)|S(STRONG_INTRA_SMOOTHING_ENABLED)<<16;
 unsigned pcmmin=S(PCM_ENABLED)?s->log2_min_pcm_luma_coding_block_size_minus3+3:7,pcmmax=S(PCM_ENABLED)?pcmmin+s->log2_diff_max_min_pcm_luma_coding_block_size:5;
 p->sps_size=mincb|(ctb<<4)|(mintb<<8)|(maxtb<<12)|(pcmmin<<16)|(pcmmax<<20)|(s->max_transform_hierarchy_depth_inter<<24)|(s->max_transform_hierarchy_depth_intra<<28);
 p->pps_word=P(SIGN_DATA_HIDING_ENABLED)|P(CABAC_INIT_PRESENT)<<1|P(CONSTRAINED_INTRA_PRED)<<2|P(TRANSFORM_SKIP_ENABLED)<<3|P(WEIGHTED_PRED)<<4|P(WEIGHTED_BIPRED)<<5|P(OUTPUT_FLAG_PRESENT)<<6|
  P(TRANSQUANT_BYPASS_ENABLED)<<7|P(DEPENDENT_SLICE_SEGMENT_ENABLED)<<8|P(ENTROPY_CODING_SYNC_ENABLED)<<9|P(SLICE_SEGMENT_HEADER_EXTENSION_PRESENT)<<11|
  pps->num_ref_idx_l0_default_active_minus1<<16|pps->num_ref_idx_l1_default_active_minus1<<20|(pps->log2_parallel_merge_level_minus2+2)<<24|pps->num_extra_slice_header_bits<<28;
 p->pps_qp=P(CU_QP_DELTA_ENABLED)|P(PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT)<<1|(ctb-pps->diff_cu_qp_delta_depth)<<4|((26+pps->init_qp_minus26)&63)<<8|((u32)pps->pps_cb_qp_offset&31)<<16|((u32)pps->pps_cr_qp_offset&31)<<24;
 p->pps_tile=P(TILES_ENABLED)|pps->num_tile_columns_minus1<<8|pps->num_tile_rows_minus1<<16|P(LOOP_FILTER_ACROSS_TILES_ENABLED)<<24;
 p->pps_dbk=P(PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED)|P(DEBLOCKING_FILTER_CONTROL_PRESENT)<<1|P(DEBLOCKING_FILTER_OVERRIDE_ENABLED)<<2|P(PPS_DISABLE_DEBLOCKING_FILTER)<<3|((u32)pps->pps_beta_offset_div2&31)<<4|((u32)pps->pps_tc_offset_div2&31)<<9;
 p->pps_cb_qp_offset=pps->pps_cb_qp_offset;p->pps_cr_qp_offset=pps->pps_cr_qp_offset;p->lists_modification_present_flag=P(LISTS_MODIFICATION_PRESENT);
 for(unsigned axis=0;axis<2;axis++){
  unsigned total=axis?p->lcu_h:p->lcu_w,n=P(TILES_ENABLED)?(axis?pps->num_tile_rows_minus1:pps->num_tile_columns_minus1)+1:1,edge=0;
  if(n>total || (axis?n>22:n>20))return -EINVAL;
  for(unsigned i=0;i<n;i++){
   unsigned width=P(UNIFORM_SPACING)?((i+1)*total/n-i*total/n):i+1==n?total-edge:(axis?pps->row_height_minus1[i]:pps->column_width_minus1[i])+1;
   if(!width || edge+width>total)return -EINVAL;
   edge+=width;(axis?p->tile_y:p->tile_x)[(edge-1)/32]|=BIT((edge-1)%32);
  }
 }
 p->scaling_enabled=S(SCALING_LIST_ENABLED);
 if(p->scaling_enabled){
  /* The internal rows are 64 bytes even for 4x4 lists. */
  for(unsigned i=0;i<6;i++){memcpy(p->scaling[0][i],sc->scaling_list_4x4[i],16);memcpy(p->scaling[1][i],sc->scaling_list_8x8[i],64);memcpy(p->scaling[2][i],sc->scaling_list_16x16[i],64);p->scaling_dc[2][i]=sc->scaling_list_dc_coef_16x16[i];}
  for(unsigned i=0;i<2;i++){memcpy(p->scaling[3][i*3],sc->scaling_list_32x32[i],64);p->scaling_dc[3][i*3]=sc->scaling_list_dc_coef_32x32[i];}
 }
#undef S
#undef P
 p->nal_type=sl[0].nal_unit_type;p->is_idr=p->nal_type==19 || p->nal_type==20;p->is_reference=p->nal_type>=16 || (p->nal_type&1);
 p->slice_type=sl[0].slice_type;p->top_field_order_cnt=d->pic_order_cnt_val;p->keep_ref_count=d->num_active_dpb_entries;
 p->rps_prediction_ref_count=d->num_delta_pocs_of_ref_rps_idx;
 for(unsigned i=0;i<d->num_active_dpb_entries;i++){p->keep_ref_ts[i]=d->dpb[i].timestamp;p->keep_ref_long_term[i]=!!(d->dpb[i].flags&V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE);}
 unsigned used=d->num_poc_st_curr_before+d->num_poc_st_curr_after+d->num_poc_lt_curr;
 if(used>16)return -EINVAL;
 for(unsigned i=0;i<nslices;i++){
  if(sl[i].slice_type>2 || sl[i].num_ref_idx_l0_active_minus1>15 || sl[i].num_ref_idx_l1_active_minus1>15 || sl[i].data_byte_offset>=ctx->bytes || sl[i].five_minus_max_num_merge_cand>4)return -EINVAL;
  p->picture_has_inter|=sl[i].slice_type!=2;
 }
 p->ref_table_count=p->picture_has_inter?used:0;
 if(p->picture_has_inter && !used)return -EINVAL;
 for(unsigned list=0;list<2;list++){
  unsigned at=0;
  for(unsigned part=0;part<3;part++){
   unsigned which=part==2?2:part^list,n=which==0?d->num_poc_st_curr_before:which==1?d->num_poc_st_curr_after:d->num_poc_lt_curr;
   const u8 *indices=which==0?d->poc_st_curr_before:which==1?d->poc_st_curr_after:d->poc_lt_curr;
   for(unsigned i=0;i<n;i++,at++){
    unsigned idx=indices[i];if(idx>=d->num_active_dpb_entries || at>=16)return -EINVAL;
    (list?p->l1_ref_ts:p->l0_ref_ts)[at]=d->dpb[idx].timestamp;
    (list?p->l1_poc_diff:p->l0_poc_diff)[at]=d->pic_order_cnt_val-d->dpb[idx].pic_order_cnt_val;
    (list?p->l1_long_term:p->l0_long_term)[at]=which==2;
   }
  }
 }
 p->num_ref_idx_l0_active_minus1=sl[0].num_ref_idx_l0_active_minus1;p->num_ref_idx_l1_active_minus1=sl[0].num_ref_idx_l1_active_minus1;
 p->slice_qp=26+pps->init_qp_minus26+sl[0].slice_qp_delta;if(p->slice_qp<0 || p->slice_qp>51)return -EINVAL;
 p->slice_cb_qp_offset=sl[0].slice_cb_qp_offset;p->slice_cr_qp_offset=sl[0].slice_cr_qp_offset;p->beta_offset_div2=sl[0].slice_beta_offset_div2;p->tc_offset_div2=sl[0].slice_tc_offset_div2;
#define SL(field,flag) p->field=!!(sl[0].flags&V4L2_HEVC_SLICE_PARAMS_FLAG_##flag)
 SL(sao_luma,SLICE_SAO_LUMA);SL(sao_chroma,SLICE_SAO_CHROMA);SL(cabac_init_flag,CABAC_INIT);SL(collocated_from_l0_flag,COLLOCATED_FROM_L0);SL(mvd_l1_zero_flag,MVD_L1_ZERO);SL(slice_temporal_mvp_enabled_flag,SLICE_TEMPORAL_MVP_ENABLED);SL(deblocking_filter_disabled_flag,SLICE_DEBLOCKING_FILTER_DISABLED);SL(loop_filter_across_slices_enabled_flag,SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED);
#undef SL
 p->collocated_ref_idx=sl[0].collocated_ref_idx;p->five_minus_max_num_merge_cand=sl[0].five_minus_max_num_merge_cand;p->header_bit_size=sl[0].data_byte_offset*8;
 int ret=locate(p,s,pps,d,ctx->buffers[0].cpu,ctx->bytes);if(ret)return ret;
 p->num_long_term_used=d->num_poc_lt_curr;
 unsigned short_used=d->num_poc_st_curr_before+d->num_poc_st_curr_after;
 for(unsigned i=0;i<min(short_used,p->rps_neg);i++)p->rps_neg_used[i]=1;
 for(unsigned i=p->rps_neg;i<short_used;i++)if(i-p->rps_neg<16)p->rps_pos_used[i-p->rps_neg]=1;
 return 0;
}
