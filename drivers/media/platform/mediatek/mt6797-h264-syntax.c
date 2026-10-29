// SPDX-License-Identifier: GPL-2.0-only
/* Bounded H.264 header translation for stateless slice Requests. */
#ifdef __KERNEL__
#include "mt6797-vdec.h"
#else
#include <errno.h>
#include <stdbool.h>
#define MT6797_H264_SYNTAX_ONLY
#include "h264-job.h"
typedef uint8_t u8;
typedef uint32_t u32;
typedef int32_t s32;
typedef int64_t s64;
#define min(a,b) ((a)<(b)?(a):(b))
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define swap(a,b) do { unsigned temporary=(a);(a)=(b);(b)=temporary; } while(0)
#endif

struct reader { const u8 *data; unsigned bytes, source, bits, zeros, pos; u8 byte; int error; };
static u32 get(struct reader *r,unsigned count)
{
 u32 value=0;
 if (count>32 || r->pos+count>16384) { r->error=-EINVAL;return 0; }
 while (count-- && !r->error) {
  if (!r->bits) {
   if (r->source>=r->bytes) { r->error=-EINVAL;break; }
   u8 b=r->data[r->source++];
   if (r->zeros==2 && b==3) {
    if (r->source>=r->bytes || r->data[r->source]>3) { r->error=-EINVAL;break; }
    b=r->data[r->source++];r->zeros=0;
   }
   r->zeros=b==0?min(r->zeros+1,2U):0;r->byte=b;r->bits=8;
  }
  value=(value<<1)|((r->byte>>--r->bits)&1);r->pos++;
 }
 return value;
}
static u32 ue(struct reader *r)
{
 unsigned zeros=0;
 while (!r->error && !get(r,1)) if (++zeros>30) {r->error=-ERANGE;return 0;}
 return ((1U<<zeros)-1)+get(r,zeros);
}
static s32 se(struct reader *r) {u32 v=ue(r);return v&1?(s32)((v+1)/2):-(s32)(v/2);}
static void same(struct reader *r,s64 a,s64 b) {if(a!=b)r->error=-EINVAL;}
static s32 poc(const struct v4l2_h264_dpb_entry *e)
{return e->fields==V4L2_H264_BOTTOM_FIELD_REF?e->bottom_field_order_cnt:e->top_field_order_cnt;}
static bool before(const struct v4l2_h264_dpb_entry *a,const struct v4l2_h264_dpb_entry *b,int list,s32 picture_poc)
{
 bool al=a->flags&V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM,bl=b->flags&V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM;
 if(al!=bl)return !al;
 if(al)return a->pic_num<=b->pic_num;
 if(!list)return (s32)a->pic_num>=(s32)b->pic_num;
 s32 ap=poc(a),bp=poc(b);
 bool aside=list==1?ap>=picture_poc:ap<picture_poc,bside=list==1?bp>=picture_poc:bp<picture_poc;
 return aside!=bside?aside<bside:(ap>=picture_poc?ap<=bp:ap>=bp);
}
static int lists(struct reader *r,struct control *p)
{
 struct v4l2_ctrl_h264_slice_params *s=&p->slice;
 unsigned defaults[2][MAX_ACTIVE_REFS],n=0;
 unsigned count[2]={s->num_ref_idx_l0_active_minus1+1,s->num_ref_idx_l1_active_minus1+1};
 for(unsigned i=0;i<MAX_DPB_REFS;i++)if(p->decode.dpb[i].flags&V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) {
  if(n==MAX_ACTIVE_REFS)return -EINVAL;
  defaults[0][n]=defaults[1][n]=i;n++;
 }
 for(unsigned list=0;list<(s->slice_type==1?2:1);list++)
  for(unsigned i=1;i<n;i++)for(unsigned j=i;j;j--) {
   unsigned a=defaults[list][j-1],b=defaults[list][j];
   if(before(&p->decode.dpb[a],&p->decode.dpb[b],s->slice_type==1?list+1:0,p->decode.top_field_order_cnt))break;
   defaults[list][j-1]=b;defaults[list][j]=a;
  }
 if(s->slice_type==1 && n>1 && !memcmp(defaults[0],defaults[1],n*sizeof(unsigned)))swap(defaults[1][0],defaults[1][1]);
 p->rpl_start_bit=r->pos;
 for(unsigned list=0;list<(s->slice_type==1?2:1);list++) {
  unsigned order[64],length=n,at=0;
  memcpy(order,defaults[list],n*sizeof(unsigned));
  u32 max_num=1U<<(p->sps.log2_max_frame_num_minus4+4+p->field_picture);
  u32 predicted=p->decode.frame_num*(p->field_picture?2:1)+p->field_picture;
  if(get(r,1))for(unsigned operation=0;!r->error;) {
   operation=ue(r);
   if(operation==3)break;
   if(operation>3 || at>=32)return -EINVAL;
   u32 argument=ue(r),identity=argument;
   if(operation!=2) {
    if(argument>=max_num)return -EINVAL;
    predicted=(predicted+(operation?argument+1:max_num-argument-1))%max_num;identity=predicted;
   }
   unsigned selected=MAX_DPB_REFS;
   for(unsigned i=0;i<MAX_DPB_REFS;i++) {
    struct v4l2_h264_dpb_entry *e=&p->decode.dpb[i];
    if(!(e->flags&V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))continue;
    bool lt=e->flags&V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM;
    if((operation==2)==lt && (lt?e->pic_num:(u32)(((s32)e->pic_num+(s32)max_num)%(s32)max_num))==identity) {
     if(selected!=MAX_DPB_REFS)return -EINVAL;
     selected=i;
    }
   }
   if(selected==MAX_DPB_REFS || at>length || length>=ARRAY_SIZE(order))return -EINVAL;
   memmove(order+at+1,order+at,(length-at)*sizeof(unsigned));order[at++]=selected;length++;
   for(unsigned i=at;i<length;)if(order[i]==selected){memmove(order+i,order+i+1,(--length-i)*sizeof(unsigned));}else i++;
  }
  if(count[list]>length)return -EINVAL;
  for(unsigned i=0;i<count[list];i++) {
   struct v4l2_h264_reference *ref=list?&s->ref_pic_list1[i]:&s->ref_pic_list0[i];
   ref->index=order[i];ref->fields=p->decode.dpb[order[i]].fields;
  }
 }
 p->rpl_end_bit=r->pos;
 return r->error;
}
int mt6797_h264_parse(struct control *p,const u8 *nal,unsigned bytes)
{
 struct reader r={.data=nal,.bytes=bytes};
 struct v4l2_ctrl_h264_slice_params *s=&p->slice;
 struct mt6797_h264_decode_params *d=&p->decode;
 struct v4l2_ctrl_h264_sps *sps=&p->sps;
 struct v4l2_ctrl_h264_pps *pps=&p->pps;
 struct mt6797_h264_header_words hw;
 if(bytes<4 || mt6797_h264_header_words(sps,pps,&hw))return -EINVAL;
 memset(s,0,sizeof(*s));memset(&p->weights,0,sizeof(p->weights));
 u32 header=get(&r,8),kind=header&31;
 if(header&0x80 || (kind!=1 && kind!=5))return -EINVAL;
 same(&r,(header>>5)&3,d->nal_ref_idc);
 same(&r,kind==5,!!(d->flags&V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC));
 s->first_mb_in_slice=ue(&r);u32 type=ue(&r);
 if(type>9 || type%5>2)return -EINVAL;
 s->slice_type=type%5;
 same(&r,ue(&r),pps->pic_parameter_set_id);
 same(&r,get(&r,sps->log2_max_frame_num_minus4+4),d->frame_num);
 p->field_picture=!(sps->flags&V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)?get(&r,1):0;
 p->bottom_field=p->field_picture?get(&r,1):0;
 same(&r,p->field_picture,!!(d->flags&V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC));
 same(&r,p->bottom_field,!!(d->flags&V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD));
 if(kind==5)same(&r,ue(&r),d->idr_pic_id);
 unsigned start=r.pos;
 if(sps->pic_order_cnt_type==0) {
  same(&r,get(&r,sps->log2_max_pic_order_cnt_lsb_minus4+4),d->pic_order_cnt_lsb);
  if((pps->flags&V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) && !p->field_picture)same(&r,se(&r),d->delta_pic_order_cnt_bottom);
 } else if(sps->pic_order_cnt_type==1 && !(sps->flags&V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO)) {
  same(&r,se(&r),d->delta_pic_order_cnt0);
  if((pps->flags&V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) && !p->field_picture)same(&r,se(&r),d->delta_pic_order_cnt1);
 }
 d->pic_order_cnt_bit_size=r.pos-start;
 if(pps->flags&V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT)s->redundant_pic_cnt=ue(&r);
 if(s->slice_type==1)s->flags=get(&r,1)?V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED:0;
 s->num_ref_idx_l0_active_minus1=pps->num_ref_idx_l0_default_active_minus1;
 s->num_ref_idx_l1_active_minus1=s->slice_type==1?pps->num_ref_idx_l1_default_active_minus1:0;
 if(s->slice_type!=2) {
  if(get(&r,1)) {
   u32 c=ue(&r);if(c>31)return -EINVAL;s->num_ref_idx_l0_active_minus1=c;
   if(s->slice_type==1){c=ue(&r);if(c>31)return -EINVAL;s->num_ref_idx_l1_active_minus1=c;}
  }
  int ret=lists(&r,p);if(ret)return ret;
 }
 if(V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(pps,s)) {
  u32 luma=ue(&r),chroma=ue(&r);if(luma>7 || chroma>7)return -EINVAL;
  p->weights.luma_log2_weight_denom=luma;p->weights.chroma_log2_weight_denom=chroma;
  for(unsigned list=0;list<(s->slice_type==1?2:1);list++) {
   unsigned count=(list?s->num_ref_idx_l1_active_minus1:s->num_ref_idx_l0_active_minus1)+1;
   struct v4l2_h264_weight_factors *w=&p->weights.weight_factors[list];
   for(unsigned i=0;i<count;i++) {
    w->luma_weight[i]=1<<luma;
    if(get(&r,1)) {s32 a=se(&r),b=se(&r);if(a< -128 || a>127 || b< -128 || b>127)return -EINVAL;w->luma_weight[i]=a;w->luma_offset[i]=b;}
    w->chroma_weight[i][0]=w->chroma_weight[i][1]=1<<chroma;
    if(get(&r,1))for(unsigned j=0;j<2;j++){s32 a=se(&r),b=se(&r);if(a< -128 || a>127 || b< -128 || b>127)return -EINVAL;w->chroma_weight[i][j]=a;w->chroma_offset[i][j]=b;}
   }
  }
 }
 start=r.pos;
 if(d->nal_ref_idc) {
  if(kind==5){get(&r,1);get(&r,1);}
  else if(get(&r,1))for(unsigned i=0;;i++) {
   u32 operation=ue(&r);if(!operation || r.error)break;
   if(i>=64 || operation>6)return -EINVAL;
   if(operation==1 || operation==3)ue(&r);
   if(operation==2 || operation==3 || operation==4 || operation==6)ue(&r);
   if(operation==5)p->memory_reset=1;
  }
 }
 d->dec_ref_pic_marking_bit_size=r.pos-start;
 if(s->slice_type!=2 && (pps->flags&V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE))s->cabac_init_idc=ue(&r);
 s32 qp=se(&r);if(qp< -51 || qp>51)return -EINVAL;s->slice_qp_delta=qp;
 if(pps->flags&V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) {
  u32 mode=ue(&r);if(mode>2)return -EINVAL;s->disable_deblocking_filter_idc=mode;
  if(mode!=1){s32 a=se(&r),b=se(&r);if(a< -6 || a>6 || b< -6 || b>6)return -EINVAL;s->slice_alpha_c0_offset_div2=a;s->slice_beta_offset_div2=b;}
 }
 s->header_bit_size=r.pos;
 if(r.error || mt6797_h264_cabac_alignment(pps,s,nal,bytes))return -EINVAL;
 if(V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(pps,s))return mt6797_h264_weight_range(sps,pps,s,d,&p->weights,nal,bytes,&p->weight_range);
 return 0;
}
