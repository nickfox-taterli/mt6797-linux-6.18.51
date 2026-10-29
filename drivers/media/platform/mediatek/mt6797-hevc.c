// SPDX-License-Identifier: GPL-2.0-only
/* MT6797 HEVC register algorithm from the qualified open sequence decoder.
 * Standard controls are translated separately; only this driver owns DMA/MMIO. */
#include "mt6797-hevc.h"
#define VLD(o) (0x1000+(o))
#define TOP(o) (0x1800+(o))
#define MC(o) (0x2000+(o))
#define HVMV(o) (0x4000+(o))
#define PP(o) (0x5000+(o))
#define HVLD(o) (0x8000+(o))
static void wr(struct vdec_ctx *ctx,unsigned off,u32 v){writel(v,ctx->dev->regs+off);}
static u32 rd(struct vdec_ctx *ctx,unsigned off){return readl(ctx->dev->regs+off);}
static int poll_word(struct vdec_ctx *ctx,unsigned off,u32 mask,u32 target){u32 v;return readl_poll_timeout(ctx->dev->regs+off,v,(v&mask)==target,1,100000);}
static unsigned ceil_log2_word(unsigned v){return v>1?fls(v-1):0;}
static void advance_bits(struct vdec_ctx *ctx,unsigned bits){while(bits){unsigned n=min(bits,32U);(void)rd(ctx,HVLD(n*4));bits-=n;}}
static int check_window(struct vdec_ctx *ctx,const u8 *nal,size_t length,unsigned position)
{
 u32 expected=0,mask=0;unsigned bit=0,count=0,zeros=0;
 for(size_t i=0;i<length && count<32;i++){
  unsigned value=nal[i];if(zeros>=2 && value==3){zeros=0;continue;}zeros=value?0:zeros+1;
  for(unsigned j=0;j<8 && count<32;j++,bit++)if(bit>=position){expected|=((value>>(7-j))&1U)<<(31-count);mask|=1U<<(31-count++);}
 }
 return !count || (rd(ctx,HVLD(0))&mask)!=expected?-EINVAL:0;
}
void mt6797_hevc_forget(struct vdec_ctx *ctx,unsigned index)
{
 struct mt6797_hevc_state *s=ctx->hevc;
 if(!s || index>=MAX_SLOTS)return;
 struct buffer *buffers[2]={&s->picture[index],&s->motion[index]};
 for(unsigned i=0;i<2;i++){struct buffer *b=buffers[i];if(b->cpu)dma_free_coherent(ctx->dev->dev,b->size,b->cpu,b->dma);memset(b,0,sizeof(*b));}
 s->timestamps[index]=0;
}
static int backing(struct vdec_ctx *ctx,struct buffer *b,unsigned used)
{
 if(b->cpu && b->used!=used){dma_free_coherent(ctx->dev->dev,b->size,b->cpu,b->dma);memset(b,0,sizeof(*b));}
 if(!b->cpu)return vdec_buffer(ctx,b,used+GUARD_SIZE);
 return 0;
}
int mt6797_hevc_run(struct vdec_ctx *ctx,struct vb2_v4l2_buffer *dst)
{
 struct mt6797_hevc_state *s=ctx->hevc;
 struct mt6797_hevc_control *p=&s->control;
 unsigned output_slot=dst->vb2_buf.index,luma=p->aligned_width*p->aligned_height,pixels=luma*3/2;
 if(output_slot>=MAX_SLOTS || !vb2_plane_vaddr(&dst->vb2_buf,0) || vb2_plane_size(&dst->vb2_buf,0)<pixels)return -EINVAL;
 unsigned mapping[2][MAX_REFS]={{0}},list_count[2]={p->ref_table_count,p->ref_table_count};
 bool is_p=p->slice_type!=2,has_inter=p->picture_has_inter;
 bool rpl_load=is_p && p->lists_modification_present_flag && p->ref_table_count>1;
 u64 protected=0;int low_delay=1,failed=0;unsigned attempt=0,retry_requested=0;
 u64 *timestamps=s->timestamps;
 if(p->is_idr || (p->nal_type>=16 && p->nal_type<=18))memset(timestamps,0,sizeof(s->timestamps));
 for(unsigned i=0;i<p->keep_ref_count;i++){
  unsigned slot;for(slot=0;slot<MAX_SLOTS;slot++)if(timestamps[slot]==p->keep_ref_ts[i])break;
  if(slot==MAX_SLOTS)return -ENOENT;
  protected|=BIT_ULL(slot);
 }
 if(protected&BIT_ULL(output_slot))return -EINVAL;
 for(unsigned list=0;list<(has_inter?2U:0U);list++)for(unsigned i=0;i<list_count[list];i++){
  u64 ts=list?p->l1_ref_ts[i]:p->l0_ref_ts[i];unsigned slot;
  for(slot=0;slot<MAX_SLOTS;slot++)if(timestamps[slot]==ts)break;
  s32 diff=list?p->l1_poc_diff[i]:p->l0_poc_diff[i];
  if(slot==MAX_SLOTS || (s64)p->top_field_order_cnt-s->pocs[slot]!=diff || s->picture[slot].used!=pixels)return -ENOENT;
  mapping[list][i]=slot;if(diff<0)low_delay=0;
 }
 unsigned mv_bytes=max(ALIGN(luma/4,4096),32768U);
 if(backing(ctx,&s->picture[output_slot],pixels) || backing(ctx,&s->motion[output_slot],mv_bytes))return -ENOMEM;
 memset(s->motion[output_slot].cpu,0,mv_bytes);
 struct buffer b[2]={ctx->buffers[0],s->picture[output_slot]};
 static const u8 eob[]={0,0,0,1,0x48,1};
 writel(1,ctx->dev->gcon);
		
		unsigned char *nalp = b[0].cpu+4;
		size_t nal_len = ctx->bytes;
		memmove(b[0].cpu+4,b[0].cpu,nal_len);memset(b[0].cpu,0,4);
		b[0].cpu[p->start_code_bits / 8 - 1] = 1;
		
		/* Temporal extraction/POC/DPB decisions belong to the software
		 * parser. Present accepted pictures to the temporal-id-zero
		 * start-code scanner; the original NAL and CABAC payload stay
		 * intact outside this staging copy. */
		b[0].cpu[p->start_code_bits / 8 + 1] = 1;
		size_t staged = p->start_code_bits / 8 + nal_len;
		memcpy(b[0].cpu + staged, eob, sizeof(eob));
		staged += sizeof(eob);
		dma_wmb();
		/* Bound the FIFO to this NAL and its EOS trailer. A large zero
		 * tail can finish the last CTB without delivering DONE16. */
		u64 stream_end = b[0].dma + staged + 256;
		u32 ctrl2 = is_p ? 0x1011 : 0x811;
		for (unsigned round = 0; round < 2 && !failed; ++round) {
			/* The per-picture tail already stops the bus, but the round-0
			 * fetch and the bounded retry path can leave the prefetcher
			 * running when this reset dance begins. Resetting GCON/VLD in
			 * that window makes the fetcher request IOVA 0 (larb1 port3
			 * fault, reproduced ~11%/case under sustained decode). Stop the
			 * bus and wait for the idle bits before every reset dance, the
			 * same primitive the kernel exit path uses. */
			wr(ctx,0x0100, rd(ctx,0x0100) | 1);
			if (poll_word(ctx,0x0104, 0x11, 0x11)) { failed = 1; break; }
			wr(ctx,VLD(0x108), 0x101);
			wr(ctx,0x00f4, 0);
			wr(ctx,VLD(0xf4), 0);
			wr(ctx,VLD(0x108), 0x101);
			wr(ctx,0x00c8, 0x0000feff);
			wr(ctx,0x00cc, 0x77e30182);
			wr(ctx,0x0178, 0xfffffa60);
			wr(ctx,0x0084, 2);
			wr(ctx,VLD(0x108), 0);
			writel((readl(ctx->dev->gcon+0x18)|1)&~BIT(4),ctx->dev->gcon+0x18); dma_wmb();
			wr(ctx,0x00ec, 1);
			wr(ctx,0x0004, 0x100);
			wr(ctx,HVLD(0x84), 1);
			wr(ctx,VLD(0xec), 0x10000000);
			wr(ctx,VLD(0xb4), b[0].dma >> 6);
			wr(ctx,VLD(0xb8), stream_end >> 6);
			wr(ctx,VLD(0xb0), (u32)b[0].dma);
			wr(ctx,VLD(0xb0), (u32)b[0].dma);
			wr(ctx,VLD(0x110), (u32)stream_end);
			wr(ctx,VLD(0x108), 0x100);
			wr(ctx,VLD(0x108), 0);
			wr(ctx,VLD(0x8c), BIT(20));
			if (poll_word(ctx,VLD(0xe8), 1, 1)) { failed = 1; break; }
			wr(ctx,VLD(0x8c), BIT(23));
			if (round == 0) {
				wr(ctx,HVLD(0x84), ctrl2);
				wr(ctx,HVLD(0x94), 0x100);
				if (poll_word(ctx,HVLD(0x94), BIT(8), 0)) { failed = 1; break; }
			}
		}
		if (!failed) {
			wr(ctx,HVLD(0x88), 0x100);
			wr(ctx,HVLD(0x88), 0);
			{
				u32 sc_window = (u32)b[0].cpu[0] << 24 | (u32)b[0].cpu[1] << 16 |
					(u32)b[0].cpu[2] << 8 | b[0].cpu[3];
				/* A zero-bit peek does not consume input. Wait for the
				 * asynchronous fetch rather than treating a stale first
				 * window as a different stream position. */
				if (poll_word(ctx,HVLD(0), ~0U, sc_window)) {
					failed = 1;
					retry_requested = !attempt;
				} else {
					unsigned bits = (p->start_code_bits + p->header_bit_size + 7) & ~7U;
					if (rpl_load) {
						advance_bits(ctx,p->start_code_bits + p->rpl_start_bit);
					} else if (is_p && p->weights_end_bit) {
						advance_bits(ctx,p->start_code_bits + p->weights_start_bit);
					} else {
						advance_bits(ctx,bits);
					}
				}
			}
		}
		if (!failed) {
			wr(ctx,HVLD(0xbc), (p->is_idr ? BIT(31) : 0) | ((p->slice_type & 3) << 28) |
				(ceil_log2_word(p->lcu_w * p->lcu_h) << 20));
			wr(ctx,MC(0xa60), 0);
			for (unsigned list = 0; list < 2; ++list)
			for (unsigned i = 0; i < 16; ++i) {
				unsigned selected = list && (p->slice_type == 0 || p->control_version == 3);
				unsigned count = list_count[selected];
				unsigned slot = has_inter ? mapping[selected][i % count] : output_slot;
				u32 address = has_inter ? s->picture[slot].dma : b[1].dma;
				wr(ctx,MC(list ? 0x4dc : 0x45c) + i * 4, address);
				wr(ctx,MC(list ? 0xc60 : 0xbe0) + i * 4, address + luma);
				wr(ctx,HVMV(list ? 0x40 : 0x00) + i * 4, (s->motion[slot].dma) >> 4);
			}
			wr(ctx,HVMV(0x20c), (s->motion[output_slot].dma) >> 4);
			wr(ctx,HVLD(0xe0), 0xfffefffb);
			wr(ctx,HVLD(0xd0), staged);
			wr(ctx,HVLD(0xd4), 0x04011d01);
			wr(ctx,PP(0x40), p->pps_tile & 1 ? 0x4011 : 0x7011);
			wr(ctx,HVMV(0x210), 0);
			wr(ctx,HVMV(0x320), has_inter ? low_delay : 0);
			u32 long_mask = 0, long_dpb[4] = {0};
			if (has_inter) for (unsigned i = 0; i < 16; i++) {
				long_mask |= p->l0_long_term[i%list_count[0]] << i;
				long_mask |= (p->slice_type == 0 || p->control_version == 3 ? p->l1_long_term[i%list_count[1]] :
					p->l0_long_term[i%list_count[0]]) << (i+16);
			}
			unsigned nlong = 0;
			for (unsigned i = 0; i < p->keep_ref_count; i++) if (p->keep_ref_long_term[i]) {
				for (unsigned slot = 0; slot < MAX_SLOTS; slot++) if (timestamps[slot] == p->keep_ref_ts[i]) {
					long_dpb[nlong/4] |= (slot+1) << ((nlong%4)*8); nlong++; break;
				}
			}
			wr(ctx,HVMV(0x324), long_mask);
			for (unsigned i = 0; i < 4; i++) wr(ctx,HVMV(0x328)+i*4,long_dpb[i]);
			if (has_inter)
			for (unsigned list = 0; list < 2; ++list)
			for (unsigned i = 0; i < 16; ++i) {
				unsigned selected = list && (p->slice_type == 0 || p->control_version == 3);
				unsigned count = list_count[selected];
				unsigned index = i % count;
				s32 diff = selected ? p->l1_poc_diff[index] : p->l0_poc_diff[index];
				u32 info = ((u32)diff & 255) | ((mapping[selected][index] + 1) << 8);
				wr(ctx,HVMV(list ? 0x378 : 0x338) + i * 4, info);
			}
			wr(ctx,MC(0xb5c), 1);
			wr(ctx,MC(0xaf8), 0x00041103);
			wr(ctx,MC(0x208), p->width);
			wr(ctx,MC(0x20c), p->height);
			wr(ctx,MC(0x980), p->aligned_width / 16);
			wr(ctx,MC(0x934), luma);
			wr(ctx,MC(0x220), 1);
			wr(ctx,MC(0x224), b[1].dma >> 9);
			wr(ctx,MC(0x228), (b[1].dma + luma) >> 8);
			wr(ctx,MC(0x22c), (p->lcu_w << p->log2_ctb) / 16);
			wr(ctx,MC(0x238), 3);
			wr(ctx,MC(0x250), 1);
			wr(ctx,MC(0x260), (p->lcu_w << p->log2_ctb) / 16 - 1);
			wr(ctx,MC(0x264), (p->lcu_h << p->log2_ctb) / 16 - 1);
			wr(ctx,PP(0x630), p->scaling_enabled);
			if (p->scaling_enabled) {
				wr(ctx,PP(0x504),1);
				u32 dc0=0, dc1=0;
				for (unsigned i=0;i<4;i++) dc0 |= (u32)p->scaling_dc[2][i] << (i*8);
				dc1 = p->scaling_dc[2][4] | (u32)p->scaling_dc[2][5]<<8 |
					(u32)p->scaling_dc[3][0]<<16 | (u32)p->scaling_dc[3][3]<<24;
				wr(ctx,PP(0x634),dc0);wr(ctx,PP(0x638),dc1);
				unsigned cursor=0;
				for (unsigned size=0;size<4;size++) for(unsigned matrix=0;matrix<6;matrix+=size==3?3:1) {
					unsigned n=size?8:4;
					for(unsigned column=0;column<n;column++) for(unsigned row=0;row<n;row+=4) {
						u32 word=0;
						for(unsigned j=0;j<4;j++) word|=(u32)p->scaling[size][matrix][(row+j)*n+column]<<(j*8);
						wr(ctx,VLD(0x260),cursor++);wr(ctx,VLD(0x264),word);
					}
				}
				wr(ctx,PP(0x504),0);
			}
			if (rpl_load) {
				/* RPL parsing uses the SPS list flag and NumPocTotalCurr,
				 * not just the active minus-one fields in SLICE_2. */
				wr(ctx,HVLD(0xa0), p->sps0);
				wr(ctx,HVLD(0xac), p->pps_word);
				wr(ctx,HVLD(0xcc), p->ref_table_count | (ceil_log2_word(p->ref_table_count) << 4));
				wr(ctx,HVLD(0xc4), (p->slice_type == 0 ? p->num_ref_idx_l1_active_minus1 << 4 : 0xf0) | p->num_ref_idx_l0_active_minus1);
				if (check_window(ctx,nalp, nal_len, p->rpl_start_bit)) { failed = 1; return -EIO; }
				wr(ctx,HVLD(0x94), BIT(4));
				if (poll_word(ctx,HVLD(0x94), BIT(4), 0) || check_window(ctx,nalp, nal_len, p->rpl_end_bit)) {
					failed = 1; return -EIO;
				}
				unsigned next = p->weights_end_bit ? p->weights_start_bit : ((p->header_bit_size + 7) & ~7U);
				advance_bits(ctx,next - p->rpl_end_bit);
			}
			if (is_p && p->weights_end_bit) {
				wr(ctx,HVLD(0xc4), (p->slice_type == 0 ? p->num_ref_idx_l1_active_minus1 << 4 : 0xf0) | p->num_ref_idx_l0_active_minus1);
				(void)rd(ctx,HVLD(0));
				if (check_window(ctx,nalp, nal_len, p->weights_start_bit)) failed = 1;
				if (failed) return -EIO;
				wr(ctx,HVLD(0x94), 1);
				if (poll_word(ctx,HVLD(0x94), 1, 0)) failed = 1;
				if (check_window(ctx,nalp, nal_len, p->weights_end_bit)) failed = 1;
				if (failed) return -EIO;
				advance_bits(ctx,((p->header_bit_size + 7) & ~7U) - p->weights_end_bit);
			}
			{
				u32 qp = (u32)(p->slice_qp & 0x3f);
				wr(ctx,HVLD(0xc0), (p->sao_luma << 5) | (p->sao_chroma << 6) | (p->cabac_init_flag << 7) | (qp << 8) |
					(((u32)(p->pps_cb_qp_offset+p->slice_cb_qp_offset) & 0x1f) << 16) |
					(((u32)(p->pps_cr_qp_offset+p->slice_cr_qp_offset) & 0x1f) << 24));
				u32 merge_field = (5 - p->five_minus_max_num_merge_cand) & 7;
				wr(ctx,HVLD(0xc4), (is_p ? (p->num_ref_idx_l0_active_minus1 & 0xf) : 0xf) |
					(p->slice_type == 0 ? p->num_ref_idx_l1_active_minus1 << 4 : 0xf0) | (p->collocated_ref_idx << 8) | merge_field << 12 |
					(((u32)p->beta_offset_div2 & 0x1f) << 16) |
					(((u32)p->tc_offset_div2 & 0x1f) << 21) |
					(p->collocated_from_l0_flag << 27) | (p->mvd_l1_zero_flag << 28) | (p->slice_temporal_mvp_enabled_flag ? BIT(26) : 0) |
					((p->loop_filter_across_slices_enabled_flag & 1) << 29) |
					((p->deblocking_filter_disabled_flag & 1) << 30));
				if (has_inter) {
					u32 curr = 0;
					for (unsigned i = 0; i < MAX_REFS && i < p->rps_neg; ++i) curr += p->rps_neg_used[i];
					for (unsigned i = 0; i < MAX_REFS && i < p->rps_pos; ++i) curr += p->rps_pos_used[i];
					curr += p->num_long_term_used;
					u32 length = 0;
					if (curr > 1) {
						length = 1;
						u32 rest = curr - 1;
						while (rest >>= 1) ++length;
					}
					wr(ctx,HVLD(0xc8), p->rps_prediction_ref_count | ((p->rps_neg + p->rps_pos) << 8) |
						(p->num_long_term << 16) | (p->num_long_term_sps << 24));
					wr(ctx,HVLD(0xcc), (curr & 0xf) | ((length & 0x7) << 4));
				} else {
					wr(ctx,HVLD(0xc8), 0);
					wr(ctx,HVLD(0xcc), 0);
				}
			}
			wr(ctx,HVLD(0xf4), (0xffff << 16) | ((p->nal_type & 0x3f) << 8) | 21);
			wr(ctx,TOP(0x68), (p->lcu_w - 1) | ((p->lcu_h - 1) << 16));
			wr(ctx,TOP(0x70), p->width | (p->height << 16));
			wr(ctx,TOP(0x94), p->lcu_w | (p->lcu_h << 16));
			wr(ctx,TOP(0x100), 0);
			wr(ctx,HVLD(0xa0), p->sps0);
			wr(ctx,HVLD(0xa4), p->sps1);
			wr(ctx,HVLD(0xa8), p->sps_size);
			wr(ctx,HVLD(0xac), p->pps_word);
			wr(ctx,HVLD(0xb0), p->pps_qp);
			wr(ctx,HVLD(0xb4), p->pps_tile);
			wr(ctx,HVLD(0xb8), p->pps_dbk);
			for (unsigned off = 0x140; off <= 0x17c; off += 4) wr(ctx,HVLD(off), 0);
			for (unsigned off = 0x180; off <= 0x1bc; off += 4) wr(ctx,HVLD(off), 0);
			wr(ctx,HVLD(0x140) + ((p->lcu_w - 1) / 32) * 4, 1U << ((p->lcu_w - 1) % 32));
			wr(ctx,HVLD(0x180) + ((p->lcu_h - 1) / 32) * 4, 1U << ((p->lcu_h - 1) % 32));
			if (p->pps_tile & 1)
				for (unsigned i = 0; i < 8; i++) {
					wr(ctx,HVLD(0x140) + i*4, p->tile_x[i]);
					wr(ctx,HVLD(0x180) + i*4, p->tile_y[i]);
				}
			wr(ctx,HVLD(0x1c0), 0);
			wr(ctx,HVLD(0x1d0), 0);
			dma_wmb();
			reinit_completion(&ctx->dev->done); WRITE_ONCE(ctx->dev->status,0);
            wr(ctx,HVLD(0x90), 1);
			int waited=wait_for_completion_timeout(&ctx->dev->done,msecs_to_jiffies(500))?0:-ETIMEDOUT;
            struct {u32 status;} wait={.status=READ_ONCE(ctx->dev->status)};
            u32 misc = rd(ctx,0x00a4);
			u32 done = (wait.status & BIT(16)) | (misc & BIT(16));
			u32 errors = rd(ctx,HVLD(0xe4)), accumulated = rd(ctx,HVLD(0xe8));
			u32 last = rd(ctx,HVLD(0x12c));
			if (waited || !done || errors || accumulated ||
			    last != ((p->lcu_w - 1) | ((p->lcu_h - 1) << 16))) {
				failed = 1;
				/* A clean final CTB without completion may be reset and
				 * decoded once more. Success still requires a real IRQ;
				 * corruption/error reports never take this retry path. */
				if (!errors && !accumulated && (waited || !done) &&
				    last == ((p->lcu_w - 1) | ((p->lcu_h - 1) << 16)))
					retry_requested = !attempt;
			}
		}
		if (!failed) {
			wr(ctx,0x0100, rd(ctx,0x0100) | 1);
			if (poll_word(ctx,0x0104, 0x11, 0x11)) failed = 1;
		}

 if(failed)return retry_requested?-EAGAIN:-EIO;
 for(unsigned i=0;i<MAX_SLOTS;i++)if(s->picture[i].cpu){
  struct buffer *a=&s->picture[i],*m=&s->motion[i];
  if(memchr_inv(a->cpu+a->used,0xd3,a->size-a->used) || memchr_inv(m->cpu+m->used,0xd3,m->size-m->used))return -EIO;
 }
 if(memchr_inv(b[0].cpu+INPUT_SIZE,0xd3,GUARD_SIZE))return -EIO;
 dma_rmb();memcpy(vb2_plane_vaddr(&dst->vb2_buf,0),b[1].cpu,pixels);vb2_set_plane_payload(&dst->vb2_buf,0,pixels);
 for(unsigned i=0;i<MAX_SLOTS;i++)if(!(protected&BIT_ULL(i)))timestamps[i]=0;
 timestamps[output_slot]=p->is_reference?ctx->timestamp:0;s->pocs[output_slot]=p->top_field_order_cnt;
 return 0;
}
