// SPDX-License-Identifier: GPL-2.0-only
/* MT6797 H.264 algorithm moved from the qualified open userspace path.
 * Register protocol adapted from georgewhewell/nixos-gemini engine.c,
 * GPL-2.0-only, commit 0c005c3b88d11e370797dd9d53842cb2be2c5b6e.
 */
#include <linux/sizes.h>
#include <linux/slab.h>
#include "mt6797-vdec.h"
static void wr(struct vdec_ctx *ctx,unsigned off,u32 value) { writel(value,ctx->dev->regs+off); }
static u32 rd(struct vdec_ctx *ctx,unsigned off) { return readl(ctx->dev->regs+off); }
static int poll_word(struct vdec_ctx *ctx,unsigned off,u32 mask,u32 expected)
{ u32 value; return readl_poll_timeout(ctx->dev->regs+off,value,(value&mask)==expected,1,100000); }
static int poll_nonzero(struct vdec_ctx *ctx,unsigned off)
{ u32 value; return readl_poll_timeout(ctx->dev->regs+off,value,value,1,100000); }
static int check_pattern(const u8 *p,size_t n,u8 value) { return memchr_inv(p,value,n)?-EIO:0; }
int mt6797_h264_run(struct vdec_ctx *ctx,struct vb2_v4l2_buffer *dst)
{
 struct buffer *b=ctx->buffers;
 struct mt6797_h264_cached_picture *cache=ctx->cache;
 const u8 *nal[1]={b[0].cpu};
 size_t lengths[1]={ctx->bytes};
 unsigned picture=0,max_luma=ctx->width*ALIGN(ctx->height,32);
 unsigned slot_used[MAX_SLOTS];
 static const u8 ending[]={0,0,0,1,0x0a};
 for (unsigned i=0;i<MAX_SLOTS;i++) slot_used[i]=max_luma*7/4;
 if (!vb2_plane_vaddr(&dst->vb2_buf,0)) return -EFAULT;
		struct mt6797_h264_header_words header;
		struct mt6797_h264_slice_words slice;
		struct mt6797_h264_reference_plan plan;
		struct control *p = &ctx->picture;
		unsigned luma = p->width * ((p->height + 31) & ~31U), pixels = luma * 3 / 2, motion = luma / 4;
		unsigned type = nal[picture][0] & 31;
		unsigned is_idr = type == 5;
		unsigned is_b = p->slice.slice_type == V4L2_H264_SLICE_TYPE_B;
		if (is_idr && !p->complementary_idr) memset(cache, 0, sizeof(ctx->cache));

		if (mt6797_h264_header_words(&p->sps, &p->pps, &header) ||
		    mt6797_h264_slice_words(&p->sps, &p->pps, &p->slice, &p->decode, lengths[picture], &slice) ||
		    mt6797_h264_picture_poc(&p->sps, &p->pps, &p->decode) ||
		    mt6797_h264_reference_plan(&p->sps, &p->slice, &p->decode, cache,
					       p->reference_ts, &plan) ||
		    plan.output >= MAX_SLOTS || plan.count > MAX_ACTIVE_REFS || plan.count1 > MAX_ACTIVE_REFS ||
		    p->width != header.width_mbs * 16 ||
		    (p->height + 15) / 16 != header.height_mbs || p->height % 2) {

			return -EIO;
		}
		if (p->field_pair_ts) {
			unsigned slot;
			for (slot=0;slot<MAX_SLOTS;slot++)
				if (cache[slot].valid && cache[slot].field_timestamp[p->bottom_field^1]==p->field_pair_ts &&
				    !(cache[slot].field_mask & (1U<<p->bottom_field))) break;
			if (slot==MAX_SLOTS || !p->field_picture) {
				return -EIO;
			}
			plan.output=slot;
		}
        unsigned tables[3][MAX_ACTIVE_REFS],table_counts[3]={0};
        for (unsigned mode=0;mode<3;mode++) {
            unsigned defaults[MAX_DPB_REFS],n=0;
            for (unsigned i=0;i<MAX_DPB_REFS;i++)
                if (p->decode.dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) defaults[n++]=i;
            for (unsigned i=1;i<n;i++) for (unsigned j=i;j;j--) {
                unsigned x=defaults[j-1],y=defaults[j];
                const struct v4l2_h264_dpb_entry *a=&p->decode.dpb[x],*b=&p->decode.dpb[y];
                unsigned al=!!(a->flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM),bl=!!(b->flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM);
                int before;
                if (al!=bl) before=al<bl;
                else if (al) before=a->pic_num<=b->pic_num;
                else if (!mode) before=(s32)a->pic_num>=(s32)b->pic_num;
                else {
                    s32 ap=a->fields==V4L2_H264_BOTTOM_FIELD_REF?a->bottom_field_order_cnt:a->top_field_order_cnt;
                    s32 bp=b->fields==V4L2_H264_BOTTOM_FIELD_REF?b->bottom_field_order_cnt:b->top_field_order_cnt;
                    s32 current_poc=p->decode.top_field_order_cnt;
                    unsigned aside=mode==1?ap>=current_poc:ap<current_poc,bside=mode==1?bp>=current_poc:bp<current_poc;
                    before=aside!=bside?aside<bside:(ap>=current_poc?ap<=bp:ap>=bp);
                }
                if (before) break;
                defaults[j-1]=y;defaults[j]=x;
            }
            unsigned selected=mode==0?(p->slice.slice_type==V4L2_H264_SLICE_TYPE_P?plan.count:0):
                (is_b?(mode==1?plan.count:plan.count1):0);
            for (unsigned i=0;i<selected;i++) {
                unsigned index=(mode==2?p->slice.ref_pic_list1[i]:p->slice.ref_pic_list0[i]).index,present=0;
                if (p->rpl_end_bit) for(unsigned j=0;j<table_counts[mode];j++) present|=tables[mode][j]==index;
                if (!present) tables[mode][table_counts[mode]++]=index;
            }
            for (unsigned i=0;i<n && table_counts[mode]<MAX_ACTIVE_REFS;i++) {
                unsigned present=0;
                for (unsigned j=0;j<table_counts[mode];j++) present|=tables[mode][j]==defaults[i];
                if (!present) tables[mode][table_counts[mode]++]=defaults[i];
            }
            if (mode==2 && !is_b && table_counts[1]==table_counts[2] && table_counts[2]>1 &&
                !memcmp(tables[1],tables[2],table_counts[2]*sizeof(unsigned))) {
                unsigned first=tables[2][0];tables[2][0]=tables[2][1];tables[2][1]=first;
            }
        }

		struct buffer *out = &b[4 + plan.output];
		/* Map a DPB slot only when the reference planner first selects it.
		 * Preallocating all 33 possible field slots wastes DMA memory and
		 * turns guard validation into a scan of uncached, unused pictures. */
		if (!out->cpu) {
			if (vdec_buffer(ctx, out, max_luma * 7 / 4 + GUARD_SIZE)) {
				 return -EIO;
			}
			memset(out->cpu, 0xd3, out->size);
		}

		/* Reset only the target slot; live references keep their pixels. */
		if (!p->field_pair_ts) {
			memset(out->cpu, 0xd3, out->size);
			memset(out->cpu + pixels, 0, motion);
		}
		slot_used[plan.output] = pixels + motion;

		memcpy(b[0].cpu + lengths[picture], ending, sizeof(ending));
		for (unsigned int i = 1; i < 4; ++i) memset(b[i].cpu, 0, WORK_SIZE);
		dma_wmb();
		writel(1, ctx->dev->gcon);
		wr(ctx, 0x1108, 0x101); wr(ctx, 0x00f4, rd(ctx, 0x00f4) & ~1U);
		wr(ctx, 0x00a4, (rd(ctx, 0x00a4) | 1) & ~BIT(4));
		wr(ctx, 0x00c8, 0); wr(ctx, 0x00cc, 0); wr(ctx, 0x0178, 0);
		wr(ctx, 0, rd(ctx, 0) & 0xff400000); wr(ctx, 0x00ec, rd(ctx, 0x00ec) | 1);
		writel(readl(ctx->dev->gcon+0x18)|1,ctx->dev->gcon+0x18);
		wr(ctx, 0x0084, 2); wr(ctx, 0x1108, 0);
		wr(ctx, 0x12ec, rd(ctx, 0x12ec) | BIT(8)); wr(ctx, 0x12ec, rd(ctx, 0x12ec) & ~BIT(8));
		wr(ctx, 0x10c0, rd(ctx, 0x10c0) | BIT(18)); wr(ctx, 0x3084, 0xa8000007);
		wr(ctx, 0x2024, 6); wr(ctx, 0x1828, b[1].dma);
		wr(ctx, 0x10b4, b[0].dma >> 6); wr(ctx, 0x10b8, (b[0].dma + lengths[picture] + 256) >> 6);
		wr(ctx, 0x1110, rd(ctx, 0x1110) | 1); wr(ctx, 0x10b0, b[0].dma); wr(ctx, 0x10b0, b[0].dma);
		wr(ctx, 0x1110, b[0].dma + lengths[picture] + 256);
		wr(ctx, 0x328c, 1); wr(ctx, 0x1108, BIT(8)); wr(ctx, 0x1108, 0);
		wr(ctx, 0x108c, BIT(20));
		if (poll_word(ctx, 0x10e8, 1, 1)) return -EIO;
		wr(ctx, 0x108c, BIT(23)); wr(ctx, 0x328c, 0);
		u32 first = (u32)nal[picture][0] << 24 | (u32)nal[picture][1] << 16 |
			(u32)nal[picture][2] << 8 | nal[picture][3];
		if (rd(ctx, 0x3000) != first) {  return -EIO; }
		wr(ctx, 0x3228, 1);
		if (poll_word(ctx, 0x3228, 1, 0)) return -EIO;
		wr(ctx, 0x30a0, 1U << (p->sps.log2_max_frame_num_minus4 + 4 + p->field_picture));
		wr(ctx, 0x30a4, p->decode.frame_num*(p->field_picture?2:1)+p->field_picture);
		if (p->pps.flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT) {
			struct mt6797_h264_scaling_word scaling[MT6797_H264_SCALING_WORDS];
			if (mt6797_h264_scaling_words(&p->scaling, scaling)) return -EIO;
			for (unsigned i = 0; i < MT6797_H264_SCALING_WORDS; i++) {
				wr(ctx, 0x1260, 0x200 + scaling[i].location);
				wr(ctx, 0x1264, scaling[i].coefficients);
				wr(ctx, 0x1260, 0x100 + scaling[i].location);
				if (rd(ctx, 0x1264) != scaling[i].coefficients) {
					 return -EIO;
				}
			}
		}
		wr(ctx, 0x3088, header.sps); wr(ctx, 0x3090, header.pps); wr(ctx, 0x3094, header.reference_counts);
		wr(ctx, 0x308c, (rd(ctx, 0x308c) & ~0x1ffffU) | slice.first_mb);
		wr(ctx, 0x3098, (rd(ctx, 0x3098) & 0x1fff) | slice.header1); wr(ctx, 0x309c, slice.header2);
		/* BSP AVC INIT_CTX_SRAM initializes CABAC from PPS, slice QP
		 * and cabac_init_idc before starting picture reconstruction. */
		if (p->pps.flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
			wr(ctx, 0x3234, 1);
		wr(ctx, 0x3280, (u32)~(BIT(14) | BIT(21))); wr(ctx, 0x32c8, rd(ctx, 0x32c8) & ~BIT(5));
        /* Populate P/L0/L1 search banks from the complete retained DPB.
         * Subsequent slices parse their own type, counts, reordering and
         * weights, while every referenced PicNum has a live DMA binding. */
        for (unsigned mode=0;mode<3;mode++) {
            u32 field_mask=0;
            for (unsigned i=0;i<table_counts[mode];i++) {
                unsigned index=tables[mode][i],slot;
                const struct v4l2_h264_dpb_entry *entry=&p->decode.dpb[index];
                for (slot=0;slot<MAX_SLOTS;slot++) if (cache[slot].valid &&
                    (cache[slot].timestamp==entry->reference_ts || cache[slot].field_timestamp[0]==entry->reference_ts ||
                     cache[slot].field_timestamp[1]==entry->reference_ts)) break;
                struct mt6797_h264_p_reference_words word;
                if (slot==MAX_SLOTS || (mode?mt6797_h264_b_reference_words(entry,slot,&word):
                    mt6797_h264_p_reference_words(entry,slot,&word))) return -EIO;
                u32 address=b[4+slot].dma;
                unsigned mc=mode==0?0x23dc:mode==1?0x245c:0x24dc;
                unsigned vld=mode==0?0x30a8:mode==1?0x3128:0x31a8;
                unsigned mv=mode==0?0x4000:mode==1?0x4080:0x4100;
                unsigned bottom=entry->fields==V4L2_H264_BOTTOM_FIELD_REF;
                wr(ctx, mc+i*4,address);wr(ctx, vld+i*4,word.picture_number);
                wr(ctx, mv+i*(p->field_picture?4:8),word.top);
                if (!p->field_picture) wr(ctx, mv+i*8+4,word.bottom);
                if (p->field_picture && bottom) field_mask|=BIT(i);
                if (mode==2) {
                    u32 motion=(address+pixels)>>4;
                    wr(ctx, 0x4180+i*(p->field_picture?4:8),motion+(p->field_picture?bottom*2:0));
                    if (!p->field_picture) wr(ctx, 0x4184+i*8,motion+4);
                }
            }
            wr(ctx, mode==0?0x2564:mode==1?0x2568:0x256c,field_mask);
        }

		if (p->weight_range.end || p->rpl_end_bit) {
			wr(ctx, 0x3088, header.sps); wr(ctx, 0x3090, header.pps); wr(ctx, 0x3094, header.reference_counts);
			wr(ctx, 0x3098, (rd(ctx, 0x3098) & 0x1fff) | slice.header1);
		}
		for (unsigned int bits = p->rpl_end_bit ? p->rpl_start_bit : p->weight_range.end ? p->weight_range.start : p->slice.header_bit_size; bits;) {
			unsigned int n = bits > 32 ? 32 : bits;
			(void)rd(ctx, 0x3000 + n * 4); bits -= n;
		}
        if (p->rpl_end_bit) {
            wr(ctx, 0x322c, 1);
            if (poll_nonzero(ctx, 0x322c) || (((rd(ctx, 0x3084)>>16)&31) != (p->rpl_end_bit&31))) {
                return -EIO;
            }
            unsigned next=p->weight_range.end?p->weight_range.start:p->slice.header_bit_size;
            for (unsigned bits=next-p->rpl_end_bit;bits;) {
                unsigned n=bits>32?32:bits;(void)rd(ctx, 0x3000+n*4);bits-=n;
            }
        }

		if (p->weight_range.end) {
			struct mt6797_h264_weight_range *range = &p->weight_range;
			u32 before = (rd(ctx, 0x3084) >> 16) & 63, window = rd(ctx, 0x3000);
			if ((before & 31) != (range->start & 31) ||
			    (window & range->start_mask) != range->start_word) {
				 return -EIO;
			}
			u32 select = rd(ctx, 0x00f0);
			wr(ctx, 0x00f0, select | 1); wr(ctx, 0x3230, 1);
			int parsed = poll_nonzero(ctx, 0x3230);
			wr(ctx, 0x00f0, select);
			u32 after = (rd(ctx, 0x3084) >> 16) & 63;
			window = rd(ctx, 0x3000);

			if (parsed || (after & 31) != (range->end & 31) ||
			    (window & range->end_mask) != range->end_word) {
				 return -EIO;
			}
			for (unsigned bits = p->slice.header_bit_size - range->end; bits;) {
				unsigned n = bits > 32 ? 32 : bits;
				(void)rd(ctx, 0x3000 + n * 4); bits -= n;
			}
		}
		s32 min_poc = p->decode.top_field_order_cnt < p->decode.bottom_field_order_cnt ?
			p->decode.top_field_order_cnt : p->decode.bottom_field_order_cnt;
		wr(ctx, 0x4200, (u32)min_poc & 0x3ffff);
		wr(ctx, 0x4204, (u32)p->decode.top_field_order_cnt & 0x3ffff);
		wr(ctx, 0x4208, (u32)p->decode.bottom_field_order_cnt & 0x3ffff);
		wr(ctx, 0x1868, (header.width_mbs - 1) | (p->sps.pic_height_in_map_units_minus1 << 16));
		wr(ctx, 0x1870, header.width_mbs * 16 | (header.height_mbs * 16 << 16));
		wr(ctx, 0x1858, ~0U); wr(ctx, 0x186c, (header.width_mbs * header.height_mbs) >> p->field_picture);
		unsigned pred_words = header.width_mbs * ((p->pps.flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) ? 6 : 5);
        if (p->sps.flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD) pred_words *= 2;
        wr(ctx, 0x1894, b[1].dma + (pred_words-1)*16);
		wr(ctx, 0x1850, ~0U); wr(ctx, 0x1854, rd(ctx, 0x1854) | 1); wr(ctx, 0x00f0, rd(ctx, 0x00f0) | 1);
		wr(ctx, 0x2090, 4); wr(ctx, 0x2768, (rd(ctx, 0x2768) & ~7U) | 4);
		wr(ctx, 0x2208, p->width); wr(ctx, 0x220c, header.height_mbs * 16);
		wr(ctx, 0x2220, 1); wr(ctx, 0x2224, out->dma >> 9); wr(ctx, 0x2228, (out->dma + luma) >> 8);
		wr(ctx, 0x222c, header.width_mbs); wr(ctx, 0x2238, 3); wr(ctx, 0x2250, 1);
		unsigned picture_mbs_height=header.height_mbs >> p->field_picture;
		wr(ctx, 0x2260, header.width_mbs - 1); wr(ctx, 0x2264, picture_mbs_height - 1); wr(ctx, 0x2268, 1);
		wr(ctx, 0x2560, out->dma); wr(ctx, 0x255c, luma); wr(ctx, 0x2980, header.width_mbs);
		wr(ctx, 0x2a20, b[2].dma); wr(ctx, 0x2a24, b[3].dma);
		wr(ctx, 0x25e4, rd(ctx, 0x25e4) | BIT(12)); wr(ctx, 0x2920, rd(ctx, 0x2920) | BIT(24));
		wr(ctx, 0x420c, ((out->dma + pixels) >> 4) + (p->bottom_field?2:0));
		wr(ctx, 0x4220, rd(ctx, 0x4220) & ~BIT(5));
		wr(ctx, 0x5804, rd(ctx, 0x5804) & ~BIT(4));
		if (poll_word(ctx, 0x325c, BIT(25), 0)) return -EIO;
		if (rd(ctx, 0x4200) != ((u32)min_poc & 0x3ffff)) {

			return -EIO;
		}
		dma_wmb();
		reinit_completion(&ctx->dev->done); WRITE_ONCE(ctx->dev->status,0);
		wr(ctx, 0x3238, 1);
		int waited = wait_for_completion_timeout(&ctx->dev->done, msecs_to_jiffies(500)) ? 0 : -ETIMEDOUT;
 struct { u32 status; } wait = {.status=READ_ONCE(ctx->dev->status)};
		u32 complete = rd(ctx, 0x3274), state = rd(ctx, 0x3278), error = rd(ctx, 0x3284), message = rd(ctx, 0x3270);
		u32 mbx = rd(ctx, 0x2028), mby = rd(ctx, 0x202c);
		int tail_valid = 0;
		/* The BSP identifies bit14 as CABAC_ZERO_WORD_ERR. Admit this
		 * trailer-only report only after a complete picture, its final MB,
		 * and a byte-aligned zero/start-code window. Preserve raw errors.
		 * Truncated CABAC pictures and every other error remain fatal. */
		if (!waited && complete && (state & BIT(31)) &&
		    (p->pps.flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) &&
		    error == BIT(14) && (!message || message == BIT(14)) &&
		    mbx == header.width_mbs - 1 && mby == picture_mbs_height - 1) {
			unsigned offset = (rd(ctx, 0x3084) >> 16) & 63, advance = (-offset) & 7;
			if (advance) (void)rd(ctx, 0x3000 + advance * 4);
			u32 tail = rd(ctx, 0x3000);
			tail_valid = !tail || tail == 1 || (tail >> 8) == 1;

		}

		if (waited || !(wait.status & BIT(16)) || !complete || !(state & BIT(31)) ||
		    ((error || message) && !tail_valid) ||
		    mbx != header.width_mbs - 1 || mby != picture_mbs_height - 1)
			return -EIO;
		/* Stop before observing DMA guards or writing output. */
		wr(ctx, 0x0100, rd(ctx, 0x0100) | 1);
		if (poll_word(ctx, 0x0104, 0x11, 0x11)) return -EIO;
		wr(ctx, 0x1108, 0x101); wr(ctx, 0x1108, 0); dma_wmb();
		for (unsigned i = 0; i < MAX_SLOTS; ++i)
			if (b[4 + i].cpu && check_pattern(b[4 + i].cpu + slot_used[i],
					  b[4 + i].size - slot_used[i], 0xd3)) {
				 return -EIO;
			}
		if (check_pattern(b[0].cpu + INPUT_SIZE, GUARD_SIZE, 0xd3)) {

			return -EIO;
		}
		for (unsigned int i = 1; i < 4; ++i)
			if (check_pattern(b[i].cpu + WORK_SIZE, GUARD_SIZE, 0xd3)) {

				return -EIO;
			}
 memcpy(vb2_plane_vaddr(&dst->vb2_buf,0),out->cpu,pixels);
 cache[plan.output]=(struct mt6797_h264_cached_picture){
  .timestamp=p->reference_ts,.top_poc=p->decode.top_field_order_cnt,.bottom_poc=p->decode.bottom_field_order_cnt,
  .frame_number=p->decode.frame_num,.width=p->width,.height=p->height,.allocated=1,.valid=1,.field_mask=3,
  .field_timestamp={p->reference_ts,p->reference_ts}};
 vb2_set_plane_payload(&dst->vb2_buf,0,pixels);
 return 0;
}
