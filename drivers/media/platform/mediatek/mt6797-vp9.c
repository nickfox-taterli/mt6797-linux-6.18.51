// SPDX-License-Identifier: GPL-2.0-only
/* MT6797 VP9 algorithm ported from the qualified open hardware path. */
#include <linux/vmalloc.h>
#include "mt6797-vp9.h"
#include "mt6797-vp9-quant.h"
#define VLD(o) (0x1000+(o))
#define TOP(o) (0x1800+(o))
#define MC(o) (0x2000+(o))
#define MV(o) (0x4000+(o))
#define PP(o) (0x5000+(o))
#define VP9(o) (0x8400+(o))
#define BS2(o) (0x7800+(o))
static void wr(struct vdec_ctx *ctx,unsigned off,u32 value){writel(value,ctx->dev->regs+off);}
static u32 rd(struct vdec_ctx *ctx,unsigned off){return readl(ctx->dev->regs+off);}
static int poll_word(struct vdec_ctx *ctx,unsigned off,u32 mask,u32 value){u32 n;return readl_poll_timeout(ctx->dev->regs+off,n,(n&mask)==value,1,100000);}
struct count_memory {
 u32 raw[4096];struct v4l2_vp9_frame_symbol_counts counts;
 u32 y[4][10],skip[3][2],filters[4][3];
 u32 sign[2][2],class0[2][2],classes[2][11],bits[2][10][2],class0_fp[2][2][4],fp[2][4],class0_hp[2][2],hp_counts[2][2];
};
static int signed_bits(unsigned value, unsigned bits)
{
	return (int)(value&((1U<<bits)-1))-(int)((value&(1U<<(bits-1)))<<1);
}
static int validate_segmentation(const struct mt6797_vp9_control *c)
{
	if (c->seg_enabled>1 || c->seg_update_map>1 || c->seg_temporal>1 || c->seg_abs>1 ||
	    c->seg_update_data>1 || c->seg_tree1>0xffffff || c->seg_pred>0xffffff ||
	    (!c->seg_enabled && (c->seg_update_map || c->seg_update_data || c->seg_temporal)) ||
	    (c->seg_temporal && !c->seg_update_map))return -1;
	for (unsigned i=0;i<8;i++) {
		u32 f=c->seg_feature[i], mask=f>>24;
		int q=signed_bits(f,9), lf=signed_bits(f>>9,7);
		if ((f&~0x0f03ffffU) || q < -255 || lf < -63 ||
		    (!(mask&1) && q) || (!(mask&2) && lf) || (!(mask&4) && (f&0x30000)))return -1;
	}
	return 0;
}
static int plan_tiles(const struct mt6797_vp9_control *c, const u8 *data, struct tiles *t)
{
	unsigned sbcols=(c->width+63)/64, sbrows=(c->height+63)/64;
	unsigned min=0, max=0;
	while ((64U<<min)<sbcols) min++;
	while ((sbcols>>(max+1))>=4) max++;
	if (c->tile_cols_log2<min || c->tile_cols_log2>max || c->tile_rows_log2>2)
		goto bad;
	t->cols=1U<<c->tile_cols_log2;
	unsigned rows=1U<<c->tile_rows_log2, offset=c->tile_offset;
	for (unsigned row=0;row<rows;row++) {
		unsigned h=(((row+1)*sbrows)>>c->tile_rows_log2)-((row*sbrows)>>c->tile_rows_log2);
		if (h) t->rows++;
		for (unsigned col=0;col<t->cols;col++) {
			unsigned n=c->bytes-offset;
			if (row+1!=rows || col+1!=t->cols) {
				if (n<4) goto bad;
				n=(u32)data[offset]<<24 | (u32)data[offset+1]<<16 |
				  (u32)data[offset+2]<<8 | data[offset+3];
				offset+=4;
			}
			if (!n || n>c->bytes-offset) goto bad;
			unsigned w=(((col+1)*sbcols)>>c->tile_cols_log2)-((col*sbcols)>>c->tile_cols_log2);
			/* Syntax may name empty rows. Consume their payloads, but the
			 * hardware grid contains only rows with visible superblocks. */
			if (h) {
				if (!w || t->count==MAX_TILES) goto bad;
				t->tile[t->count++]=(struct tile){.offset=offset,.bytes=n,.grid=((h-1)<<6)|(w-1)};
			}
			offset+=n;
		}
	}
	return 0;
bad:
	return -1;
}
static void context_codec(struct v4l2_vp9_frame_context *p, u8 raw[2560], int pack)
{
 u8 *a = raw, *b = raw + 2048;
 if (pack) memset(raw, 0, 2560);
#define transfer(dst, src, bytes) do { if (pack) memcpy(dst, src, bytes); else memcpy(src, dst, bytes); } while (0)
	unsigned row = 0;
	for (unsigned tx = 0; tx < 4; tx++)
		for (unsigned plane = 0; plane < 2; plane++)
			for (unsigned ref = 0; ref < 2; ref++)
				for (unsigned band = 0; band < 6; band++, row++) {
					u8 data[20] = {0};
					unsigned contexts = band ? 6 : 3;
                    if (pack) {
                        memcpy(data, p->coef[tx][plane][ref][band], contexts * 3);
                        memcpy(a + row * 16, data, 16); memcpy(b + row * 4, data + 16, 4);
                    } else {
                        memcpy(data, a + row * 16, 16); memcpy(data + 16, b + row * 4, 4);
                        memcpy(p->coef[tx][plane][ref][band], data, contexts * 3);
                    }
				}
	for (unsigned i = 0; i < 4; i++) {
		transfer(a + (96 + i) * 16, p->y_mode[i], 9);
		transfer(a + (100 + i) * 16, p->interp_filter[i], 2);
	}
	transfer(a + 106 * 16, p->comp_mode, 5);
	transfer(a + 107 * 16, p->comp_ref, 5);
	transfer(a + 108 * 16, p->single_ref, 10);
	transfer(a + 109 * 16, p->mv.joint, 3);
	for (unsigned i = 0; i < 2; i++) {
		u8 *m = a + (110 + i) * 16;
		transfer(&m[0], &p->mv.sign[i], 1);
		transfer(m + 1, p->mv.classes[i], 10);
		m = a + (112 + i) * 16;
		transfer(&m[0], &p->mv.class0_bit[i], 1);
		transfer(m + 1, p->mv.bits[i], 10);
		m = a + (114 + i) * 16;
		transfer(m, p->mv.class0_fr[i], 6);
		transfer(m + 6, p->mv.fr[i], 3);
		transfer(&m[9], &p->mv.class0_hp[i], 1);
		transfer(&m[10], &p->mv.hp[i], 1);
	}
	for (unsigned i = 0; i < 10; i++)
		transfer(a + (116 + i) * 16, p->uv_mode[i], 9);
	for (unsigned i = 0; i < 16; i++)
		transfer(b + (96 + i) * 4, p->partition[i], 3);
	for (unsigned i = 0; i < 7; i++)
		transfer(b + (112 + i) * 4, p->inter_mode[i], 3);
	transfer(b + 119 * 4, p->skip, 3);
	for (unsigned i = 0; i < 2; i++) {
		transfer(b + (120 + i) * 4, p->tx8[i], 1);
		transfer(b + (122 + i) * 4, p->tx16[i], 2);
		transfer(b + (124 + i) * 4, p->tx32[i], 3);
	}
	transfer(b + 126 * 4, p->is_inter, 4);
#undef transfer
}
static void write_raw_probs(struct vdec_ctx *ctx,const u32 both[640])
{
 wr(ctx,VP9(0x198), BIT(16));
 for (unsigned i=0;i<512;i++) wr(ctx,VP9(0x19c),both[i]);
 wr(ctx,VP9(0x1a0), BIT(16));
 for (unsigned i=0;i<128;i++) wr(ctx,VP9(0x1a4),both[512+i]);
}
static void read_raw_probs(struct vdec_ctx *ctx,u32 both[640])
{
 for (unsigned i=0;i<512;i++) {wr(ctx,VP9(0x198),i);both[i]=rd(ctx,VP9(0x19c));}
 for (unsigned i=0;i<128;i++) {wr(ctx,VP9(0x1a0),i);both[512+i]=rd(ctx,VP9(0x1a4));}
}
static void load_probs(struct vdec_ctx *ctx,const struct v4l2_vp9_frame_context *probs)
{
 struct mt6797_vp9_state *v=ctx->vp9;
 v->scratch=*probs;context_codec(&v->scratch,(u8 *)v->raw,1);write_raw_probs(ctx,v->raw);
}
static int update_probs(struct vdec_ctx *ctx,unsigned bit)
{
	wr(ctx,VP9(0xc8), BIT(bit));
	wr(ctx,VP9(0xc8), 0);
	return poll_word(ctx,VP9(0xcc), BIT(bit), BIT(bit));
}
static int adapt_from_counts(struct vdec_ctx *ctx,struct v4l2_vp9_frame_context *probs, unsigned inter,
 unsigned last_type, unsigned refmode, unsigned filter, unsigned txmode, unsigned hp)
{
 struct count_memory *mem=kvzalloc(sizeof(*mem),GFP_KERNEL);
 if(!mem)return -ENOMEM;
 u32 *raw=mem->raw;
 const unsigned lengths[4]={576,1024,1024,544};
 for (unsigned bank=0;bank<4;bank++)
  for(unsigned i=0;i<lengths[bank];i++) {
   wr(ctx,VP9(0x1b0+bank*8),i);raw[bank*1024+i]=rd(ctx,VP9(0x1b4+bank*8));
  }
 struct v4l2_vp9_frame_symbol_counts *counts=&mem->counts;
 unsigned eob=0,coef=1024;
 for(unsigned i=0;i<4096;i++) {
  if (raw[i]>16777216) {goto fail;}

 }
 for(unsigned t=0;t<4;t++)for(unsigned p=0;p<2;p++)for(unsigned r=0;r<2;r++)
 for(unsigned band=0;band<6;band++)for(unsigned context=0;context<(band?6U:3U);context++) {
  if(raw[eob]<raw[coef+3]) {goto fail;}
  counts->coeff[t][p][r][band][context]=(void *)&raw[coef];
  counts->eob[t][p][r][band][context][0]=&raw[eob];
  counts->eob[t][p][r][band][context][1]=&raw[coef+3];
  coef+=4;eob+=band==0&&context==2?2:band==5&&context==5?3:1;
 }
 unsigned n=3136;
#define MAP(field,words) do {counts->field=(void *)&raw[n];n+=(words);} while(0)
 MAP(intra_inter,8);MAP(comp,12);MAP(comp_ref,12);MAP(single_ref,20);MAP(mv_mode,28);

 for(unsigned i=0;i<4;i++){memcpy(mem->y[i],raw+n,40);n+=12;}counts->y_mode=&mem->y;
 MAP(uv_mode,100);MAP(partition,64);

 for(unsigned i=0;i<4;i++){memcpy(mem->filters[i],raw+n,12);n+=4;}counts->filter=&mem->filters;
 MAP(tx8p,4);MAP(tx16p,8);MAP(tx32p,8);
 for(unsigned i=0;i<3;i++){memcpy(mem->skip[i],raw+n,8);n+=4;}counts->skip=&mem->skip;
 MAP(mv_joint,4);

 for(unsigned i=0;i<2;i++) {
#define COPY(dst,words,advance) do {memcpy(dst,raw+n,(words)*4);n+=(advance);} while(0)
 COPY(mem->sign[i],2,2);COPY(mem->class0[i],2,2);COPY(mem->classes[i],11,12);COPY(mem->bits[i],20,24);
 COPY(mem->class0_fp[i],8,8);COPY(mem->fp[i],4,4);COPY(mem->class0_hp[i],2,2);COPY(mem->hp_counts[i],2,2);
#undef COPY
 }
#undef MAP
 counts->sign=&mem->sign;counts->class0=&mem->class0;counts->classes=&mem->classes;counts->bits=&mem->bits;
 counts->class0_fp=&mem->class0_fp;counts->fp=&mem->fp;counts->class0_hp=&mem->class0_hp;counts->hp=&mem->hp_counts;
 v4l2_vp9_adapt_coef_probs(probs,counts,inter&&!last_type,!inter);
 if(inter)v4l2_vp9_adapt_noncoef_probs(probs,counts,refmode,filter,txmode,hp?V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV:0);
 kvfree(mem);return 0;
fail:
 kvfree(mem);return -EIO;
}

void mt6797_vp9_forget(struct vdec_ctx *ctx,unsigned index)
{
 struct mt6797_vp9_state *v=ctx->vp9;if(!v || index>=MAX_SLOTS)return;
 struct buffer *b=&v->image[index];if(b->cpu)dma_free_coherent(ctx->dev->dev,b->size,b->cpu,b->dma);memset(b,0,sizeof(*b));v->timestamps[index]=0;
}
void mt6797_vp9_release(struct vdec_ctx *ctx)
{
 struct mt6797_vp9_state *v=ctx->vp9;if(!v)return;
 for(unsigned i=0;i<MAX_SLOTS;i++)mt6797_vp9_forget(ctx,i);
 for(unsigned i=0;i<3;i++){struct buffer *b=&v->work[i];if(b->cpu)dma_free_coherent(ctx->dev->dev,b->size,b->cpu,b->dma);}
 kfree(v);ctx->vp9=NULL;
}
static int backing(struct vdec_ctx *ctx,struct buffer *b,unsigned size)
{
 if(b->cpu && b->used!=size){dma_free_coherent(ctx->dev->dev,b->size,b->cpu,b->dma);memset(b,0,sizeof(*b));}
 return b->cpu?0:vdec_buffer(ctx,b,size+GUARD_SIZE);
}
static unsigned qclamp(int q){return clamp(q,0,255);}
int mt6797_vp9_run(struct vdec_ctx *ctx,struct vb2_v4l2_buffer *dst)
{
 struct mt6797_vp9_state *v=ctx->vp9;struct mt6797_vp9_control c=v->control;
 unsigned output=dst->vb2_buf.index,stride=ALIGN(c.width,64),rows=ALIGN(c.height,64),luma=stride*rows,pixels=luma*3/2;
 if(output>=MAX_SLOTS || !vb2_plane_vaddr(&dst->vb2_buf,0) || vb2_plane_size(&dst->vb2_buf,0)<pixels)return -EINVAL;
 bool inter=c.frame_type && !c.intra_only;
 unsigned reset_mask=(!c.frame_type || c.error_resilient || (c.intra_only && c.reset_frame_context==3))?15:
  (c.intra_only && c.reset_frame_context==2)?BIT(c.reset_context_idx):0;
 struct buffer b[8]={0};b[0]=ctx->buffers[0];
 if(backing(ctx,&v->image[output],pixels) || backing(ctx,&v->work[0],SZ_1M) || backing(ctx,&v->work[1],MAX_TILES*16) || backing(ctx,&v->work[2],SEG_SIZE))return -ENOMEM;
 b[1]=v->image[output];b[2]=v->work[0];b[6]=v->work[1];b[7]=v->work[2];
 unsigned used[8]={INPUT_SIZE,pixels,SZ_1M,pixels,pixels,pixels,MAX_TILES*16,SEG_SIZE};
 const struct v4l2_ctrl_vp9_frame *f=vdec_control(ctx,V4L2_CID_STATELESS_VP9_FRAME,NULL);
 u64 references[3]={f->last_frame_ts,f->golden_frame_ts,f->alt_frame_ts};
 for(unsigned i=0;i<3;i++){
  if(!inter){b[3+i]=b[1];continue;}
  unsigned slot;for(slot=0;slot<MAX_SLOTS;slot++)if(v->timestamps[slot]==references[i])break;
  if(slot==MAX_SLOTS || slot==output)return -ENOENT;
  b[3+i]=v->image[slot];used[3+i]=b[3+i].used;
 }
 if(!c.use_prev_mi)memset(b[2].cpu,0,used[2]);
 if(!inter || c.error_resilient || c.size_changed)memset(b[7].cpu,0,SEG_SIZE);
 struct tiles tiles={0};if(validate_segmentation(&c) || plan_tiles(&c,b[0].cpu,&tiles))return -EINVAL;
	dma_wmb();
	writel(1,ctx->dev->gcon);
	dma_wmb();
	wr(ctx,0xf4, 0);
	wr(ctx,0xec, 1);
	writel(1,ctx->dev->gcon+0x18);
	dma_wmb();
	wr(ctx,VLD(0x108), 0x101);
	wr(ctx,0xc8, 0xfdff);
	wr(ctx,0xcc, 0x71e31182);
	wr(ctx,0x178, 0xfffff0c1);
	wr(ctx,0x84, 2);
	wr(ctx,VLD(0x108), 0);
	wr(ctx,4, 3);
	wr(ctx,VP9(0xa4), 1);
	wr(ctx,VP9(0x1a8), 1);
	if (poll_word(ctx,VP9(0x1a8), 1, 0))
		return -EIO;
	u32 end = b[0].dma + ((c.bytes + 512) & ~63U);
	for (unsigned i = 0; i < 2; i++) {
		unsigned bank = i ? 0x7800 : 0x1000;
		wr(ctx,bank + 0xb4, b[0].dma >> 6);
		wr(ctx,bank + 0xb8, end >> 6);
		wr(ctx,bank + 0xec, 0x10000000);
	}
	wr(ctx,VLD(0xb0), b[0].dma);
	wr(ctx,VLD(0x110), end);
	for (unsigned i = 0; i < 5; i++)
		wr(ctx,VLD(0x108), 0x100);
	wr(ctx,VLD(0x108), 0);
	wr(ctx,VLD(0x8c), BIT(20));
	if (poll_word(ctx,VLD(0xe8), 1, 1))
		return -EIO;
	wr(ctx,VLD(0x8c), BIT(23));
	wr(ctx,VP9(0x144), 1);
	wr(ctx,VP9(0x144), 0);
	for (unsigned bits = c.uncompressed_size * 8; bits;) {
		unsigned n = bits > 32 ? 32 : bits;
		(void)rd(ctx,VP9(n * 4));
		bits -= n;
	}
	wr(ctx,TOP(0x70), c.width | (c.height << 16));
	wr(ctx,TOP(0x68), ((c.width + 63) / 64 - 1) | (((c.height + 63) / 64 - 1) << 16));
	wr(ctx,VP9(0xa4), 1);
	unsigned lossless = !(c.qindex || c.y_dc_delta || c.uv_dc_delta || c.uv_ac_delta);
	wr(ctx,VP9(0xa8), c.high_precision_mv << 31 | c.interp_filter << 24 |
			  c.seg_temporal << 10 | c.seg_update_map << 9 | c.seg_enabled << 8 | c.error_resilient << 4 |
			  c.parallel << 3 | lossless << 2 | c.intra_only << 1 | c.frame_type);
	u32 ref_skip=0, lf_control=0, lf_data[2]={0};
	if (c.seg_enabled) {
		wr(ctx,VP9(0x1f0),c.seg_tree0);wr(ctx,VP9(0x1f4),c.seg_tree1);wr(ctx,VP9(0x1f8),c.seg_pred);
		wr(ctx,TOP(0x118),b[7].dma);wr(ctx,TOP(0x11c),b[7].dma);
		lf_control=c.seg_abs<<8;
		for (unsigned i=0;i<8;i++) {
			u32 f=c.seg_feature[i],mask=f>>24;
			ref_skip|=((f>>16)&3)<<(8+2*i) | ((mask>>2)&1)<<(24+i) | ((mask>>3)&1)<<i;
			lf_control|=((mask>>1)&1)<<i;
			lf_data[i/4]|=(u32)(signed_bits(f>>9,7)&255)<<((i%4)*8);
		}
	}
	wr(ctx,VP9(0xac),ref_skip);
	wr(ctx,PP(0xe4),lf_control);wr(ctx,PP(0xe8),lf_data[0]);wr(ctx,PP(0xec),lf_data[1]);
	wr(ctx,PP(0xd8), c.delta_enabled | (c.filter_level << 16) | (c.sharpness << 8));
	wr(ctx,PP(0xdc), c.ref_deltas);
	wr(ctx,PP(0xe0), c.mode_deltas);
 load_probs(ctx,(reset_mask&BIT(c.context_idx))?&v4l2_vp9_default_probs:&v->probs[c.context_idx]);
 read_raw_probs(ctx,v->raw);v->previous=v4l2_vp9_default_probs;context_codec(&v->previous,(u8 *)v->raw,0);
	wr(ctx,VP9(0x144), 1);
	wr(ctx,VP9(0x144), 0);
	wr(ctx,VP9(0x110), 1);
	if (poll_word(ctx,VP9(0x110), BIT(16), BIT(16)))
		return -EIO;
	unsigned tx = lossless ? 0 : rd(ctx,VP9(0x88));
	if (tx == 3)
		tx += rd(ctx,VP9(0x84));
	wr(ctx,VP9(0xb4), tx);
	if ((tx == 4 && update_probs(ctx,0)) || update_probs(ctx,1) || update_probs(ctx,2))
		return -EIO;
	unsigned refmode = 0, compound = 0;
	if (inter) {
		if (update_probs(ctx,3) || (c.interp_filter == 4 && update_probs(ctx,4)) || update_probs(ctx,5))
			return -EIO;
		unsigned bias = (c.sign_bias >> 1) & 7;
		if (bias != 0 && bias != 7) {
			if (rd(ctx,VP9(0x84)))
				refmode = rd(ctx,VP9(0x84)) ? 2 : 1;
			if (refmode) {
				unsigned fixed, var0, var1;
				if ((bias & 1) == ((bias >> 1) & 1)) {
					fixed = 3;
					var0 = 1;
					var1 = 2;
				} else if ((bias & 1) == ((bias >> 2) & 1)) {
					fixed = 2;
					var0 = 1;
					var1 = 3;
				} else {
					fixed = 1;
					var0 = 2;
					var1 = 3;
				}
				compound = ((var0 | (var1 << 2)) << 24) | (fixed << 8);
			}
		}
		if ((refmode == 2 && update_probs(ctx,6)) || (refmode != 1 && update_probs(ctx,7)) ||
		    (refmode != 0 && update_probs(ctx,8)) || update_probs(ctx,9) || update_probs(ctx,10) ||
		    update_probs(ctx,11) || update_probs(ctx,12) ||
		    (c.high_precision_mv && update_probs(ctx,13)))
			return -EIO;
	}
	u32 parsed_bits = rd(ctx,VP9(0x138));
	if (parsed_bits > c.compressed_size * 8) {
		return -EIO;
	}
	wr(ctx,VP9(0xb0), compound | (c.sign_bias << 16) | refmode);
	wr(ctx,MV(0x3c4), c.use_prev_mi);
	wr(ctx,MV(0x3c0), b[2].dma >> 4);
	wr(ctx,MV(0x20c), b[2].dma >> 4);
	for (unsigned i = 0; i < 3; i++) {
		u64 address = b[inter ? 3 + i : 1].dma;
		wr(ctx,MC(0xd28) + i * 4, address);
		wr(ctx,MC(0xd34) + i * 4, address + (inter ? used[3+i]*2/3 : luma));
	}
	if (inter) {
		u32 scaling=0,limits=0;unsigned scaled=0;
		for (unsigned i = 0; i < 3; i++) {
			unsigned w=c.ref_width[i],h=c.ref_height[i],sx=(w<<14)/c.width,sy=(h<<14)/c.height;
			unsigned enable=w!=c.width || h!=c.height;
			unsigned bx=enable ? (sx<=16384?4:3) : 7,by=enable ? (sy<=16384?4:3) : 7;
			scaling|=enable<<((2-i)*4);scaled+=enable;
			limits|=bx<<(24-i*4) | by<<(8-i*4);
			wr(ctx,MC(848*4)+i*4,((sx>>10)<<16)|(sy>>10));
			wr(ctx,MC(854*4)+i*8,sx);wr(ctx,MC(855*4)+i*8,sy);
			wr(ctx,MC(851 * 4) + i * 4, (w << 16) | h);
			wr(ctx,MC(860 * 4) + i * 4, ((w+63)&~63U)/16);
		}
		wr(ctx,MC(864*4),scaling);wr(ctx,MC(863 * 4),limits);
		}
	wr(ctx,MC(0xd88), ((c.width + 7) / 8) << 16 | ((c.height + 7) / 8));
	wr(ctx,MC(0x220), 1);
	wr(ctx,MC(0x224), b[1].dma >> 9);
	wr(ctx,MC(0x228), (b[1].dma + luma) >> 8);
	wr(ctx,MC(0x22c), stride / 16);
	wr(ctx,MC(0x238), 3);
	wr(ctx,MC(0x250), 1);
	wr(ctx,MC(0x260), stride / 16 - 1);
	wr(ctx,MC(0x264), rows / 16 - 1);
	wr(ctx,PP(0xf0), c.width | (c.height << 16));
	wr(ctx,PP(0x504), 1);
	for (unsigned i = 0; i < 8; i++) {
		u32 f=c.seg_feature[i];int q=c.qindex;
		if(c.seg_enabled && (f&BIT(24)))q=qclamp((c.seg_abs?0:q)+signed_bits(f,9));
		unsigned yd=dc_qlookup[qclamp(q+c.y_dc_delta)],ya=ac_qlookup[q],
			 ud=dc_qlookup[qclamp(q+c.uv_dc_delta)],ua=ac_qlookup[qclamp(q+c.uv_ac_delta)];
		wr(ctx,VLD(0x260),i*2);wr(ctx,VLD(0x264),yd|(ya<<16));
		wr(ctx,VLD(0x260),i*2+1);wr(ctx,VLD(0x264),ud|(ua<<16));
	}
	wr(ctx,PP(0x504), 0);
	if (lossless)
		wr(ctx,PP(0x9c0), 0xe00);
	wr(ctx,MC(0xa60), 0);
	wr(ctx,VP9(0xc4), ((tiles.rows-1)<<16)|(tiles.cols-1));
	wr(ctx,VP9(0x100), 0);
	for (unsigned i=0;i<tiles.count;i++) {
		struct tile *t=&tiles.tile[i];
		u32 address=b[0].dma+t->offset;
		t->words[0]=t->bytes*8+(address&15)*8;
		t->words[1]=address&~15U;
		t->words[2]=(address&15)*8;
		t->words[3]=t->grid;
		if (tiles.count<=8)
			for (unsigned j=0;j<4;j++) wr(ctx,VP9(0x104),t->words[j]);
		else memcpy(b[6].cpu+i*16,t->words,16);
	}
	wr(ctx,VLD(0x140), 0x100014);
	wr(ctx,BS2(0x140), 0x100014);
	if (tiles.count>8) wr(ctx,TOP(0xb8),b[6].dma);
	wr(ctx,VP9(0xbc), tiles.count<=8 ? 0x10111 : 0x00111);
	wr(ctx,VP9(0x1a8), 1);
	if (poll_word(ctx,VP9(0x1a8), 1, 0))
		return -EIO;
	wr(ctx,TOP(0x124), 0x07200010);
	dma_wmb();
	reinit_completion(&ctx->dev->done);WRITE_ONCE(ctx->dev->status,0);
 wr(ctx,VP9(0xb8), 1);
	struct {u32 status;} wait;
 int waited=wait_for_completion_timeout(&ctx->dev->done,msecs_to_jiffies(500))?0:-ETIMEDOUT;
 wait.status=READ_ONCE(ctx->dev->status);
	if (waited || !(wait.status & BIT(16)))
		return -EIO;
 wr(ctx,VP9(0x100),0);
 for(unsigned i=0;i<tiles.count;i++) {
  struct tile *t=&tiles.tile[i];u32 s[4];
  if(tiles.count<=8)for(unsigned j=0;j<4;j++)s[j]=rd(ctx,VP9(0x104));
  else memcpy(s,b[6].cpu+i*16,16);
  /* DONE16 alone may accompany a truncated arithmetic stream. Verify
   * every tile, including the completed boolean cursor/error state. */
  if(s[0]!=t->words[0] || s[1]!=t->words[1] || s[2]<=t->words[2] || s[2]>s[0] ||
     (s[3]&BIT(15)) || (s[3]&0x7fff)!=t->grid) {
   return -EIO;
  }
 }

	wr(ctx,0x100, rd(ctx,0x100) | 1);
	if (poll_word(ctx,0x104, 0x11, 0x11))
		return -EIO;
	dma_wmb();
	for (unsigned i = 0; i < ARRAY_SIZE(b); i++)
		for (unsigned n = used[i]; n < b[i].size; n++)
			if (b[i].cpu[n] != 0xd3) {
				return -EIO;
			}
 if (!c.parallel && !c.error_resilient && c.refresh_context) {
  if (!inter) {
   v->forward=v4l2_vp9_default_probs;
   read_raw_probs(ctx,v->raw);context_codec(&v->forward,(u8 *)v->raw,0);
   memcpy(v->previous.tx8,v->forward.tx8,sizeof(v->previous.tx8));
   memcpy(v->previous.tx16,v->forward.tx16,sizeof(v->previous.tx16));
   memcpy(v->previous.tx32,v->forward.tx32,sizeof(v->previous.tx32));
   memcpy(v->previous.skip,v->forward.skip,sizeof(v->previous.skip));
  }
  if (adapt_from_counts(ctx,&v->previous, inter, c.last_frame_type, refmode, c.interp_filter, tx, c.high_precision_mv)) return -EIO;
  load_probs(ctx,&v->previous);
 }

 for(unsigned i=0;i<4;i++)if(reset_mask&BIT(i))v->probs[i]=v4l2_vp9_default_probs;
 if(c.refresh_context){read_raw_probs(ctx,v->raw);context_codec(&v->probs[c.context_idx],(u8 *)v->raw,0);}
 dma_rmb();memcpy(vb2_plane_vaddr(&dst->vb2_buf,0),b[1].cpu,pixels);vb2_set_plane_payload(&dst->vb2_buf,0,pixels);
 v->timestamps[output]=ctx->timestamp;v->width[output]=c.width;v->height[output]=c.height;
 v->last_width=c.width;v->last_height=c.height;v->last_show=!!(f->flags&V4L2_VP9_FRAME_FLAG_SHOW_FRAME);v->last_type=c.frame_type;v->last_intra=c.intra_only;v->valid=true;
 return 0;
}
