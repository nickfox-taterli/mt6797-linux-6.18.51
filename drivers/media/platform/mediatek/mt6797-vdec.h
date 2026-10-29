/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MT6797_VDEC_H
#define MT6797_VDEC_H
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/workqueue.h>
#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include "mt6797-h264-controls.h"
#define INPUT_SIZE SZ_1M
#define WORK_SIZE SZ_128K
#define GUARD_SIZE SZ_4K
#define MAX_SLOTS MT6797_H264_PICTURE_SLOTS
#define MAX_ACTIVE_REFS 32
#define MAX_DPB_REFS MT6797_H264_DPB_ENTRIES
struct buffer { void *cpu; dma_addr_t dma; unsigned size, used; };
struct control {
 struct v4l2_ctrl_h264_sps sps;
 struct v4l2_ctrl_h264_pps pps;
 struct v4l2_ctrl_h264_slice_params slice;
 struct mt6797_h264_decode_params decode;
 struct v4l2_ctrl_h264_pred_weights weights;
 struct v4l2_ctrl_h264_scaling_matrix scaling;
 struct mt6797_h264_weight_range weight_range;
 u64 reference_ts, field_pair_ts;
 u32 width,height,memory_reset,field_picture,bottom_field,complementary_idr;
 u32 rpl_start_bit,rpl_end_bit;
};
struct vdec_ctx;
struct vdec_dev {
 struct device *dev,*larb;
 void __iomem *regs,*gcon;
 struct clk_bulk_data *clks;
 int num_clks,irq;
 struct mutex lock;
 struct v4l2_device v4l2;
 struct media_device media;
 struct video_device video;
 struct v4l2_m2m_dev *m2m;
 struct work_struct work;
 struct completion done;
 struct vdec_ctx *job;
 u32 status;
 bool powered;
};
struct vdec_ctx {
 struct v4l2_fh fh;
 struct vdec_dev *dev;
 struct v4l2_ctrl_handler controls;
 struct v4l2_ctrl *control[8];
 struct buffer buffers[4+MAX_SLOTS];
 struct mt6797_h264_cached_picture cache[MAX_SLOTS];
 struct control picture;
 struct v4l2_ctrl_h264_decode_params native_decode;
 unsigned width,height,bytes,first_bytes,slices,sequence,last_mb;
 u64 timestamp;
 bool aborting;
};
int vdec_buffer(struct vdec_ctx *ctx,struct buffer *b,unsigned size);
int mt6797_h264_run(struct vdec_ctx *ctx,struct vb2_v4l2_buffer *dst);
#endif
