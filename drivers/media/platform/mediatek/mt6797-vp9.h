/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MT6797_VP9_H
#define MT6797_VP9_H
#include "mt6797-vdec.h"
#include <media/v4l2-vp9.h>
#define MAX_TILES 16U
#define SEG_SIZE 0x12000U
struct mt6797_vp9_control {
	u32 width, height, bytes, uncompressed_size, compressed_size, tile_offset;
	u32 error_resilient, parallel, filter_level, sharpness, delta_enabled, ref_deltas,
	    mode_deltas, qindex;
	s32 y_dc_delta, uv_dc_delta, uv_ac_delta;
	u32 frame_type, frame_index, refresh_context, context_idx, use_prev_mi,
	    high_precision_mv, interp_filter, sign_bias;
	u32 ref_index0, ref_index1, ref_index2, last_frame_type;
	u32 tile_cols_log2, tile_rows_log2;
	u32 seg_enabled, seg_update_map, seg_temporal, seg_abs, seg_tree0, seg_tree1,
	    seg_pred, seg_update_data, seg_feature[8];
	u32 intra_only, reset_frame_context, reset_context_idx;
	u32 ref_width[3],ref_height[3],size_changed;
	int seg_present;
};
struct tile {
	u32 offset, bytes, grid, words[4];
};
struct tiles {
	struct tile tile[MAX_TILES];
	unsigned count, cols, rows;
};

struct mt6797_vp9_state {
 struct mt6797_vp9_control control;
 struct buffer image[MAX_SLOTS],work[3];
 u64 timestamps[MAX_SLOTS];unsigned width[MAX_SLOTS],height[MAX_SLOTS];
 struct v4l2_vp9_frame_context probs[4],previous,forward,scratch;
 u32 raw[640];
 unsigned last_width,last_height,last_type,last_show,last_intra;
 bool valid;
};
int mt6797_vp9_stage(struct vdec_ctx *ctx);
int mt6797_vp9_run(struct vdec_ctx *ctx,struct vb2_v4l2_buffer *dst);
void mt6797_vp9_forget(struct vdec_ctx *ctx,unsigned index);
void mt6797_vp9_release(struct vdec_ctx *ctx);
#endif
