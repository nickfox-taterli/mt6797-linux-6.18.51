/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MT6797_HEVC_H
#define MT6797_HEVC_H
#include "mt6797-vdec.h"
#define MAX_REFS 16
struct mt6797_hevc_control {
	u32 width, height, aligned_width, aligned_height;
	u32 lcu_w, lcu_h, log2_ctb;
	u32 sps0, sps1, sps_size, pps_word, pps_qp, pps_tile, pps_dbk;
	u32 bytes, slice_offset, start_code_bits;
	u32 slice_type, is_idr, nal_type, pic_order_cnt_lsb, reference_ts;
	u32 rps_neg, rps_pos;
	u32 rps_neg_delta[MAX_REFS], rps_neg_used[MAX_REFS], rps_pos_used[MAX_REFS];
	u64 l0_ref_ts[MAX_REFS], l1_ref_ts[MAX_REFS];
	s32 l0_poc_diff[MAX_REFS], l1_poc_diff[MAX_REFS];
	u32 num_ref_idx_l1_active_minus1, collocated_from_l0_flag, mvd_l1_zero_flag;
	u32 control_version, is_reference, keep_ref_count; u64 keep_ref_ts[MAX_REFS];
	u32 rps_prediction_ref_count;
	s32 top_field_order_cnt;
	u32 num_ref_idx_l0_active_minus1;
	u32 header_bit_size, sao_luma, sao_chroma;
	u32 weights_start_bit, weights_end_bit;
	u32 cabac_init_flag, collocated_ref_idx;
	s32 slice_qp, slice_qp_delta, beta_offset_div2, tc_offset_div2;
	u32 slice_temporal_mvp_enabled_flag, five_minus_max_num_merge_cand;
	u32 deblocking_filter_disabled_flag, loop_filter_across_slices_enabled_flag;
	s32 pps_cb_qp_offset, pps_cr_qp_offset;
	u32 tile_x[8], tile_y[8];
	s32 slice_cb_qp_offset, slice_cr_qp_offset;
	u32 num_long_term, num_long_term_sps, num_long_term_used;
	u32 l0_long_term[MAX_REFS], l1_long_term[MAX_REFS], keep_ref_long_term[MAX_REFS];
	u32 scaling_enabled, pic_output_flag;
	u32 lists_modification_present_flag;
	u32 ref_table_count, picture_has_inter, rpl_start_bit, rpl_end_bit;
	unsigned char scaling[4][6][64], scaling_dc[4][6];
};

struct mt6797_hevc_state {
 struct mt6797_hevc_control control;
 struct buffer picture[MAX_SLOTS],motion[MAX_SLOTS];
 u64 timestamps[MAX_SLOTS];
 s32 pocs[MAX_SLOTS];
};
int mt6797_hevc_stage(struct vdec_ctx *ctx);
int mt6797_hevc_run(struct vdec_ctx *ctx,struct vb2_v4l2_buffer *dst);
void mt6797_hevc_forget(struct vdec_ctx *ctx,unsigned index);
#endif
