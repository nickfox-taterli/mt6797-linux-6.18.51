/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MT6797_AUTOK_ANALYSIS_H
#define MT6797_AUTOK_ANALYSIS_H
#include <linux/types.h>

struct BOUND_INFO {
	unsigned int Bound_Start;
	unsigned int Bound_End;
	unsigned int Bound_width;
	bool is_fullbound;
};

#define BD_MAX_CNT 4	/* Max Allowed Boundary Number */
struct AUTOK_SCAN_RES {
	/* Bound info record, currently only allow max to 2 bounds exist,
	   but in extreme case, may have 4 bounds */
	struct BOUND_INFO bd_info[BD_MAX_CNT];
	/* Bound cnt record, must be in rang [0,3] */
	unsigned int bd_cnt;
	/* Full boundary cnt record */
	unsigned int fbd_cnt;
};

struct AUTOK_REF_INFO {
	/* inf[0] - rising edge res, inf[1] - falling edge res */
	struct AUTOK_SCAN_RES scan_info[2];
	/* optimised sample edge select */
	unsigned int opt_edge_sel;
	/* optimised dly cnt sel */
	unsigned int opt_dly_cnt;
	/* 1clk cycle equal how many delay cell cnt, if cycle_cnt is 0,
	   that is cannot calc cycle_cnt by current Boundary info */
	unsigned int cycle_cnt;
};


int autok_simple_score64(char *text, u64 failures);
int autok_check_scan_res64(u64 failures, struct AUTOK_SCAN_RES *result);
int autok_pad_dly_sel(struct AUTOK_REF_INFO *info);
#endif
