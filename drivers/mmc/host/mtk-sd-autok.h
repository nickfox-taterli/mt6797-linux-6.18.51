/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2015 MediaTek Inc.
 * MT6797 BSP autok.c boundary filtering and edge/delay/DS selection.
 * Adapted to mainline request submission in mtk-sd.c. Raw maps use 1=fail
 * inside the BSP routines; the public wrappers use 1=pass.
 * Bounds checks precede boundary writes, shifts use unsigned 64-bit masks,
 * and every selected delay is checked against its measured passing map.
 */
#ifndef __MTK_SD_AUTOK_H
#define __MTK_SD_AUTOK_H

#define AUTOK_TUNING_INACCURACY 3
#define AUTOK_MARGIN_THOLD 5
#define AUTOK_BD_WIDTH_REF 3
#define HS400_ONE_CYCLE 50
#define HS400_ACTUAL_CYCLE 30
#define AUTOK_RAWPRINT(...) pr_debug(__VA_ARGS__)
#define AUTOK_DBGPRINT(level, ...) pr_debug(__VA_ARGS__)
enum { RD_SCAN_NONE, RD_SCAN_PAD_BOUND_S, RD_SCAN_PAD_BOUND_E, RD_SCAN_PAD_MARGIN };
enum { DAT_RD_D_DLY1, DAT_RD_D_DLY1_SEL };

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
	unsigned int margin_lost;
};


static int autok_check_scan_res64(u64 rawdat, struct AUTOK_SCAN_RES *scan_res)
{
	unsigned int bit;
	unsigned int filter = 4;
	struct BOUND_INFO *pBD = (struct BOUND_INFO *)scan_res->bd_info;
	unsigned int RawScanSta = RD_SCAN_NONE;

	for (bit = 0; bit < 64; bit++) {
		if (rawdat & BIT_ULL(bit)) {
			switch (RawScanSta) {
			case RD_SCAN_NONE:
				RawScanSta = RD_SCAN_PAD_BOUND_S;
				pBD->Bound_Start = 0;
				pBD->Bound_width = 1;
				scan_res->bd_cnt += 1;
				break;
			case RD_SCAN_PAD_MARGIN:
				RawScanSta = RD_SCAN_PAD_BOUND_S;
				pBD->Bound_Start = bit;
				pBD->Bound_width = 1;
				scan_res->bd_cnt += 1;
				break;
			case RD_SCAN_PAD_BOUND_E:
				if ((filter) && ((bit - pBD->Bound_End) <= AUTOK_TUNING_INACCURACY)) {
					AUTOK_DBGPRINT(AUTOK_DBG_TRACE,
					       "[AUTOK]WARN: Try to filter the holes on raw data \r\n");
					RawScanSta = RD_SCAN_PAD_BOUND_S;

					pBD->Bound_width += (bit - pBD->Bound_End);
					pBD->Bound_End = 0;
					filter--;

					/* update full bound info */
					if (pBD->is_fullbound) {
						pBD->is_fullbound = 0;
						scan_res->fbd_cnt -= 1;
					}
				} else {
					/* No filter Check and Get the next boundary information */
					RawScanSta = RD_SCAN_PAD_BOUND_S;
					if (scan_res->bd_cnt >= BD_MAX_CNT)
						return -EINVAL;
					pBD++;
					pBD->Bound_Start = bit;
					pBD->Bound_width = 1;
					scan_res->bd_cnt += 1;
					if (scan_res->bd_cnt > BD_MAX_CNT) {
						AUTOK_RAWPRINT
						    ("[AUTOK]Error: more than %d Boundary Exist\r\n",
						     BD_MAX_CNT);
						return -1;
					}
				}
				break;
			case RD_SCAN_PAD_BOUND_S:
				pBD->Bound_width++;
				break;
			default:
				break;
			}
		} else {
			switch (RawScanSta) {
			case RD_SCAN_NONE:
				RawScanSta = RD_SCAN_PAD_MARGIN;
				break;
			case RD_SCAN_PAD_BOUND_S:
				RawScanSta = RD_SCAN_PAD_BOUND_E;
				pBD->Bound_End = bit - 1;
				/* update full bound info */
				if (pBD->Bound_Start > 0) {
					pBD->is_fullbound = 1;
					scan_res->fbd_cnt += 1;
				}
				break;
			case RD_SCAN_PAD_MARGIN:
			case RD_SCAN_PAD_BOUND_E:
			default:
				break;
			}
		}
	}
	if ((pBD->Bound_End == 0) && (pBD->Bound_width != 0))
		pBD->Bound_End = pBD->Bound_Start + pBD->Bound_width - 1;

	return 0;
}

static int autok_pad_dly_sel(struct AUTOK_REF_INFO *pInfo)
{
	struct AUTOK_SCAN_RES *pBdInfo_R = NULL; /* scan result @ rising edge */
	struct AUTOK_SCAN_RES *pBdInfo_F = NULL; /* scan result @ falling edge */
	struct BOUND_INFO *pBdPrev = NULL; /* Save the first boundary info for calc optimised dly count */
	struct BOUND_INFO *pBdNext = NULL; /* Save the second boundary info for calc optimised dly count */
	struct BOUND_INFO *pBdTmp = NULL;
	unsigned int FBound_Cnt_R = 0;	/* Full Boundary count */
	unsigned int Bound_Cnt_R = 0;
	unsigned int Bound_Cnt_F = 0;
	unsigned int cycle_cnt = 64;
	int uBD_mid_prev = 0;
	int uBD_mid_next = 0;
	int uBD_width = 3;
	int uDlySel_F = 0;
	int uDlySel_R = 0;
	int uMgLost_F = 0; /* for falling edge margin compress */
	int uMgLost_R = 0; /* for rising edge margin compress */
	unsigned int i;
	unsigned int ret = 0;

	pBdInfo_R = &(pInfo->scan_info[0]);
	pBdInfo_F = &(pInfo->scan_info[1]);
	FBound_Cnt_R = pBdInfo_R->fbd_cnt;
	Bound_Cnt_R = pBdInfo_R->bd_cnt;
	Bound_Cnt_F = pBdInfo_F->bd_cnt;

	switch (FBound_Cnt_R) {
	case 4:	/* SSSS Corner may cover 2~3T */
	case 3:
		AUTOK_RAWPRINT("[ATUOK]Warning: Too Many Full boundary count:%d\r\n", FBound_Cnt_R);
		fallthrough;
	case 2:	/* mode_1 : 2 full boudary */
		for (i = 0; i < BD_MAX_CNT; i++) {
			if (pBdInfo_R->bd_info[i].is_fullbound) {
				if (pBdPrev == NULL) {
					pBdPrev = &(pBdInfo_R->bd_info[i]);
				} else {
					pBdNext = &(pBdInfo_R->bd_info[i]);
					break;
				}
			}
		}

		if (pBdPrev && pBdNext) {
			uBD_mid_prev = (pBdPrev->Bound_Start + pBdPrev->Bound_End) / 2;
			uBD_mid_next = (pBdNext->Bound_Start + pBdNext->Bound_End) / 2;
			/* while in 2 full bound case, bd_width calc */
			uBD_width = (pBdPrev->Bound_width + pBdNext->Bound_width) / 2;
			cycle_cnt = uBD_mid_next - uBD_mid_prev;
			/* delay count sel at rising edge */
			if (uBD_mid_prev >= cycle_cnt / 2) {
				uDlySel_R = uBD_mid_prev - cycle_cnt / 2;
				uMgLost_R = 0;
			} else if ((cycle_cnt / 2 - uBD_mid_prev) > AUTOK_MARGIN_THOLD) {
				uDlySel_R = uBD_mid_prev + cycle_cnt / 2;
				uMgLost_R = 0;
			} else {
				uDlySel_R = 0;
				uMgLost_R = cycle_cnt / 2 - uBD_mid_prev;
			}
			/* delay count sel at falling edge */
			pBdTmp = &(pBdInfo_R->bd_info[0]);
			if (pBdTmp->is_fullbound) {
				/* ooooxxxooooooxxxooo */
				uDlySel_F = uBD_mid_prev;
				uMgLost_F = 0;
			} else {
				/* xooooooxxxoooooooxxxoo */
				if (pBdTmp->Bound_End > uBD_width / 2) {
					uDlySel_F = (pBdTmp->Bound_End) - (uBD_width / 2);
					uMgLost_F = 0;
				} else {
					uDlySel_F = 0;
					uMgLost_F = (uBD_width / 2) - (pBdTmp->Bound_End);
				}
			}
		} else {
			/* error can not find 2 foull boary */
			AUTOK_RAWPRINT("[AUTOK]error can not find 2 foull boudary @ Mode_1");
			return -1;
		}
		break;

	case 1:	/* rising edge find one full boundary */
		if (Bound_Cnt_R > 1) {
			/* mode_2: 1 full boundary and boundary count > 1 */
			pBdPrev = &(pBdInfo_R->bd_info[0]);
			pBdNext = &(pBdInfo_R->bd_info[1]);

			if (pBdPrev->is_fullbound)
				uBD_width = pBdPrev->Bound_width;
			else
				uBD_width = pBdNext->Bound_width;

			if ((pBdPrev->is_fullbound) || (pBdNext->is_fullbound)) {
				if (pBdPrev->Bound_Start > 0)
					cycle_cnt = pBdNext->Bound_Start - pBdPrev->Bound_Start;
				else
					cycle_cnt = pBdNext->Bound_End - pBdPrev->Bound_End;

				/* delay count sel@rising & falling edge */
				if (pBdPrev->is_fullbound) {
					uBD_mid_prev = (pBdPrev->Bound_Start + pBdPrev->Bound_End) / 2;
					uDlySel_F = uBD_mid_prev;
					uMgLost_F = 0;
					if (uBD_mid_prev >= cycle_cnt / 2) {
						uDlySel_R = uBD_mid_prev - cycle_cnt / 2;
						uMgLost_R = 0;
					} else if ((cycle_cnt / 2 - uBD_mid_prev) >
						   AUTOK_MARGIN_THOLD) {
						uDlySel_R = uBD_mid_prev + cycle_cnt / 2;
						uMgLost_R = 0;
					} else {
						uDlySel_R = 0;
						uMgLost_R = cycle_cnt / 2 - uBD_mid_prev;
					}
				} else {
					/* first boundary not full boudary */
					uBD_mid_next = (pBdNext->Bound_Start + pBdNext->Bound_End) / 2;
					uDlySel_R = uBD_mid_next - cycle_cnt / 2;
					uMgLost_R = 0;
					if (pBdPrev->Bound_End > uBD_width / 2) {
						uDlySel_F = (pBdPrev->Bound_End) - (uBD_width / 2);
						uMgLost_F = 0;
					} else {
						uDlySel_F = 0;
						uMgLost_F = (uBD_width / 2) - (pBdPrev->Bound_End);
					}
				}
			} else {
				return -1; /* full bound must in first 2 boundary */
			}
		} else if (Bound_Cnt_F > 0) {
			/* mode_3: 1 full boundary and only one boundary exist @rising edge */
			pBdPrev = &(pBdInfo_R->bd_info[0]); /* this boundary is full bound */
			pBdNext = &(pBdInfo_F->bd_info[0]);
			uBD_mid_prev = (pBdPrev->Bound_Start + pBdPrev->Bound_End) / 2;
			uBD_width = pBdPrev->Bound_width;

			if (pBdNext->Bound_Start == 0) {
				cycle_cnt = (pBdPrev->Bound_End - pBdNext->Bound_End) * 2;
			} else if (pBdNext->Bound_End == 63) {
				cycle_cnt = (pBdNext->Bound_Start - pBdPrev->Bound_Start) * 2;
			} else {
				uBD_mid_next = (pBdNext->Bound_Start + pBdNext->Bound_End) / 2;

				if (uBD_mid_next > uBD_mid_prev)
					cycle_cnt = (uBD_mid_next - uBD_mid_prev) * 2;
				else
					cycle_cnt = (uBD_mid_prev - uBD_mid_next) * 2;
			}

			uDlySel_F = uBD_mid_prev;
			uMgLost_F = 0;

			if (uBD_mid_prev >= cycle_cnt / 2) { /* case 1 */
				uDlySel_R = uBD_mid_prev - cycle_cnt / 2;
				uMgLost_R = 0;
			} else if (cycle_cnt / 2 - uBD_mid_prev <= AUTOK_MARGIN_THOLD) { /* case 2 */
				uDlySel_R = 0;
				uMgLost_R = cycle_cnt / 2 - uBD_mid_prev;
			} else if (cycle_cnt / 2 + uBD_mid_prev <= 63) { /* case 3 */
				uDlySel_R = cycle_cnt / 2 + uBD_mid_prev;
				uMgLost_R = 0;
			} else if (32 - uBD_mid_prev <= AUTOK_MARGIN_THOLD) { /* case 4 */
				uDlySel_R = 0;
				uMgLost_R = cycle_cnt / 2 - uBD_mid_prev;
			} else { /* case 5 */
				uDlySel_R = 63;
				uMgLost_R = uBD_mid_prev + cycle_cnt / 2 - 63;
			}
		} else {
			/* mode_4: falling edge no boundary found & rising edge only one full boundary exist */
			pBdPrev = &(pBdInfo_R->bd_info[0]);	/* this boundary is full bound */
			uBD_mid_prev = (pBdPrev->Bound_Start + pBdPrev->Bound_End) / 2;
			uBD_width = pBdPrev->Bound_width;

			if (pBdPrev->Bound_End > (64 - pBdPrev->Bound_Start))
				cycle_cnt = 2 * (pBdPrev->Bound_End + 1);
			else
				cycle_cnt = 2 * (64 - pBdPrev->Bound_Start);

			uDlySel_R = (uBD_mid_prev >= 32) ? 0 : 63;
			uMgLost_R = 0xFF; /* Margin enough donot care margin lost */
			uDlySel_F = uBD_mid_prev;
			uMgLost_F = 0xFF; /* Margin enough donot care margin lost */

			AUTOK_RAWPRINT("[AUTOK]Warning: 1T > %d\r\n", cycle_cnt);
		}
		break;

	case 0:	/* rising edge cannot find full boudary */
		if (Bound_Cnt_R == 2) {
			pBdPrev = &(pBdInfo_R->bd_info[0]);
			pBdNext = &(pBdInfo_F->bd_info[0]); /* this boundary is full bound */

			if (pBdNext->is_fullbound) {
				/* mode_5: rising_edge 2 boundary (not full bound), falling edge 1 full boundary */
				uBD_width = pBdNext->Bound_width;
				cycle_cnt = 2 * (pBdNext->Bound_End - pBdPrev->Bound_End);
				uBD_mid_next = (pBdNext->Bound_Start + pBdNext->Bound_End) / 2;
				uDlySel_R = uBD_mid_next;
				uMgLost_R = 0;
				if (pBdPrev->Bound_End >= uBD_width / 2) {
					uDlySel_F = pBdPrev->Bound_End - uBD_width / 2;
					uMgLost_F = 0;
				} else {
					uDlySel_F = 0;
					uMgLost_F = uBD_width / 2 - pBdPrev->Bound_End;
				}
			} else {
				/* for falling edge there must be one full boundary between two bounary_mid at rising */
				return -1;
			}
		} else if (Bound_Cnt_R == 1) {
			if (Bound_Cnt_F > 1) {
				/* when rising_edge have only one boundary (not full bound),
				falling edge should not more than 1Bound exist */
				return -1;
			} else if (Bound_Cnt_F == 1) {
				/* mode_6: rising edge only 1 boundary (not full Bound)
				   & falling edge have only 1 bound too */
				pBdPrev = &(pBdInfo_R->bd_info[0]);
				pBdNext = &(pBdInfo_F->bd_info[0]);
				if (pBdNext->is_fullbound) {
					uBD_width = pBdNext->Bound_width;
				} else {
					if (pBdNext->Bound_width > pBdPrev->Bound_width)
						uBD_width = (pBdNext->Bound_width + 1);
					else
						uBD_width = (pBdPrev->Bound_width + 1);

					if (uBD_width < AUTOK_BD_WIDTH_REF)
						uBD_width = AUTOK_BD_WIDTH_REF;
				} /* Boundary width calc done */

				if (pBdPrev->Bound_Start == 0) {
					/* Current Desing Not Allowed */
					if (pBdNext->Bound_Start == 0)
						return -1;

					cycle_cnt =
					    (pBdNext->Bound_Start - pBdPrev->Bound_End +
					     uBD_width) * 2;
				} else if (pBdPrev->Bound_End == 63) {
					/* Current Desing Not Allowed */
					if (pBdNext->Bound_End == 63)
						return -1;

					cycle_cnt =
					    (pBdPrev->Bound_Start - pBdNext->Bound_End +
					     uBD_width) * 2;
				} /* cycle count calc done */

				/* calc optimise delay count */
				if (pBdPrev->Bound_Start == 0) {
					/* falling edge sel */
					if (pBdPrev->Bound_End >= uBD_width / 2) {
						uDlySel_F = pBdPrev->Bound_End - uBD_width / 2;
						uMgLost_F = 0;
					} else {
						uDlySel_F = 0;
						uMgLost_F = uBD_width / 2 - pBdPrev->Bound_End;
					}

					/* rising edge sel */
					if (pBdPrev->Bound_End - uBD_width / 2 + cycle_cnt / 2 > 63) {
						uDlySel_R = 63;
						uMgLost_R =
						    pBdPrev->Bound_End - uBD_width / 2 +
						    cycle_cnt / 2 - 63;
					} else {
						uDlySel_R =
						    pBdPrev->Bound_End - uBD_width / 2 +
						    cycle_cnt / 2;
						uMgLost_R = 0;
					}
				} else if (pBdPrev->Bound_End == 63) {
					/* falling edge sel */
					if (pBdPrev->Bound_Start + uBD_width / 2 < 63) {
						uDlySel_F = pBdPrev->Bound_Start + uBD_width / 2;
						uMgLost_F = 0;
					} else {
						uDlySel_F = 63;
						uMgLost_F =
						    pBdPrev->Bound_Start + uBD_width / 2 - 63;
					}

					/* rising edge sel */
					if (pBdPrev->Bound_Start + uBD_width / 2 - cycle_cnt / 2 < 0) {
						uDlySel_R = 0;
						uMgLost_R =
						    cycle_cnt / 2 - (pBdPrev->Bound_Start +
								     uBD_width / 2);
					} else {
						uDlySel_R =
						    pBdPrev->Bound_Start + uBD_width / 2 -
						    cycle_cnt / 2;
						uMgLost_R = 0;
					}
				} else {
					return -1;
				}
			} else if (Bound_Cnt_F == 0) {
				/* mode_7: rising edge only one bound (not full), falling no boundary */
				cycle_cnt = 128;
				pBdPrev = &(pBdInfo_R->bd_info[0]);
				if (pBdPrev->Bound_Start == 0) {
					uDlySel_F = 0;
					uDlySel_R = 63;
				} else if (pBdPrev->Bound_End == 63) {
					uDlySel_F = 63;
					uDlySel_R = 0;
				} else {
					return -1;
				}
				uMgLost_F = 0xFF;
				uMgLost_R = 0xFF;

				AUTOK_RAWPRINT("[AUTOK]Warning: 1T > %d\r\n", cycle_cnt);
			}
		} else if (Bound_Cnt_R == 0) { /* Rising Edge No Boundary found */
			if (Bound_Cnt_F > 1) {
				/* falling edge not allowed two boundary Exist for this case */
				return -1;
			} else if (Bound_Cnt_F > 0) {
				/* mode_8: falling edge have one Boundary exist */
				pBdPrev = &(pBdInfo_F->bd_info[0]);

				/* this boundary is full bound */
				if (pBdPrev->is_fullbound) {
					uBD_mid_prev =
					    (pBdPrev->Bound_Start + pBdPrev->Bound_End) / 2;

					if (pBdPrev->Bound_End > (64 - pBdPrev->Bound_Start))
						cycle_cnt = 2 * (pBdPrev->Bound_End + 1);
					else
						cycle_cnt = 2 * (64 - pBdPrev->Bound_Start);

					uDlySel_R = uBD_mid_prev;
					uMgLost_R = 0xFF;
					uDlySel_F = (uBD_mid_prev >= 32) ? 0 : 63;
					uMgLost_F = 0xFF;
				} else {
					cycle_cnt = 128;

					uDlySel_R = (pBdPrev->Bound_Start == 0) ? 0 : 63;
					uMgLost_R = 0xFF;
					uDlySel_F = (pBdPrev->Bound_Start == 0) ? 63 : 0;
					uMgLost_F = 0xFF;
				}

				AUTOK_RAWPRINT("[AUTOK]Warning: 1T > %d\r\n", cycle_cnt);
			} else {
				/* falling edge no boundary exist no need tuning */
				cycle_cnt = 128;
				uDlySel_F = 0;
				uMgLost_F = 0xFF;
				uDlySel_R = 0;
				uMgLost_R = 0xFF;
				AUTOK_RAWPRINT("[AUTOK]Warning: 1T > %d\r\n", cycle_cnt);
			}
		} else {
			/* Error if bound_cnt > 3 there must be at least one full boundary exist */
			return -1;
		}
		break;

	default:
		/* warning if boundary count > 4 (from current hw design, this case cannot happen) */
		return -1;
		break;
	}

	/* Select Optimised Sample edge & delay count (the small one) */
	pInfo->cycle_cnt = cycle_cnt;
	if (uDlySel_R <= uDlySel_F) {
		pInfo->opt_edge_sel = 0;
		pInfo->opt_dly_cnt = uDlySel_R;
		pInfo->margin_lost = uMgLost_R;
	} else {
		pInfo->opt_edge_sel = 1;
		pInfo->opt_dly_cnt = uDlySel_F;
		pInfo->margin_lost = uMgLost_F;

	}
	AUTOK_RAWPRINT("[AUTOK]Analysis Result: 1T = %d\r\n", cycle_cnt);
	return ret;
}

static int autok_ds_dly_sel(struct AUTOK_SCAN_RES *pInfo, unsigned int cycle_value, unsigned int *pDlySel, u8 *param)
{
	unsigned int ret = 0;
	int uDlySel = 0;
	unsigned int Bound_Cnt = pInfo->bd_cnt;

	if (pInfo->fbd_cnt > 0) {
		/* no more than 2 boundary exist */
		AUTOK_RAWPRINT("[AUTOK]Error: Scan DS Not allow Full boundary Occurs!\r\n");
		return -1;
	}

	if (Bound_Cnt > 1) {
		/* bound count == 2 */
		uDlySel = (pInfo->bd_info[0].Bound_End + pInfo->bd_info[1].Bound_Start) / 2;
	} else if (Bound_Cnt > 0) {
		/* bound count == 1 */
		if (pInfo->bd_info[0].Bound_Start == 0) {
			if (pInfo->bd_info[0].Bound_End > 31) {
				uDlySel = (pInfo->bd_info[0].Bound_End + 64) / 2;
				AUTOK_RAWPRINT("[AUTOK]Warning: DS Delay not in range 0~31!\r\n");
			} else {
				uDlySel = (pInfo->bd_info[0].Bound_End + 31) / 2;
			}
		} else {
			/* uDlySel = pInfo->bd_info[0].Bound_Start / 2; */
			/* uDlySel = (((pInfo->bd_info[0].Bound_Start * 5000) / cycle_value)
				- (1250 - 400)) * cycle_value / 5000; */
			if ((cycle_value * HS400_ACTUAL_CYCLE) / (4 * HS400_ONE_CYCLE)
				> pInfo->bd_info[0].Bound_Start) {
				param[DAT_RD_D_DLY1] = (cycle_value * HS400_ACTUAL_CYCLE) / (4 * HS400_ONE_CYCLE)
					- pInfo->bd_info[0].Bound_Start;
				param[DAT_RD_D_DLY1_SEL] = 1;
				uDlySel = 0;
			} else
				uDlySel = pInfo->bd_info[0].Bound_Start
				- (cycle_value * HS400_ACTUAL_CYCLE) / (4 * HS400_ONE_CYCLE);
		}
	} else {
		/* bound count == 0 */
		uDlySel = 16;
	}
	*pDlySel = uDlySel;

	return ret;
}

struct mt6797_autok_result {
	unsigned int edge, delay, cycle, margin_lost;
	unsigned int start, width, left, right;
	bool fallback;
};

static void mt6797_autok_window(u64 pass, struct mt6797_autok_result *r)
{
	unsigned int begin = r->delay, end = r->delay;

	while (begin && (pass & BIT_ULL(begin - 1)))
		begin--;
	while (end < 63 && (pass & BIT_ULL(end + 1)))
		end++;
	r->start = begin;
	r->width = end - begin + 1;
	r->left = r->delay - begin;
	r->right = end - r->delay;
}

/* No extrapolated BSP point may override an actually failing measurement. */
static int mt6797_autok_select(u64 rise, u64 fall, struct mt6797_autok_result *r)
{
	struct AUTOK_REF_INFO ref = {};
	u64 maps[2] = { rise, fall };
	unsigned int edge, i, begin, width, best = 0;

	memset(r, 0, sizeof(*r));
	if (!autok_check_scan_res64(~rise, &ref.scan_info[0]) &&
	    !autok_check_scan_res64(~fall, &ref.scan_info[1]) &&
	    !autok_pad_dly_sel(&ref) && ref.opt_edge_sel < 2 &&
	    ref.opt_dly_cnt < 64 && (maps[ref.opt_edge_sel] & BIT_ULL(ref.opt_dly_cnt))) {
		r->edge = ref.opt_edge_sel;
		r->delay = ref.opt_dly_cnt;
		r->cycle = ref.cycle_cnt;
		r->margin_lost = ref.margin_lost;
	} else {
		/* Fragmented/noisy maps may exceed the BSP's four boundaries. */
		r->fallback = true;
		for (edge = 0; edge < 2; edge++) {
			for (i = 0; i < 64;) {
				if (!(maps[edge] & BIT_ULL(i))) { i++; continue; }
				begin = i;
				while (i < 64 && (maps[edge] & BIT_ULL(i))) i++;
				width = i - begin;
				if (width > best) {
					best = width; r->edge = edge; r->delay = begin + (width - 1) / 2;
				}
			}
		}
		if (!best)
			return -EIO;
	}
	mt6797_autok_window(maps[r->edge], r);
	return 0;
}

static int mt6797_autok_select_ds(u32 pass, unsigned int cycle,
		struct mt6797_autok_result *r, u8 *data_adjust)
{
	struct AUTOK_SCAN_RES scan = {};
	unsigned int delay;
	int ret;
	u8 param[2] = {};

	memset(r, 0, sizeof(*r));
	*data_adjust = 0;
	if (!autok_check_scan_res64(~(u64)pass, &scan) &&
	    !autok_ds_dly_sel(&scan, cycle ? cycle : 64, &delay, param) &&
	    delay < 32 && (pass & BIT(delay)) && param[DAT_RD_D_DLY1] < 32) {
		r->delay = delay;
		if (param[DAT_RD_D_DLY1_SEL])
			*data_adjust = param[DAT_RD_D_DLY1];
		mt6797_autok_window(pass, r);
		return 0;
	}
	ret = mt6797_autok_select(pass, 0, r);
	r->fallback = true;
	return ret;
}

#undef AUTOK_RAWPRINT
#undef AUTOK_DBGPRINT
#endif
