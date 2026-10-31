/*
 * IE search helpers shared by the cfg80211 and P2P paths.
 */
#ifndef _IE_SEARCH_H
#define _IE_SEARCH_H

BOOLEAN
ieSrchDesiredWPAIE(IN PUINT_8 pucIEStart,
		   IN INT_32 i4TotalIeLen, IN UINT_8 ucDesiredElemId, OUT PUINT_8 *ppucDesiredIE);
#if CFG_SUPPORT_WAPI
BOOLEAN ieSrchDesiredWAPIIE(IN PUINT_8 pucIEStart, IN INT_32 i4TotalIeLen, OUT PUINT_8 *ppucDesiredIE);
#endif
#if CFG_SUPPORT_PASSPOINT
BOOLEAN ieSrchDesiredHS20IE(IN PUINT_8 pucIEStart, IN INT_32 i4TotalIeLen, OUT PUINT_8 *ppucDesiredIE);
BOOLEAN ieSrchDesiredInterworkingIE(IN PUINT_8 pucIEStart, IN INT_32 i4TotalIeLen, OUT PUINT_8 *ppucDesiredIE);
BOOLEAN ieSrchDesiredAdvProtocolIE(IN PUINT_8 pucIEStart, IN INT_32 i4TotalIeLen, OUT PUINT_8 *ppucDesiredIE);
BOOLEAN ieSrchDesiredRoamingConsortiumIE(IN PUINT_8 pucIEStart, IN INT_32 i4TotalIeLen, OUT PUINT_8 *ppucDesiredIE);
BOOLEAN ieIsDesiredRoamingConsortiumIE(IN PUINT_8 pucIEStart, IN INT_32 i4TotalIeLen);
#endif
#if CFG_SUPPORT_WPS
BOOLEAN
ieSrchDesiredWPSIE(IN PUINT_8 pucIEStart,
		   IN INT_32 i4TotalIeLen, IN UINT_8 ucDesiredElemId, OUT PUINT_8 *ppucDesiredIE);
#endif

#endif /* _IE_SEARCH_H */
