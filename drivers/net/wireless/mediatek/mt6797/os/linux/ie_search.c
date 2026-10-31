/*
 * IE search helpers for the cfg80211 and P2P association paths.
 *
 * Extracted verbatim from the gen3 glue (formerly in the wext layer);
 * these are information-element parsers, independent of any userspace
 * ioctl interface.
 */

#include "precomp.h"
#include "ie_search.h"

/*----------------------------------------------------------------------------*/

BOOLEAN
ieSrchDesiredWPAIE(IN PUINT_8 pucIEStart,
		     IN INT_32 i4TotalIeLen, IN UINT_8 ucDesiredElemId, OUT PUINT_8 *ppucDesiredIE)
{
	INT_32 i4InfoElemLen;

	ASSERT(pucIEStart);
	ASSERT(ppucDesiredIE);

	while (i4TotalIeLen >= 2) {
		i4InfoElemLen = (INT_32) pucIEStart[1] + 2;

		if (pucIEStart[0] == ucDesiredElemId && i4InfoElemLen <= i4TotalIeLen) {
			if (ucDesiredElemId != 0xDD) {
				/* Non 0xDD, OK! */
				*ppucDesiredIE = &pucIEStart[0];
				return TRUE;
			}
			/* EID == 0xDD, check WPA IE */
			if (pucIEStart[1] >= 4) {
				if (memcmp(&pucIEStart[2], "\x00\x50\xf2\x01", 4) == 0) {
					*ppucDesiredIE = &pucIEStart[0];
					return TRUE;
				}
			}	/* check WPA IE length */
		}

		/* check desired EID */
		/* Select next information element. */
		i4TotalIeLen -= i4InfoElemLen;
		pucIEStart += i4InfoElemLen;
	}

	return FALSE;
}				/* ieSrchDesiredWPAIE */

#if CFG_SUPPORT_WAPI
/*----------------------------------------------------------------------------*/
/*!
* \brief Find the desired WAPI Information Element .
*
* \param[in] pucIEStart IE starting address.
* \param[in] i4TotalIeLen Total length of all the IE.
* \param[out] ppucDesiredIE Pointer to the desired IE.
*
* \retval TRUE Find the desired IE.
* \retval FALSE Desired IE not found.
*
* \note
*/
/*----------------------------------------------------------------------------*/
BOOLEAN ieSrchDesiredWAPIIE(IN PUINT_8 pucIEStart, IN INT_32 i4TotalIeLen, OUT PUINT_8 *ppucDesiredIE)
{
	INT_32 i4InfoElemLen;

	ASSERT(pucIEStart);
	ASSERT(ppucDesiredIE);

	while (i4TotalIeLen >= 2) {
		i4InfoElemLen = (INT_32) pucIEStart[1] + 2;

		if (pucIEStart[0] == ELEM_ID_WAPI && i4InfoElemLen <= i4TotalIeLen) {
			*ppucDesiredIE = &pucIEStart[0];
			return TRUE;
		}

		/* check desired EID */
		/* Select next information element. */
		i4TotalIeLen -= i4InfoElemLen;
		pucIEStart += i4InfoElemLen;
	}

	return FALSE;
}				/* ieSrchDesiredWAPIIE */
#endif

#if CFG_SUPPORT_PASSPOINT
/*----------------------------------------------------------------------------*/
/*!
* \brief Check if exist the desired HS2.0 Information Element according to desiredElemID.
*
* \param[in] pucIEStart IE starting address.
* \param[in] i4TotalIeLen Total length of all the IE.
* \param[in] ucDesiredElemId Desired element ID.
* \param[out] ppucDesiredIE Pointer to the desired IE.
*
* \retval TRUE Find the desired IE.
* \retval FALSE Desired IE not found.
*
* \note
*/
/*----------------------------------------------------------------------------*/
BOOLEAN wextIsDesiredHS20IE(IN PUINT_8 pucCurIE, IN INT_32 i4TotalIeLen)
{
	INT_32 i4InfoElemLen;

	ASSERT(pucCurIE);

	i4InfoElemLen = (INT_32) pucCurIE[1] + 2;

	if (pucCurIE[0] == ELEM_ID_VENDOR && i4InfoElemLen <= i4TotalIeLen) {
		if (pucCurIE[1] >= ELEM_MIN_LEN_HS20_INDICATION) {
			if (memcmp(&pucCurIE[2], "\x50\x6f\x9a\x10", 4) == 0)
				return TRUE;
		}
	}
	/* check desired EID */
	return FALSE;
}				/* wextIsDesiredHS20IE */

/*----------------------------------------------------------------------------*/
/*!
* \brief Check if exist the desired interworking Information Element according to desiredElemID.
*
* \param[in] pucIEStart IE starting address.
* \param[in] i4TotalIeLen Total length of all the IE.
* \param[in] ucDesiredElemId Desired element ID.
* \param[out] ppucDesiredIE Pointer to the desired IE.
*
* \retval TRUE Find the desired IE.
* \retval FALSE Desired IE not found.
*
* \note
*/
/*----------------------------------------------------------------------------*/
BOOLEAN wextIsDesiredInterworkingIE(IN PUINT_8 pucCurIE, IN INT_32 i4TotalIeLen)
{
	INT_32 i4InfoElemLen;

	ASSERT(pucCurIE);

	i4InfoElemLen = (INT_32) pucCurIE[1] + 2;

	if (pucCurIE[0] == ELEM_ID_INTERWORKING && i4InfoElemLen <= i4TotalIeLen) {
		switch (pucCurIE[1]) {
		case IW_IE_LENGTH_ANO:
		case IW_IE_LENGTH_ANO_HESSID:
		case IW_IE_LENGTH_ANO_VENUE:
		case IW_IE_LENGTH_ANO_VENUE_HESSID:
			return TRUE;
		default:
			break;
		}

	}
	/* check desired EID */
	return FALSE;
}				/* wextIsDesiredInterworkingIE */

/*----------------------------------------------------------------------------*/
/*!
* \brief Check if exist the desired Adv Protocol Information Element according to desiredElemID.
*
* \param[in] pucIEStart IE starting address.
* \param[in] i4TotalIeLen Total length of all the IE.
* \param[in] ucDesiredElemId Desired element ID.
* \param[out] ppucDesiredIE Pointer to the desired IE.
*
* \retval TRUE Find the desired IE.
* \retval FALSE Desired IE not found.
*
* \note
*/
/*----------------------------------------------------------------------------*/
BOOLEAN wextIsDesiredAdvProtocolIE(IN PUINT_8 pucCurIE, IN INT_32 i4TotalIeLen)
{
	INT_32 i4InfoElemLen;

	ASSERT(pucCurIE);

	i4InfoElemLen = (INT_32) pucCurIE[1] + 2;

	if (pucCurIE[0] == ELEM_ID_ADVERTISEMENT_PROTOCOL && i4InfoElemLen <= i4TotalIeLen)
		return TRUE;
	/* check desired EID */
	return FALSE;
}				/* wextIsDesiredAdvProtocolIE */

/*----------------------------------------------------------------------------*/
/*!
* \brief Check if exist the desired Roaming Consortium Information Element according to desiredElemID.
*
* \param[in] pucIEStart IE starting address.
* \param[in] i4TotalIeLen Total length of all the IE.
* \param[in] ucDesiredElemId Desired element ID.
* \param[out] ppucDesiredIE Pointer to the desired IE.
*
* \retval TRUE Find the desired IE.
* \retval FALSE Desired IE not found.
*
* \note
*/
/*----------------------------------------------------------------------------*/
BOOLEAN ieIsDesiredRoamingConsortiumIE(IN PUINT_8 pucCurIE, IN INT_32 i4TotalIeLen)
{
	INT_32 i4InfoElemLen;

	ASSERT(pucCurIE);

	i4InfoElemLen = (INT_32) pucCurIE[1] + 2;

	if (pucCurIE[0] == ELEM_ID_ROAMING_CONSORTIUM && i4InfoElemLen <= i4TotalIeLen)
		return TRUE;
	/* check desired EID */
	return FALSE;
}				/* ieIsDesiredRoamingConsortiumIE */

/*----------------------------------------------------------------------------*/
/*!
* \brief Find the desired HS2.0 Information Element according to desiredElemID.
*
* \param[in] pucIEStart IE starting address.
* \param[in] i4TotalIeLen Total length of all the IE.
* \param[in] ucDesiredElemId Desired element ID.
* \param[out] ppucDesiredIE Pointer to the desired IE.
*
* \retval TRUE Find the desired IE.
* \retval FALSE Desired IE not found.
*
* \note
*/
/*----------------------------------------------------------------------------*/
BOOLEAN ieSrchDesiredHS20IE(IN PUINT_8 pucIEStart, IN INT_32 i4TotalIeLen, OUT PUINT_8 *ppucDesiredIE)
{
	INT_32 i4InfoElemLen;

	ASSERT(pucIEStart);
	ASSERT(ppucDesiredIE);

	while (i4TotalIeLen >= 2) {
		i4InfoElemLen = (INT_32) pucIEStart[1] + 2;

		if (pucIEStart[0] == ELEM_ID_VENDOR && i4InfoElemLen <= i4TotalIeLen) {
			if (pucIEStart[1] >= ELEM_MIN_LEN_HS20_INDICATION) {
				if (memcmp(&pucIEStart[2], "\x50\x6f\x9a\x10", 4) == 0) {
					*ppucDesiredIE = &pucIEStart[0];
					return TRUE;
				}
			}
		}

		/* check desired EID */
		/* Select next information element. */
		i4TotalIeLen -= i4InfoElemLen;
		pucIEStart += i4InfoElemLen;
	}

	return FALSE;
}				/* ieSrchDesiredHS20IE */

/*----------------------------------------------------------------------------*/
/*!
* \brief Find the desired interworking Information Element according to desiredElemID.
*
* \param[in] pucIEStart IE starting address.
* \param[in] i4TotalIeLen Total length of all the IE.
* \param[in] ucDesiredElemId Desired element ID.
* \param[out] ppucDesiredIE Pointer to the desired IE.
*
* \retval TRUE Find the desired IE.
* \retval FALSE Desired IE not found.
*
* \note
*/
/*----------------------------------------------------------------------------*/
BOOLEAN ieSrchDesiredInterworkingIE(IN PUINT_8 pucIEStart, IN INT_32 i4TotalIeLen, OUT PUINT_8 *ppucDesiredIE)
{
	INT_32 i4InfoElemLen;

	ASSERT(pucIEStart);
	ASSERT(ppucDesiredIE);

	while (i4TotalIeLen >= 2) {
		i4InfoElemLen = (INT_32) pucIEStart[1] + 2;

		if (pucIEStart[0] == ELEM_ID_INTERWORKING && i4InfoElemLen <= i4TotalIeLen) {
			*ppucDesiredIE = &pucIEStart[0];
			return TRUE;
		}

		/* check desired EID */
		/* Select next information element. */
		i4TotalIeLen -= i4InfoElemLen;
		pucIEStart += i4InfoElemLen;
	}

	return FALSE;
}				/* ieSrchDesiredInterworkingIE */

/*----------------------------------------------------------------------------*/
/*!
* \brief Find the desired Adv Protocol Information Element according to desiredElemID.
*
* \param[in] pucIEStart IE starting address.
* \param[in] i4TotalIeLen Total length of all the IE.
* \param[in] ucDesiredElemId Desired element ID.
* \param[out] ppucDesiredIE Pointer to the desired IE.
*
* \retval TRUE Find the desired IE.
* \retval FALSE Desired IE not found.
*
* \note
*/
/*----------------------------------------------------------------------------*/
BOOLEAN ieSrchDesiredAdvProtocolIE(IN PUINT_8 pucIEStart, IN INT_32 i4TotalIeLen, OUT PUINT_8 *ppucDesiredIE)
{
	INT_32 i4InfoElemLen;

	ASSERT(pucIEStart);
	ASSERT(ppucDesiredIE);

	while (i4TotalIeLen >= 2) {
		i4InfoElemLen = (INT_32) pucIEStart[1] + 2;

		if (pucIEStart[0] == ELEM_ID_ADVERTISEMENT_PROTOCOL && i4InfoElemLen <= i4TotalIeLen) {
			*ppucDesiredIE = &pucIEStart[0];
			return TRUE;
		}

		/* check desired EID */
		/* Select next information element. */
		i4TotalIeLen -= i4InfoElemLen;
		pucIEStart += i4InfoElemLen;
	}

	return FALSE;
}				/* ieSrchDesiredAdvProtocolIE */

/*----------------------------------------------------------------------------*/
/*!
* \brief Find the desired Roaming Consortium Information Element according to desiredElemID.
*
* \param[in] pucIEStart IE starting address.
* \param[in] i4TotalIeLen Total length of all the IE.
* \param[in] ucDesiredElemId Desired element ID.
* \param[out] ppucDesiredIE Pointer to the desired IE.
*
* \retval TRUE Find the desired IE.
* \retval FALSE Desired IE not found.
*
* \note
*/
/*----------------------------------------------------------------------------*/
BOOLEAN ieSrchDesiredRoamingConsortiumIE(IN PUINT_8 pucIEStart, IN INT_32 i4TotalIeLen, OUT PUINT_8 *ppucDesiredIE)
{
	INT_32 i4InfoElemLen;

	ASSERT(pucIEStart);
	ASSERT(ppucDesiredIE);

	while (i4TotalIeLen >= 2) {
		i4InfoElemLen = (INT_32) pucIEStart[1] + 2;

		if (pucIEStart[0] == ELEM_ID_ROAMING_CONSORTIUM && i4InfoElemLen <= i4TotalIeLen) {
			*ppucDesiredIE = &pucIEStart[0];
			return TRUE;
		}

		/* check desired EID */
		/* Select next information element. */
		i4TotalIeLen -= i4InfoElemLen;
		pucIEStart += i4InfoElemLen;
	}

	return FALSE;
}				/* ieSrchDesiredRoamingConsortiumIE */

#endif /* CFG_SUPPORT_PASSPOINT */

#if CFG_SUPPORT_WPS
/*----------------------------------------------------------------------------*/
/*!
* \brief Find the desired WPS Information Element according to desiredElemID.
*
* \param[in] pucIEStart IE starting address.
* \param[in] i4TotalIeLen Total length of all the IE.
* \param[in] ucDesiredElemId Desired element ID.
* \param[out] ppucDesiredIE Pointer to the desired IE.
*
* \retval TRUE Find the desired IE.
* \retval FALSE Desired IE not found.
*
* \note
*/
/*----------------------------------------------------------------------------*/
BOOLEAN
ieSrchDesiredWPSIE(IN PUINT_8 pucIEStart,
		     IN INT_32 i4TotalIeLen, IN UINT_8 ucDesiredElemId, OUT PUINT_8 *ppucDesiredIE)
{
	INT_32 i4InfoElemLen;

	ASSERT(pucIEStart);
	ASSERT(ppucDesiredIE);

	while (i4TotalIeLen >= 2) {
		i4InfoElemLen = (INT_32) pucIEStart[1] + 2;

		if (pucIEStart[0] == ucDesiredElemId && i4InfoElemLen <= i4TotalIeLen) {
			if (ucDesiredElemId != 0xDD) {
				/* Non 0xDD, OK! */
				*ppucDesiredIE = &pucIEStart[0];
				return TRUE;
			}

			/* EID == 0xDD, check WPS IE */
			if (pucIEStart[1] >= 4) {
				if (memcmp(&pucIEStart[2], "\x00\x50\xf2\x04", 4) == 0) {
					*ppucDesiredIE = &pucIEStart[0];
					return TRUE;
				}
			}	/* check WPS IE length */
		}

		/* check desired EID */
		/* Select next information element. */
		i4TotalIeLen -= i4InfoElemLen;
		pucIEStart += i4InfoElemLen;
	}

	return FALSE;
}				/* ieSrchDesiredWPSIE */
#endif
