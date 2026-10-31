/*
 * GSCAN data types of the firmware command/event/OID ABI.
 *
 * These describe the payloads of the GSCAN/PSCN firmware commands and
 * events used by nic_cmd_event.h, wlan_oid and the scan path; the
 * Android vendor-command layer that shared them is gone.  Values and
 * layout are the firmware ABI, unchanged.
 */
#ifndef _WLAN_GSCAN_TYPES_H
#define _WLAN_GSCAN_TYPES_H

typedef enum {
	WIFI_BAND_UNSPECIFIED,
	WIFI_BAND_BG = 1,	    /* 2.4 GHz */
	WIFI_BAND_A = 2,	    /* 5 GHz without DFS */
	WIFI_BAND_A_DFS = 4,	    /* 5 GHz DFS only */
	WIFI_BAND_A_WITH_DFS = 6,   /* 5 GHz with DFS */
	WIFI_BAND_ABG = 3,	    /* 2.4 GHz + 5 GHz; no DFS */
	WIFI_BAND_ABG_WITH_DFS = 7, /* 2.4 GHz + 5 GHz with DFS */
} WIFI_BAND;

typedef enum {
	WIFI_SCAN_BUFFER_FULL,
	WIFI_SCAN_COMPLETE,
} WIFI_SCAN_EVENT;

#define GSCAN_MAX_REPORT_THRESHOLD   1024000
#define GSCAN_MAX_CHANNELS                 8
#define GSCAN_MAX_BUCKETS                  8
#define MAX_HOTLIST_APS                   16
#define MAX_SIGNIFICANT_CHANGE_APS        16
#define PSCAN_VERSION                      1

typedef UINT_64 wifi_timestamp;	/* In microseconds (us) */
typedef UINT_64 wifi_timespan;	/* In nanoseconds  (ns) */

typedef UINT_8 mac_addr[6];
typedef UINT_32 wifi_channel;	/* Indicates channel frequency in MHz */
typedef INT_32 wifi_rssi;

typedef struct _PARAM_WIFI_GSCAN_GET_RESULT_PARAMS {
	UINT_32 get_num;
	UINT_8 flush;
} PARAM_WIFI_GSCAN_GET_RESULT_PARAMS, *P_PARAM_WIFI_GSCAN_GET_RESULT_PARAMS;

typedef struct _PARAM_WIFI_GSCAN_ACTION_CMD_PARAMS {
	UINT_8 ucPscanAct;
	UINT_8 aucReserved[3];
} PARAM_WIFI_GSCAN_ACTION_CMD_PARAMS, *P_PARAM_WIFI_GSCAN_ACTION_CMD_PARAMS;

typedef struct _PARAM_WIFI_GSCAN_CAPABILITIES_STRUCT_T {
	UINT_32 max_scan_cache_size;	/*total space allocated for scan (in bytes) */
	UINT_32 max_scan_buckets;	/*maximum number of channel buckets */
	UINT_32 max_ap_cache_per_scan;	/*maximum number of APs that can be stored per scan */
	UINT_32 max_rssi_sample_size;	/*number of RSSI samples used for averaging RSSI */
	UINT_32 max_scan_reporting_threshold;	/*max possible report_threshold as described in wifi_scan_cmd_params */
	UINT_32 max_hotlist_aps;	/*maximum number of entries for hotlist APs */
	UINT_32 max_significant_wifi_change_aps;	/*maximum number of entries for significant wifi change APs */
	UINT_32 max_bssid_history_entries;	/*number of BSSID/RSSI entries that device can hold */
} PARAM_WIFI_GSCAN_CAPABILITIES_STRUCT_T, *P_PARAM_WIFI_GSCAN_CAPABILITIES_STRUCT_T;

typedef struct _PARAM_WIFI_GSCAN_CHANNEL_SPEC {
	UINT_32 channel;
	UINT_32 dwellTimeMs;
	UINT_32 passive;
	/* Add channel class */
} PARAM_WIFI_GSCAN_CHANNEL_SPEC, *P_PARAM_WIFI_GSCAN_CHANNEL_SPEC;

typedef struct _PARAM_WIFI_GSCAN_BUCKET_SPEC {
	UINT_32 bucket;		/* bucket index, 0 based */
	WIFI_BAND band;		/*when UNSPECIFIED, use channel lis */
	UINT_32 period;	/* desired period, in millisecond; if this is too  low, the firmware should choose to generate
				   results as fast as it can instead of failing the command */
	/* report_events semantics -
	 *  0 => report only when scan history is % full
	 *  1 => same as 0 + report a scan completion event after scanning this bucket
	 *  2 => same as 1 + forward scan results (beacons/probe responses + IEs) in real time to HAL
	 *  3 => same as 2 + forward scan results (beacons/probe responses + IEs) in real time to
	 supplicant as well (optional) . */
	UINT_8 report_events;

	UINT_32 num_channels;
	PARAM_WIFI_GSCAN_CHANNEL_SPEC channels[GSCAN_MAX_CHANNELS];
	/* channels to scan; these may include DFS channels */
} PARAM_WIFI_GSCAN_BUCKET_SPEC, *P_PARAM_WIFI_GSCAN_BUCKET_SPEC;

typedef struct _PARAM_WIFI_GSCAN_CMD_PARAMS {
	UINT_32 base_period;	/* base timer period in ms */
	UINT_32 max_ap_per_scan;	/* number of APs to store in each scan in the
								BSSID/RSSI history buffer (keep the highest RSSI APs) */
	UINT_32 report_threshold;	/* in %, when scan buffer is this much full, wake up AP */
	UINT_32 num_scans;
	UINT_32 num_buckets;
	PARAM_WIFI_GSCAN_BUCKET_SPEC buckets[GSCAN_MAX_BUCKETS];
} PARAM_WIFI_GSCAN_CMD_PARAMS, *P_PARAM_WIFI_GSCAN_CMD_PARAMS;

typedef struct _PARAM_WIFI_GSCAN_RESULT {
	wifi_timestamp ts;	/* time since boot (in microsecond) when the result was */
	/* retrieved */
	UINT_8 ssid[32 + 1];	/* null terminated */
	mac_addr bssid;
	wifi_channel channel;	/* channel frequency in MHz */
	wifi_rssi rssi;		/* in db */
	wifi_timespan rtt;	/* in nanoseconds */
	wifi_timespan rtt_sd;	/* standard deviation in rtt */
	UINT_16 beacon_period;	/* period advertised in the beacon */
	UINT_16 capability;	/* capabilities advertised in the beacon */
	UINT_32 ie_length;	/* size of the ie_data blob */
	UINT_8 ie_data[1];	/* blob of all the information elements found in the */
	/* beacon; this data should be a packed list of */
	/* wifi_information_element objects, one after the other. */
	/* other fields */
} PARAM_WIFI_GSCAN_RESULT, *P_PARAM_WIFI_GSCAN_RESULT;

	   /* Significant wifi change */
#if 0
	typedef struct _PARAM_WIFI_CHANGE_RESULT {
		mac_addr bssid;	/* BSSID */
		wifi_channel channel;	/* channel frequency in MHz */
		UINT_32 num_rssi;	/* number of rssi samples */
		wifi_rssi rssi[8];	/* RSSI history in db */
	} PARAM_WIFI_CHANGE_RESULT, *P_PARAM_WIFI_CHANGE_RESULT;
#endif

typedef struct _PARAM_WIFI_CHANGE_RESULT {
	UINT_16 flags;
	UINT_16 channel;
	mac_addr bssid;	/* BSSID */
	INT_8 rssi[8];	/* RSSI history in db */
} PARAM_WIFI_CHANGE_RESULT, *P_PARAM_WIFI_CHANGE_RESULT;

typedef struct _PARAM_AP_THRESHOLD {
	mac_addr bssid;	/* AP BSSID */
	wifi_rssi low;	/* low threshold */
	wifi_rssi high;	/* high threshold */
	wifi_channel channel;	/* channel hint */
} PARAM_AP_THRESHOLD, *P_PARAM_AP_THRESHOLD;

typedef struct _PARAM_WIFI_BSSID_HOTLIST {
	UINT_32 lost_ap_sample_size;
	UINT_32 num_ap;	/* number of hotlist APs */
	PARAM_AP_THRESHOLD ap[MAX_HOTLIST_APS];	/* hotlist APs */
} PARAM_WIFI_BSSID_HOTLIST, *P_PARAM_WIFI_BSSID_HOTLIST;

typedef struct _PARAM_WIFI_SIGNIFICANT_CHANGE {
	UINT_16 rssi_sample_size;	/* number of samples for averaging RSSI */
	UINT_16 lost_ap_sample_size;	/* number of samples to confirm AP loss */
	UINT_16 min_breaching;	/* number of APs breaching threshold */
	UINT_16 num_ap;	/* max 64 */
	PARAM_AP_THRESHOLD ap[MAX_SIGNIFICANT_CHANGE_APS];
} PARAM_WIFI_SIGNIFICANT_CHANGE, *P_PARAM_WIFI_SIGNIFICANT_CHANGE;
#endif /* _WLAN_GSCAN_TYPES_H */
