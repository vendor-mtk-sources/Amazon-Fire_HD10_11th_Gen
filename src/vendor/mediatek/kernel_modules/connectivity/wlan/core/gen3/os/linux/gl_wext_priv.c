/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

/*
** Id: //Department/DaVinci/BRANCHES/MT6620_WIFI_DRIVER_V2_3/os/linux/gl_wext_priv.c#8
*/

/*
 * ! \file gl_wext_priv.c
 * \brief This file includes private ioctl support.
 */

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/
#include "precomp.h"
#include "gl_os.h"
#include "gl_wext_priv.h"
#if CFG_SUPPORT_WAPI
#include "gl_sec.h"
#endif
#if CFG_ENABLE_WIFI_DIRECT
#include "gl_p2p_os.h"
#endif

#if CFG_SUPPORT_QA_TOOL
#include "gl_ate_agent.h"
#include "gl_qa_agent.h"
/* extern UINT_16 g_u2DumpIndex; */
#endif

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/
#define NUM_SUPPORTED_OIDS	(sizeof(arWlanOidReqTable) / sizeof(WLAN_REQ_ENTRY))
#define	CMD_OID_BUF_LENGTH	4096

#define CMD_GET_WIFI_TYPE	"GET_WIFI_TYPE"
#ifdef ENABLED_IN_ENGUSERDEBUG
enum UT_TRIGGER_CHIP_RESET trChipReset = TRIGGER_RESET_START;
#endif
#if CFG_SUPPORT_FW_ACTIVE_TIME_STATISTICS
#define CMD_FW_ACTIVE_STATISTICS   "fw_active_statistics"
extern struct CMD_FW_ACTIVE_TIME_STATISTICS g_FwActiveTime;
extern UINT32 g_FwActiveTimeStatus;
#endif

#define CMD_WIFI_ON_TIME_STATISTICS "wifi_on_time_statistics"

extern struct WIFI_ON_TIME_STATISTICS wifiOnTimeStatistics;

extern WAKEUP_STATISTIC g_arWakeupStatistic[WAKEUP_TYPE_NUM];
extern UINT_32 g_wake_event_count[EVENT_ID_END];

extern void updateWifiOnTimeStatistics(void);
/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

static int
priv_get_ndis(IN struct net_device *prNetDev, IN NDIS_TRANSPORT_STRUCT * prNdisReq, OUT PUINT_32 pu4OutputLen);

static int
priv_set_ndis(IN struct net_device *prNetDev, IN NDIS_TRANSPORT_STRUCT * prNdisReq, OUT PUINT_32 pu4OutputLen);

static void
priv_driver_get_chip_config_16(PUINT_8 pucStartAddr,
			       UINT_32 u4Length, UINT_32 u4Line, int i4TotalLen, INT_32 i4BytesWritten,
			       char *pcCommand);

static void
priv_driver_get_chip_config_4(PUINT_32 pu4StartAddr,
			      UINT_32 u4Length, UINT_32 u4Line, int i4TotalLen, INT_32 i4BytesWritten, char *pcCommand);

#if 0				/* CFG_SUPPORT_WPS */
static int
priv_set_appie(IN struct net_device *prNetDev,
	       IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, OUT char *pcExtra);

static int
priv_set_filter(IN struct net_device *prNetDev,
		IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, OUT char *pcExtra);
#endif /* CFG_SUPPORT_WPS */

static BOOLEAN reqSearchSupportedOidEntry(IN UINT_32 rOid, OUT P_WLAN_REQ_ENTRY * ppWlanReqEntry);

#if 0
static WLAN_STATUS
reqExtQueryConfiguration(IN P_GLUE_INFO_T prGlueInfo,
			 OUT PVOID pvQueryBuffer, IN UINT_32 u4QueryBufferLen, OUT PUINT_32 pu4QueryInfoLen);

static WLAN_STATUS
reqExtSetConfiguration(IN P_GLUE_INFO_T prGlueInfo,
		       IN PVOID pvSetBuffer, IN UINT_32 u4SetBufferLen, OUT PUINT_32 pu4SetInfoLen);
#endif

static WLAN_STATUS
reqExtSetAcpiDevicePowerState(IN P_GLUE_INFO_T prGlueInfo,
			      IN PVOID pvSetBuffer, IN UINT_32 u4SetBufferLen, OUT PUINT_32 pu4SetInfoLen);

/*******************************************************************************
*                       P R I V A T E   D A T A
********************************************************************************
*/
static UINT_8 aucOidBuf[CMD_OID_BUF_LENGTH] = { 0 };

/* OID processing table */
/*
 * Order is important here because the OIDs should be in order of
 * increasing value for binary searching.
 */
static WLAN_REQ_ENTRY arWlanOidReqTable[] = {
	/*
	 * {(NDIS_OID)rOid,
	 * (PUINT_8)pucOidName,
	 * fgQryBufLenChecking, fgSetBufLenChecking, fgIsHandleInGlueLayerOnly, u4InfoBufLen,
	 * pfOidQueryHandler,
	 * pfOidSetHandler}
	 */
	/* General Operational Characteristics */

	/* Ethernet Operational Characteristics */
	{OID_802_3_CURRENT_ADDRESS,
	 DISP_STRING("OID_802_3_CURRENT_ADDRESS"),
	 TRUE, TRUE, ENUM_OID_DRIVER_CORE, 6,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidQueryCurrentAddr,
	 NULL},

	/* OID_802_3_MULTICAST_LIST */
	/* OID_802_3_MAXIMUM_LIST_SIZE */
	/* Ethernet Statistics */

	/* NDIS 802.11 Wireless LAN OIDs */
	{OID_802_11_SUPPORTED_RATES,
	 DISP_STRING("OID_802_11_SUPPORTED_RATES"),
	 TRUE, FALSE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_RATES_EX),
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidQuerySupportedRates,
	 NULL}
	,
	/*
	 * {OID_802_11_CONFIGURATION,
	 * DISP_STRING("OID_802_11_CONFIGURATION"),
	 * TRUE, TRUE, ENUM_OID_GLUE_EXTENSION, sizeof(PARAM_802_11_CONFIG_T),
	 * (PFN_OID_HANDLER_FUNC_REQ)reqExtQueryConfiguration,
	 * (PFN_OID_HANDLER_FUNC_REQ)reqExtSetConfiguration},
	 */
	{OID_PNP_SET_POWER,
	 DISP_STRING("OID_PNP_SET_POWER"),
	 TRUE, FALSE, ENUM_OID_GLUE_EXTENSION, sizeof(PARAM_DEVICE_POWER_STATE),
	 NULL,
	 (PFN_OID_HANDLER_FUNC_REQ) reqExtSetAcpiDevicePowerState}
	,

	/* Custom OIDs */
	{OID_CUSTOM_OID_INTERFACE_VERSION,
	 DISP_STRING("OID_CUSTOM_OID_INTERFACE_VERSION"),
	 TRUE, FALSE, ENUM_OID_DRIVER_CORE, 4,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidQueryOidInterfaceVersion,
	 NULL}
	,

	/*
	 * #if PTA_ENABLED
	 * {OID_CUSTOM_BT_COEXIST_CTRL,
	 * DISP_STRING("OID_CUSTOM_BT_COEXIST_CTRL"),
	 * FALSE, TRUE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_CUSTOM_BT_COEXIST_T),
	 * NULL,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetBtCoexistCtrl},
	 * #endif
	 */

	/*
	 * {OID_CUSTOM_POWER_MANAGEMENT_PROFILE,
	 * DISP_STRING("OID_CUSTOM_POWER_MANAGEMENT_PROFILE"),
	 * FALSE, FALSE, ENUM_OID_DRIVER_CORE, 0,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidQueryPwrMgmtProfParam,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetPwrMgmtProfParam},
	 * {OID_CUSTOM_PATTERN_CONFIG,
	 * DISP_STRING("OID_CUSTOM_PATTERN_CONFIG"),
	 * TRUE, TRUE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_CUSTOM_PATTERN_SEARCH_CONFIG_STRUCT_T),
	 * NULL,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetPatternConfig},
	 * {OID_CUSTOM_BG_SSID_SEARCH_CONFIG,
	 * DISP_STRING("OID_CUSTOM_BG_SSID_SEARCH_CONFIG"),
	 * FALSE, FALSE, ENUM_OID_DRIVER_CORE, 0,
	 * NULL,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetBgSsidParam},
	 * {OID_CUSTOM_VOIP_SETUP,
	 * DISP_STRING("OID_CUSTOM_VOIP_SETUP"),
	 * TRUE, TRUE, ENUM_OID_DRIVER_CORE, 4,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidQueryVoipConnectionStatus,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetVoipConnectionStatus},
	 * {OID_CUSTOM_ADD_TS,
	 * DISP_STRING("OID_CUSTOM_ADD_TS"),
	 * TRUE, TRUE, ENUM_OID_DRIVER_CORE, 4,
	 * NULL,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidAddTS},
	 * {OID_CUSTOM_DEL_TS,
	 * DISP_STRING("OID_CUSTOM_DEL_TS"),
	 * TRUE, TRUE, ENUM_OID_DRIVER_CORE, 4,
	 * NULL,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidDelTS},
	 */

	/*
	 * #if CFG_LP_PATTERN_SEARCH_SLT
	 * {OID_CUSTOM_SLT,
	 * DISP_STRING("OID_CUSTOM_SLT"),
	 * FALSE, FALSE, ENUM_OID_DRIVER_CORE, 0,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidQuerySltResult,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetSltMode},
	 * #endif
	 *
	 * {OID_CUSTOM_ROAMING_EN,
	 * DISP_STRING("OID_CUSTOM_ROAMING_EN"),
	 * TRUE, TRUE, ENUM_OID_DRIVER_CORE, 4,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidQueryRoamingFunction,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetRoamingFunction},
	 * {OID_CUSTOM_WMM_PS_TEST,
	 * DISP_STRING("OID_CUSTOM_WMM_PS_TEST"),
	 * TRUE, TRUE, ENUM_OID_DRIVER_CORE, 4,
	 * NULL,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetWiFiWmmPsTest},
	 * {OID_CUSTOM_COUNTRY_STRING,
	 * DISP_STRING("OID_CUSTOM_COUNTRY_STRING"),
	 * FALSE, FALSE, ENUM_OID_DRIVER_CORE, 0,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidQueryCurrentCountry,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetCurrentCountry},
	 *
	 * #if CFG_SUPPORT_802_11D
	 * {OID_CUSTOM_MULTI_DOMAIN_CAPABILITY,
	 * DISP_STRING("OID_CUSTOM_MULTI_DOMAIN_CAPABILITY"),
	 * FALSE, FALSE, ENUM_OID_DRIVER_CORE, 0,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidQueryMultiDomainCap,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetMultiDomainCap},
	 * #endif
	 *
	 * {OID_CUSTOM_GPIO2_MODE,
	 * DISP_STRING("OID_CUSTOM_GPIO2_MODE"),
	 * FALSE, TRUE, ENUM_OID_DRIVER_CORE, sizeof(ENUM_PARAM_GPIO2_MODE_T),
	 * NULL,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetGPIO2Mode},
	 * {OID_CUSTOM_CONTINUOUS_POLL,
	 * DISP_STRING("OID_CUSTOM_CONTINUOUS_POLL"),
	 * FALSE, TRUE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_CONTINUOUS_POLL_T),
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidQueryContinuousPollInterval,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetContinuousPollProfile},
	 * {OID_CUSTOM_DISABLE_BEACON_DETECTION,
	 * DISP_STRING("OID_CUSTOM_DISABLE_BEACON_DETECTION"),
	 * FALSE, TRUE, ENUM_OID_DRIVER_CORE, 4,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidQueryDisableBeaconDetectionFunc,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetDisableBeaconDetectionFunc},
	 */

	/* WPS */
	/*
	 * {OID_CUSTOM_DISABLE_PRIVACY_CHECK,
	 * DISP_STRING("OID_CUSTOM_DISABLE_PRIVACY_CHECK"),
	 * FALSE, TRUE, ENUM_OID_DRIVER_CORE, 4,
	 * NULL,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetDisablePriavcyCheck},
	 */

	{OID_CUSTOM_MCR_RW,
	 DISP_STRING("OID_CUSTOM_MCR_RW"),
	 TRUE, TRUE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_CUSTOM_MCR_RW_STRUCT_T),
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidQueryMcrRead,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidSetMcrWrite}
	,

	{OID_CUSTOM_EEPROM_RW,
	 DISP_STRING("OID_CUSTOM_EEPROM_RW"),
	 TRUE, TRUE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_CUSTOM_EEPROM_RW_STRUCT_T),
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidQueryEepromRead,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidSetEepromWrite}
	,

	{OID_CUSTOM_SW_CTRL,
	 DISP_STRING("OID_CUSTOM_SW_CTRL"),
	 TRUE, TRUE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_CUSTOM_SW_CTRL_STRUCT_T),
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidQuerySwCtrlRead,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidSetSwCtrlWrite}
	,

	{OID_CUSTOM_MEM_DUMP,
	 DISP_STRING("OID_CUSTOM_MEM_DUMP"),
	 TRUE, TRUE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_CUSTOM_MEM_DUMP_STRUCT_T),
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidQueryMemDump,
	 NULL}
	,

	{OID_CUSTOM_TEST_MODE,
	 DISP_STRING("OID_CUSTOM_TEST_MODE"),
	 FALSE, FALSE, ENUM_OID_DRIVER_CORE, 0,
	 NULL,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidRftestSetTestMode}
	,

	/*
	 * {OID_CUSTOM_TEST_RX_STATUS,
	 * DISP_STRING("OID_CUSTOM_TEST_RX_STATUS"),
	 * FALSE, TRUE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_CUSTOM_RFTEST_RX_STATUS_STRUCT_T),
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidQueryRfTestRxStatus,
	 * NULL},
	 * {OID_CUSTOM_TEST_TX_STATUS,
	 * DISP_STRING("OID_CUSTOM_TEST_TX_STATUS"),
	 * FALSE, TRUE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_CUSTOM_RFTEST_TX_STATUS_STRUCT_T),
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidQueryRfTestTxStatus,
	 * NULL},
	 */
	{OID_CUSTOM_ABORT_TEST_MODE,
	 DISP_STRING("OID_CUSTOM_ABORT_TEST_MODE"),
	 FALSE, FALSE, ENUM_OID_DRIVER_CORE, 0,
	 NULL,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidRftestSetAbortTestMode}
	,
	{OID_CUSTOM_MTK_WIFI_TEST,
	 DISP_STRING("OID_CUSTOM_MTK_WIFI_TEST"),
	 FALSE, FALSE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_MTK_WIFI_TEST_STRUCT_T),
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidRftestQueryAutoTest,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidRftestSetAutoTest}
	,
	{OID_CUSTOM_TEST_ICAP_MODE,
	 DISP_STRING("OID_CUSTOM_TEST_ICAP_MODE"),
	 FALSE, FALSE, ENUM_OID_DRIVER_CORE, 0,
	 NULL,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidRftestSetTestIcapMode}
	,

	/* OID_CUSTOM_EMULATION_VERSION_CONTROL */

	/* BWCS */
#if CFG_SUPPORT_BCM && CFG_SUPPORT_BCM_BWCS
	{OID_CUSTOM_BWCS_CMD,
	 DISP_STRING("OID_CUSTOM_BWCS_CMD"),
	 FALSE, FALSE, ENUM_OID_DRIVER_CORE, sizeof(PTA_IPC_T),
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidQueryBT,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidSetBT}
	,
#endif

	/*
	 * {OID_CUSTOM_SINGLE_ANTENNA
	 * ISP_STRING("OID_CUSTOM_SINGLE_ANTENNA")
	 * ALSE, FALSE, ENUM_OID_DRIVER_CORE, 4
	 * PFN_OID_HANDLER_FUNC_REQ)wlanoidQueryBtSingleAntenna
	 * PFN_OID_HANDLER_FUNC_REQ)wlanoidSetBtSingleAntenna},
	 * {OID_CUSTOM_SET_PTA
	 * ISP_STRING("OID_CUSTOM_SET_PTA
	 * FALSE, FALSE, ENUM_OID_DRIVER_CORE,
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidQueryP
	 * (PFN_OID_HANDLER_FUNC_REQ)wlanoidSetPta},
	 */

	{OID_CUSTOM_MTK_NVRAM_RW,
	 DISP_STRING("OID_CUSTOM_MTK_NVRAM_RW"),
	 TRUE, TRUE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_CUSTOM_NVRAM_RW_STRUCT_T),
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidQueryNvramRead,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidSetNvramWrite}
	,

	{OID_CUSTOM_CFG_SRC_TYPE,
	 DISP_STRING("OID_CUSTOM_CFG_SRC_TYPE"),
	 FALSE, FALSE, ENUM_OID_DRIVER_CORE, sizeof(ENUM_CFG_SRC_TYPE_T),
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidQueryCfgSrcType,
	 NULL}
	,

	{OID_CUSTOM_EEPROM_TYPE,
	 DISP_STRING("OID_CUSTOM_EEPROM_TYPE"),
	 FALSE, FALSE, ENUM_OID_DRIVER_CORE, sizeof(ENUM_EEPROM_TYPE_T),
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidQueryEepromType,
	 NULL}
	,

#if CFG_SUPPORT_WAPI
	{OID_802_11_WAPI_MODE,
	 DISP_STRING("OID_802_11_WAPI_MODE"),
	 FALSE, TRUE, ENUM_OID_DRIVER_CORE, 4,
	 NULL,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidSetWapiMode}
	,
	{OID_802_11_WAPI_ASSOC_INFO,
	 DISP_STRING("OID_802_11_WAPI_ASSOC_INFO"),
	 FALSE, FALSE, ENUM_OID_DRIVER_CORE, 0,
	 NULL,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidSetWapiAssocInfo}
	,
	{OID_802_11_SET_WAPI_KEY,
	 DISP_STRING("OID_802_11_SET_WAPI_KEY"),
	 FALSE, FALSE, ENUM_OID_DRIVER_CORE, sizeof(PARAM_WPI_KEY_T),
	 NULL,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidSetWapiKey}
	,
#endif

#if CFG_SUPPORT_WPS2
	{OID_802_11_WSC_ASSOC_INFO,
	 DISP_STRING("OID_802_11_WSC_ASSOC_INFO"),
	 FALSE, FALSE, ENUM_OID_DRIVER_CORE, 0,
	 NULL,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidSetWSCAssocInfo}
	,
#endif

#if CFG_SUPPORT_LOWLATENCY_MODE
	{OID_CUSTOM_LOWLATENCY_MODE,
	 DISP_STRING("OID_CUSTOM_LOWLATENCY_MODE"),
	 FALSE, FALSE, ENUM_OID_DRIVER_CORE, sizeof(UINT_32),
	 NULL,
	 (PFN_OID_HANDLER_FUNC_REQ) wlanoidSetLowLatencyMode}
	,
#endif

	{OID_IPC_WIFI_LOG_UI,
	DISP_STRING("OID_IPC_WIFI_LOG_UI"),
	FALSE, FALSE, ENUM_OID_DRIVER_CORE, sizeof(struct PARAM_WIFI_LOG_LEVEL_UI),
	(PFN_OID_HANDLER_FUNC_REQ) wlanoidQueryWifiLogLevelSupport,
	NULL}
	,

	{OID_IPC_WIFI_LOG_LEVEL,
	DISP_STRING("OID_IPC_WIFI_LOG_LEVEL"),
	FALSE, FALSE, ENUM_OID_DRIVER_CORE, sizeof(struct PARAM_WIFI_LOG_LEVEL),
	(PFN_OID_HANDLER_FUNC_REQ) wlanoidQueryWifiLogLevel,
	(PFN_OID_HANDLER_FUNC_REQ) wlanoidSetWifiLogLevel}
	,

};

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/

/*----------------------------------------------------------------------------*/
/*!
* \brief Dispatching function for private ioctl region (SIOCIWFIRSTPRIV ~
*   SIOCIWLASTPRIV).
*
* \param[in] prNetDev Net device requested.
* \param[in] prIfReq Pointer to ifreq structure.
* \param[in] i4Cmd Command ID between SIOCIWFIRSTPRIV and SIOCIWLASTPRIV.
*
* \retval 0 for success.
* \retval -EOPNOTSUPP If cmd is not supported.
* \retval -EFAULT For fail.
*
*/
/*----------------------------------------------------------------------------*/
int priv_support_ioctl(IN struct net_device *prNetDev, IN OUT struct ifreq *prIfReq, IN int i4Cmd)
{
	/* prIfReq is verified in the caller function wlanDoIOCTL() */
	struct iwreq *prIwReq = (struct iwreq *)prIfReq;
	struct iw_request_info rIwReqInfo;

	/* prNetDev is verified in the caller function wlanDoIOCTL() */

	/* Prepare the call */
	rIwReqInfo.cmd = (__u16) i4Cmd;
	rIwReqInfo.flags = 0;

	switch (i4Cmd) {
	case IOCTL_SET_INT:
		/* NOTE(Kevin): 1/3 INT Type <= IFNAMSIZ, so we don't need copy_from/to_user() */
		return priv_set_int(prNetDev, &rIwReqInfo, &(prIwReq->u), (char *)&(prIwReq->u));

	case IOCTL_GET_INT:
		/* NOTE(Kevin): 1/3 INT Type <= IFNAMSIZ, so we don't need copy_from/to_user() */
		return priv_get_int(prNetDev, &rIwReqInfo, &(prIwReq->u), (char *)&(prIwReq->u));

	case IOCTL_SET_STRUCT:
	case IOCTL_SET_STRUCT_FOR_EM:
		return priv_set_struct(prNetDev, &rIwReqInfo, &prIwReq->u, (char *)&(prIwReq->u));

	case IOCTL_GET_STRUCT:
		return priv_get_struct(prNetDev, &rIwReqInfo, &prIwReq->u, (char *)&(prIwReq->u));

#if (CFG_SUPPORT_QA_TOOL)
	case IOCTL_QA_TOOL_DAEMON:
		return priv_qa_agent(prNetDev, &rIwReqInfo, &(prIwReq->u), (char *)&(prIwReq->u));
#endif

	default:
		return -EOPNOTSUPP;

	}			/* end of switch */

}				/* priv_support_ioctl */

#if CFG_SUPPORT_BATCH_SCAN

EVENT_BATCH_RESULT_T g_rEventBatchResult[CFG_BATCH_MAX_MSCAN];

UINT_32 batchChannelNum2Freq(UINT_32 u4ChannelNum)
{
	UINT_32 u4ChannelInMHz;

	if (u4ChannelNum >= 1 && u4ChannelNum <= 13)
		u4ChannelInMHz = 2412 + (u4ChannelNum - 1) * 5;
	else if (u4ChannelNum == 14)
		u4ChannelInMHz = 2484;
	else if (u4ChannelNum == 133)
		u4ChannelInMHz = 3665;	/* 802.11y */
	else if (u4ChannelNum == 137)
		u4ChannelInMHz = 3685;	/* 802.11y */
	else if (u4ChannelNum >= 34 && u4ChannelNum <= 165)
		u4ChannelInMHz = 5000 + u4ChannelNum * 5;
	else if (u4ChannelNum >= 183 && u4ChannelNum <= 196)
		u4ChannelInMHz = 4000 + u4ChannelNum * 5;
	else
		u4ChannelInMHz = 0;

	return u4ChannelInMHz;
}

#define TMP_TEXT_LEN_S 40
#define TMP_TEXT_LEN_L 60
static UCHAR text1[TMP_TEXT_LEN_S], text2[TMP_TEXT_LEN_L], text3[TMP_TEXT_LEN_L];	/* A safe len */

WLAN_STATUS
batchConvertResult(IN P_EVENT_BATCH_RESULT_T prEventBatchResult,
		   OUT PVOID pvBuffer, IN UINT_32 u4MaxBufferLen, OUT PUINT_32 pu4RetLen)
{
	CHAR *p = pvBuffer;
	CHAR ssid[ELEM_MAX_LEN_SSID + 1];
	INT_32 nsize, nsize1, nsize2, nsize3, scancount;
	INT_32 i, j, nleft;
	UINT_32 freq;

	P_EVENT_BATCH_RESULT_ENTRY_T prEntry;
	P_EVENT_BATCH_RESULT_T pBr;

	nsize = 0;
	nleft = u4MaxBufferLen - 5;	/* -5 for "----\n" */

	pBr = prEventBatchResult;
	scancount = 0;
	for (j = 0; j < CFG_BATCH_MAX_MSCAN; j++) {
		scancount += pBr->ucScanCount;
		pBr++;
	}

	nsize1 = kalSnprintf(text1, TMP_TEXT_LEN_S, "scancount=%d\nnextcount=%d\n", scancount, scancount);
	if (nsize1 < nleft) {
		p += nsize1 = kalSnprintf(p, nleft, "%s", text1);
		nleft -= nsize1;
	} else
		goto short_buf;

	pBr = prEventBatchResult;
	for (j = 0; j < CFG_BATCH_MAX_MSCAN; j++) {
		DBGLOG(SCN, TRACE, "convert mscan = %d, apcount=%d, nleft=%d\n", j, pBr->ucScanCount, nleft);

		if (pBr->ucScanCount == 0) {
			pBr++;
			continue;
		}

		nleft -= 5;	/* -5 for "####\n" */

		/* We only support one round scan result now. */
		nsize1 = kalSnprintf(text1, TMP_TEXT_LEN_S, "apcount=%d\n", pBr->ucScanCount);
		if (nsize1 < nleft) {
			p += nsize1 = kalSnprintf(p, nleft, "%s", text1);
			nleft -= nsize1;
		} else
			goto short_buf;

		for (i = 0; i < pBr->ucScanCount; i++) {
			prEntry = &pBr->arBatchResult[i];

			nsize1 = kalSnprintf(text1, TMP_TEXT_LEN_S, "bssid=" MACSTR "\n",
					     MAC2STR(prEntry->aucBssid));

			kalMemCopy(ssid,
				   prEntry->aucSSID,
				   (prEntry->ucSSIDLen < ELEM_MAX_LEN_SSID ? prEntry->ucSSIDLen : ELEM_MAX_LEN_SSID));
			ssid[(prEntry->ucSSIDLen <
			      (ELEM_MAX_LEN_SSID - 1) ? prEntry->ucSSIDLen : (ELEM_MAX_LEN_SSID - 1))] = '\0';
			nsize2 = kalSnprintf(text2, TMP_TEXT_LEN_L, "ssid=%s\n", ssid);

			freq = batchChannelNum2Freq(prEntry->ucFreq);
			nsize3 =
			    kalSnprintf(text3, TMP_TEXT_LEN_L,
					"freq=%u\nlevel=%d\ndist=%u\ndistSd=%u\n====\n",
					freq, prEntry->cRssi, prEntry->u4Dist, prEntry->u4Distsd);

			nsize = nsize1 + nsize2 + nsize3;
			if (nsize < nleft) {

				kalStrnCpy(p, text1, TMP_TEXT_LEN_S);
				p += nsize1;

				kalStrnCpy(p, text2, TMP_TEXT_LEN_L);
				p += nsize2;

				kalStrnCpy(p, text3, TMP_TEXT_LEN_L);
				p += nsize3;

				nleft -= nsize;
			} else {
				DBGLOG(SCN, TRACE, "Warning: Early break! (%d)\n", i);
				break;	/* discard following entries, TODO: apcount? */
			}
		}

		nsize1 = kalSnprintf(text1, TMP_TEXT_LEN_S, "%s", "####\n");
		p += kalSnprintf(p, nleft, "%s", text1);
		nleft -= nsize1;
		pBr++;
	}

	nsize1 = kalSnprintf(text1, TMP_TEXT_LEN_S, "%s", "----\n");
	nleft -= kalSnprintf(p, nleft, "%s", text1);

	*pu4RetLen = u4MaxBufferLen - nleft;
	DBGLOG(SCN, TRACE, "total len = %u (max len = %u)\n", *pu4RetLen, u4MaxBufferLen);

	return WLAN_STATUS_SUCCESS;

short_buf:
	DBGLOG(SCN, TRACE, "Short buffer issue! %u > %u, %s\n",
	       u4MaxBufferLen + (nsize - nleft), u4MaxBufferLen, (PUINT_8)pvBuffer);
	return WLAN_STATUS_INVALID_LENGTH;
}
#endif

/*----------------------------------------------------------------------------*/
/*!
* \brief Private ioctl set int handler.
*
* \param[in] prNetDev Net device requested.
* \param[in] prIwReqInfo Pointer to iwreq structure.
* \param[in] prIwReqData The ioctl data structure, use the field of sub-command.
* \param[in] pcExtra The buffer with input value
*
* \retval 0 For success.
* \retval -EOPNOTSUPP If cmd is not supported.
* \retval -EINVAL If a value is out of range.
*
*/
/*----------------------------------------------------------------------------*/
static int
_priv_set_int(IN struct net_device *prNetDev,
	     IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN char *pcExtra)
{
	UINT_32 u4SubCmd;
	PUINT_32 pu4IntBuf;
	P_NDIS_TRANSPORT_STRUCT prNdisReq;
	P_GLUE_INFO_T prGlueInfo;
	UINT_32 u4BufLen = 0;
	int status = 0;
	P_PTA_IPC_T prPtaIpc;
	unsigned char dtim_skip_count = 0;

	ASSERT(prNetDev);
	ASSERT(prIwReqInfo);
	ASSERT(prIwReqData);
	ASSERT(pcExtra);

	if (GLUE_CHK_PR3(prNetDev, prIwReqData, pcExtra) == FALSE)
		return -EINVAL;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	u4SubCmd = (UINT_32) prIwReqData->mode;
	pu4IntBuf = (PUINT_32) pcExtra;

	switch (u4SubCmd) {
	case PRIV_CMD_TEST_MODE:
		prNdisReq = (P_NDIS_TRANSPORT_STRUCT) &aucOidBuf[0];

		if (pu4IntBuf[1] == PRIV_CMD_TEST_MAGIC_KEY) {
			prNdisReq->ndisOidCmd = OID_CUSTOM_TEST_MODE;
		} else if (pu4IntBuf[1] == 0) {
			prNdisReq->ndisOidCmd = OID_CUSTOM_ABORT_TEST_MODE;
		} else if (pu4IntBuf[1] == PRIV_CMD_TEST_MAGIC_KEY_ICAP) {
			prNdisReq->ndisOidCmd = OID_CUSTOM_TEST_ICAP_MODE;
		} else {
			status = 0;
			break;
		}
		prNdisReq->inNdisOidlength = 0;
		prNdisReq->outNdisOidLength = 0;

		/* Execute this OID */
		status = priv_set_ndis(prNetDev, prNdisReq, &u4BufLen);
		break;

	case PRIV_CMD_TEST_CMD:
		prNdisReq = (P_NDIS_TRANSPORT_STRUCT) &aucOidBuf[0];

		kalMemCopy(&prNdisReq->ndisOidContent[0], &pu4IntBuf[1], 8);

		prNdisReq->ndisOidCmd = OID_CUSTOM_MTK_WIFI_TEST;
		prNdisReq->inNdisOidlength = 8;
		prNdisReq->outNdisOidLength = 8;

		/* Execute this OID */
		status = priv_set_ndis(prNetDev, prNdisReq, &u4BufLen);
		break;

#if CFG_SUPPORT_PRIV_MCR_RW
	case PRIV_CMD_ACCESS_MCR:
		prNdisReq = (P_NDIS_TRANSPORT_STRUCT) &aucOidBuf[0];

		if (pu4IntBuf[1] == PRIV_CMD_TEST_MAGIC_KEY) {
			if (pu4IntBuf[2] == PRIV_CMD_TEST_MAGIC_KEY)
				prGlueInfo->fgMcrAccessAllowed = TRUE;
			status = 0;
			break;
		}
		if (prGlueInfo->fgMcrAccessAllowed) {
			kalMemCopy(&prNdisReq->ndisOidContent[0], &pu4IntBuf[1], 8);

			prNdisReq->ndisOidCmd = OID_CUSTOM_MCR_RW;
			prNdisReq->inNdisOidlength = 8;
			prNdisReq->outNdisOidLength = 8;

			/* Execute this OID */
			status = priv_set_ndis(prNetDev, prNdisReq, &u4BufLen);
		}
		break;
#endif

	case PRIV_CMD_SW_CTRL:
		prNdisReq = (P_NDIS_TRANSPORT_STRUCT) &aucOidBuf[0];

		kalMemCopy(&prNdisReq->ndisOidContent[0], &pu4IntBuf[1], 8);

		prNdisReq->ndisOidCmd = OID_CUSTOM_SW_CTRL;
		prNdisReq->inNdisOidlength = 8;
		prNdisReq->outNdisOidLength = 8;

		/* Execute this OID */
		status = priv_set_ndis(prNetDev, prNdisReq, &u4BufLen);
		break;

#if 0
	case PRIV_CMD_BEACON_PERIOD:
		/* pu4IntBuf[0] is used as input SubCmd */
		rStatus = wlanSetInformation(prGlueInfo->prAdapter, wlanoidSetBeaconInterval, (PVOID)&pu4IntBuf[1],
					     sizeof(UINT_32), &u4BufLen);
		break;
#endif

#if CFG_TCP_IP_CHKSUM_OFFLOAD
	case PRIV_CMD_CSUM_OFFLOAD:
		{
			UINT_32 u4CSUMFlags;

			if (pu4IntBuf[1] == 1)
				u4CSUMFlags = CSUM_OFFLOAD_EN_ALL;
			else if (pu4IntBuf[1] == 0)
				u4CSUMFlags = 0;
			else
				return -EINVAL;

			if (kalIoctl(prGlueInfo,
				     wlanoidSetCSUMOffload,
				     (PVOID)&u4CSUMFlags,
				     sizeof(UINT_32), FALSE, FALSE, TRUE, &u4BufLen) == WLAN_STATUS_SUCCESS) {
				if (pu4IntBuf[1] == 1)
					prNetDev->features |= NETIF_F_HW_CSUM;
				else if (pu4IntBuf[1] == 0)
					prNetDev->features &= ~NETIF_F_HW_CSUM;
			}
		}
		break;
#endif /* CFG_TCP_IP_CHKSUM_OFFLOAD */

	case PRIV_CMD_POWER_MODE:
		{
			PARAM_POWER_MODE_T rPowerMode;
			P_BSS_INFO_T prBssInfo = prGlueInfo->prAdapter->prAisBssInfo;

			if (!prBssInfo)
				break;

			rPowerMode.ePowerMode = (PARAM_POWER_MODE) pu4IntBuf[1];
			rPowerMode.ucBssIdx = prBssInfo->ucBssIndex;

			/* pu4IntBuf[0] is used as input SubCmd */
			kalIoctl(prGlueInfo, wlanoidSet802dot11PowerSaveProfile, &rPowerMode,
				 sizeof(PARAM_POWER_MODE_T), FALSE, FALSE, TRUE, &u4BufLen);
		}
		break;
#ifdef ENABLED_IN_ENGUSERDEBUG
	case PRIV_CMD_TRIGGER_CHIP_RESET:
		{
			trChipReset = (enum UT_TRIGGER_CHIP_RESET)pu4IntBuf[1];
		}
		break;
#endif

	case PRIV_CMD_WMM_PS:
		{
			PARAM_CUSTOM_WMM_PS_TEST_STRUCT_T rWmmPsTest;

			rWmmPsTest.bmfgApsdEnAc = (UINT_8) pu4IntBuf[1];
			rWmmPsTest.ucIsEnterPsAtOnce = (UINT_8) pu4IntBuf[2];
			rWmmPsTest.ucIsDisableUcTrigger = (UINT_8) pu4IntBuf[3];
			rWmmPsTest.reserved = 0;

			kalIoctl(prGlueInfo,
				 wlanoidSetWiFiWmmPsTest,
				 (PVOID)&rWmmPsTest,
				 sizeof(PARAM_CUSTOM_WMM_PS_TEST_STRUCT_T), FALSE, FALSE, TRUE, &u4BufLen);
		}
		break;

#if 0
	case PRIV_CMD_ADHOC_MODE:
		/* pu4IntBuf[0] is used as input SubCmd */
		rStatus = wlanSetInformation(prGlueInfo->prAdapter, wlanoidSetAdHocMode, (PVOID)&pu4IntBuf[1],
					     sizeof(UINT_32), &u4BufLen);
		break;
#endif

	case PRIV_CUSTOM_BWCS_CMD:

		DBGLOG(REQ, INFO,
		       "pu4IntBuf[1] = %x, size of PTA_IPC_T = %zu.\n", pu4IntBuf[1], sizeof(PARAM_PTA_IPC_T));

		prPtaIpc = (P_PTA_IPC_T) aucOidBuf;
		prPtaIpc->u.aucBTPParams[0] = (UINT_8) (pu4IntBuf[1] >> 24);
		prPtaIpc->u.aucBTPParams[1] = (UINT_8) (pu4IntBuf[1] >> 16);
		prPtaIpc->u.aucBTPParams[2] = (UINT_8) (pu4IntBuf[1] >> 8);
		prPtaIpc->u.aucBTPParams[3] = (UINT_8) (pu4IntBuf[1]);

		DBGLOG(REQ, INFO,
		       "BCM BWCS CMD : PRIV_CUSTOM_BWCS_CMD : aucBTPParams[0] = %02x, aucBTPParams[1] = %02x.\n",
			prPtaIpc->u.aucBTPParams[0], prPtaIpc->u.aucBTPParams[1]);
		DBGLOG(REQ, INFO,
		       "BCM BWCS CMD : PRIV_CUSTOM_BWCS_CMD : aucBTPParams[2] = %02x, aucBTPParams[3] = %02x.\n",
			prPtaIpc->u.aucBTPParams[2], prPtaIpc->u.aucBTPParams[3]);

#if 0
		status = wlanSetInformation(prGlueInfo->prAdapter,
					    wlanoidSetBT, (PVOID)&aucOidBuf[0], u4CmdLen, &u4BufLen);
#endif

		status = wlanoidSetBT(prGlueInfo->prAdapter,
				      (PVOID)&aucOidBuf[0], sizeof(PARAM_PTA_IPC_T), &u4BufLen);

		if (status != WLAN_STATUS_SUCCESS)
			status = -EFAULT;

		break;

	case PRIV_CMD_BAND_CONFIG:
		{
			DBGLOG(INIT, INFO, "CMD set_band = %u\n", pu4IntBuf[1]);
		}
		break;

#if CFG_ENABLE_WIFI_DIRECT
	case PRIV_CMD_P2P_MODE:
		{
			PARAM_CUSTOM_P2P_SET_STRUCT_T rSetP2P;
			WLAN_STATUS rWlanStatus = WLAN_STATUS_SUCCESS;

			rSetP2P.u4Enable = pu4IntBuf[1];
			rSetP2P.u4Mode = pu4IntBuf[2];
#if 1
			if (!rSetP2P.u4Enable)
				p2pNetUnregister(prGlueInfo, TRUE);

			/* pu4IntBuf[0] is used as input SubCmd */
			rWlanStatus = kalIoctl(prGlueInfo, wlanoidSetP2pMode, (PVOID)&rSetP2P,
					       sizeof(PARAM_CUSTOM_P2P_SET_STRUCT_T), FALSE, FALSE, TRUE, &u4BufLen);

			if ((rSetP2P.u4Enable) && (rWlanStatus == WLAN_STATUS_SUCCESS))
				p2pNetRegister(prGlueInfo, TRUE);
#endif

		}
		break;
#endif

#if (CFG_MET_PACKET_TRACE_SUPPORT == 1)
	case PRIV_CMD_MET_PROFILING:
		{
			/* PARAM_CUSTOM_WFD_DEBUG_STRUCT_T rWfdDebugModeInfo; */
			/* rWfdDebugModeInfo.ucWFDDebugMode=(UINT_8)pu4IntBuf[1]; */
			/* rWfdDebugModeInfo.u2SNPeriod=(UINT_16)pu4IntBuf[2]; */
			/* DBGLOG(REQ, INFO, ("WFD Debug Mode:%d Period:%d\n", */
			/* rWfdDebugModeInfo.ucWFDDebugMode, rWfdDebugModeInfo.u2SNPeriod)); */
			prGlueInfo->fgMetProfilingEn = (UINT_8) pu4IntBuf[1];
			prGlueInfo->u2MetUdpPort = (UINT_16) pu4IntBuf[2];
			/*
			 * DBGLOG(INIT, INFO, ("MET_PROF: Enable=%d UDP_PORT=%d\n",
			 * prGlueInfo->fgMetProfilingEn, prGlueInfo->u2MetUdpPort);
			 */

		}
		break;

#endif
	case PRIV_CMD_DTIM_SKIP_COUNT:
		dtim_skip_count = (unsigned char)pu4IntBuf[1];
		if (prGlueInfo->prAdapter &&
		    dtim_skip_count >= 0 &&
		    dtim_skip_count <= 6) {
			prGlueInfo->prAdapter->dtim_skip_count =
				dtim_skip_count;
		} else {
			status = -EINVAL;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	return status;
}

/*----------------------------------------------------------------------------*/
/*!
* \brief Private ioctl get int handler.
*
* \param[in] pDev Net device requested.
* \param[out] pIwReq Pointer to iwreq structure.
* \param[in] prIwReqData The ioctl req structure, use the field of sub-command.
* \param[out] pcExtra The buffer with put the return value
*
* \retval 0 For success.
* \retval -EOPNOTSUPP If cmd is not supported.
* \retval -EFAULT For fail.
*
*/
/*----------------------------------------------------------------------------*/
static int
_priv_get_int(IN struct net_device *prNetDev,
	     IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN OUT char *pcExtra)
{
	UINT_32 u4SubCmd;
	PUINT_32 pu4IntBuf;
	P_GLUE_INFO_T prGlueInfo;
	UINT_32 u4BufLen = 0;
	int status = 0;
	P_NDIS_TRANSPORT_STRUCT prNdisReq;

	ASSERT(prNetDev);
	ASSERT(prIwReqInfo);
	ASSERT(prIwReqData);
	ASSERT(pcExtra);
	if (GLUE_CHK_PR3(prNetDev, prIwReqData, pcExtra) == FALSE)
		return -EINVAL;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	u4SubCmd = (UINT_32) prIwReqData->mode;
	pu4IntBuf = (PUINT_32) pcExtra;

	switch (u4SubCmd) {
	case PRIV_CMD_TEST_CMD:
		prNdisReq = (P_NDIS_TRANSPORT_STRUCT) &aucOidBuf[0];

		kalMemCopy(&prNdisReq->ndisOidContent[0], &pu4IntBuf[1], 8);

		prNdisReq->ndisOidCmd = OID_CUSTOM_MTK_WIFI_TEST;
		prNdisReq->inNdisOidlength = 8;
		prNdisReq->outNdisOidLength = 8;

		status = priv_get_ndis(prNetDev, prNdisReq, &u4BufLen);
		if (status == 0) {
			prIwReqData->mode = *(PUINT_32) &prNdisReq->ndisOidContent[4];
			/*
			 * if (copy_to_user(prIwReqData->data.pointer,
			 * &prNdisReq->ndisOidContent[4], 4)) {
			 * return -EFAULT;
			 * }
			 */
		}
		return status;

#if CFG_SUPPORT_PRIV_MCR_RW
	case PRIV_CMD_ACCESS_MCR:
		prNdisReq = (P_NDIS_TRANSPORT_STRUCT) &aucOidBuf[0];

		if (!prGlueInfo->fgMcrAccessAllowed) {
			status = 0;
			return status;
		}

		kalMemCopy(&prNdisReq->ndisOidContent[0], &pu4IntBuf[1], 8);

		prNdisReq->ndisOidCmd = OID_CUSTOM_MCR_RW;
		prNdisReq->inNdisOidlength = 8;
		prNdisReq->outNdisOidLength = 8;

		status = priv_get_ndis(prNetDev, prNdisReq, &u4BufLen);
		if (status == 0)
			prIwReqData->mode = *(PUINT_32) &prNdisReq->ndisOidContent[4];

		return status;
#endif

	case PRIV_CMD_DUMP_MEM:
		prNdisReq = (P_NDIS_TRANSPORT_STRUCT) &aucOidBuf[0];

#if 1
		if (!prGlueInfo->fgMcrAccessAllowed) {
			status = 0;
			return status;
		}
#endif
		kalMemCopy(&prNdisReq->ndisOidContent[0], &pu4IntBuf[1], 8);

		prNdisReq->ndisOidCmd = OID_CUSTOM_MEM_DUMP;
		prNdisReq->inNdisOidlength = sizeof(PARAM_CUSTOM_MEM_DUMP_STRUCT_T);
		prNdisReq->outNdisOidLength = sizeof(PARAM_CUSTOM_MEM_DUMP_STRUCT_T);

		status = priv_get_ndis(prNetDev, prNdisReq, &u4BufLen);
		if (status == 0)
			prIwReqData->mode = *(PUINT_32) &prNdisReq->ndisOidContent[0];
		return status;

	case PRIV_CMD_SW_CTRL:

		prNdisReq = (P_NDIS_TRANSPORT_STRUCT) &aucOidBuf[0];

		kalMemCopy(&prNdisReq->ndisOidContent[0], &pu4IntBuf[1], 8);

		prNdisReq->ndisOidCmd = OID_CUSTOM_SW_CTRL;
		prNdisReq->inNdisOidlength = 8;
		prNdisReq->outNdisOidLength = 8;

		status = priv_get_ndis(prNetDev, prNdisReq, &u4BufLen);
		if (status == 0)
			prIwReqData->mode = *(PUINT_32) &prNdisReq->ndisOidContent[4];

		return status;

#if 0
	case PRIV_CMD_BEACON_PERIOD:
		status = wlanQueryInformation(prGlueInfo->prAdapter,
					      wlanoidQueryBeaconInterval,
					      (PVOID) pu4IntBuf, sizeof(UINT_32), &u4BufLen);
		return status;

	case PRIV_CMD_POWER_MODE:
		status = wlanQueryInformation(prGlueInfo->prAdapter,
					      wlanoidQuery802dot11PowerSaveProfile,
					      (PVOID) pu4IntBuf, sizeof(UINT_32), &u4BufLen);
		return status;

	case PRIV_CMD_ADHOC_MODE:
		status = wlanQueryInformation(prGlueInfo->prAdapter,
					      wlanoidQueryAdHocMode, (PVOID) pu4IntBuf, sizeof(UINT_32), &u4BufLen);
		return status;
#endif

	case PRIV_CMD_BAND_CONFIG:
		DBGLOG(INIT, INFO, "CMD get_band=\n");
		prIwReqData->mode = 0;
		return status;
	case PRIV_CMD_SHOW_CHANNEL:
	{
		UINT_32 freq;

		status = wlanQueryInformation(prGlueInfo->prAdapter, wlanoidQueryFrequency,
			&freq, sizeof(UINT_32), &u4BufLen);
		if (status == 0)
			prIwReqData->mode = freq/1000; /* Hz->MHz */

		return status;
	}
	case PRIV_CMD_DTIM_SKIP_COUNT:
		if (prGlueInfo->prAdapter)
			prIwReqData->mode =
				prGlueInfo->prAdapter->dtim_skip_count;
		return status;
	default:
		break;
	}

	u4SubCmd = (UINT_32) prIwReqData->data.flags;

	switch (u4SubCmd) {
	case PRIV_CMD_GET_CH_LIST:
		{
			UINT_8 ucNumOfChannel, i;
			UINT_8 ucMaxChannelNum = 50;
			RF_CHANNEL_INFO_T aucChannelList[50];
			INT_32 ch[50];

			kalGetChannelList(prGlueInfo, BAND_NULL, ucMaxChannelNum, &ucNumOfChannel, aucChannelList);
			DBGLOG(RLM, INFO, "PRIV_CMD_GET_CH_LIST: return %d channels\n", ucNumOfChannel);
			if (ucNumOfChannel > 50)
				ucNumOfChannel = 50;

			for (i = 0; i < ucNumOfChannel; i++)
				ch[i] = (INT_32) aucChannelList[i].ucChannelNum;

			prIwReqData->data.length = ucNumOfChannel;
			if (copy_to_user(prIwReqData->data.pointer, ch, ucNumOfChannel * sizeof(INT_32)))
				return -EFAULT;
			else
				return status;
		}
	default:
		return -EOPNOTSUPP;
	}

	return status;
}				/* priv_get_int */

/*----------------------------------------------------------------------------*/
/*!
* \brief Private ioctl set int array handler.
*
* \param[in] prNetDev Net device requested.
* \param[in] prIwReqInfo Pointer to iwreq structure.
* \param[in] prIwReqData The ioctl data structure, use the field of sub-command.
* \param[in] pcExtra The buffer with input value
*
* \retval 0 For success.
* \retval -EOPNOTSUPP If cmd is not supported.
* \retval -EINVAL If a value is out of range.
*
*/
/*----------------------------------------------------------------------------*/
static int
_priv_set_ints(IN struct net_device *prNetDev,
	      IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN char *pcExtra)
{
	UINT_16 i = 0;
	UINT_32 u4SubCmd, u4BufLen, u4CmdLen;
	P_GLUE_INFO_T prGlueInfo;
	INT_32  setting[4] = {0};
	int status = 0;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	P_SET_TXPWR_CTRL_T prTxpwr;

	ASSERT(prNetDev);
	ASSERT(prIwReqInfo);
	ASSERT(prIwReqData);
	ASSERT(pcExtra);

	if (GLUE_CHK_PR3(prNetDev, prIwReqData, pcExtra) == FALSE)
		return -EINVAL;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	u4SubCmd = (UINT_32) prIwReqData->data.flags;
	u4CmdLen = (UINT_32) prIwReqData->data.length;

	switch (u4SubCmd) {
	case PRIV_CMD_SET_TX_POWER:
		{
			if (u4CmdLen > 4)
				return -EINVAL;
			if (copy_from_user(setting, prIwReqData->data.pointer, u4CmdLen))
				return -EFAULT;

			prTxpwr = &prGlueInfo->rTxPwr;
			if (setting[0] == 0 && prIwReqData->data.length == 4 /* argc num */) {
				/* 0 (All networks), 1 (legacy STA), 2 (Hotspot AP), 3 (P2P), 4 (BT over Wi-Fi) */
				if (setting[1] == 1 || setting[1] == 0) {
					if (setting[2] == 0 || setting[2] == 1)
						prTxpwr->c2GLegacyStaPwrOffset = setting[3];
					if (setting[2] == 0 || setting[2] == 2)
						prTxpwr->c5GLegacyStaPwrOffset = setting[3];
				}
				if (setting[1] == 2 || setting[1] == 0) {
					if (setting[2] == 0 || setting[2] == 1)
						prTxpwr->c2GHotspotPwrOffset = setting[3];
					if (setting[2] == 0 || setting[2] == 2)
						prTxpwr->c5GHotspotPwrOffset = setting[3];
				}
				if (setting[1] == 3 || setting[1] == 0) {
					if (setting[2] == 0 || setting[2] == 1)
						prTxpwr->c2GP2pPwrOffset = setting[3];
					if (setting[2] == 0 || setting[2] == 2)
						prTxpwr->c5GP2pPwrOffset = setting[3];
				}
				if (setting[1] == 4 || setting[1] == 0) {
					if (setting[2] == 0 || setting[2] == 1)
						prTxpwr->c2GBowPwrOffset = setting[3];
					if (setting[2] == 0 || setting[2] == 2)
						prTxpwr->c5GBowPwrOffset = setting[3];
				}
			} else if (setting[0] == 1 && prIwReqData->data.length == 2) {
				prTxpwr->ucConcurrencePolicy = setting[1];
			} else if (setting[0] == 2 && prIwReqData->data.length == 3) {
				if (setting[1] == 0) {
					for (i = 0; i < 14; i++)
						prTxpwr->acTxPwrLimit2G[i] = setting[2];
				} else if (setting[1] <= 14)
					prTxpwr->acTxPwrLimit2G[setting[1] - 1] = setting[2];
			} else if (setting[0] == 3 && prIwReqData->data.length == 3) {
				if (setting[1] == 0) {
					for (i = 0; i < 4; i++)
						prTxpwr->acTxPwrLimit5G[i] = setting[2];
				} else if (setting[1] <= 4)
					prTxpwr->acTxPwrLimit5G[setting[1] - 1] = setting[2];
			} else if (setting[0] == 4 && prIwReqData->data.length == 2) {
				if (setting[1] == 0)
					wlanDefTxPowerCfg(prGlueInfo->prAdapter);
				rStatus = kalIoctl(prGlueInfo,
						   wlanoidSetTxPower,
						   prTxpwr, sizeof(SET_TXPWR_CTRL_T), FALSE, FALSE, TRUE, &u4BufLen);
			} else
				return -EFAULT;
		}
		return status;
	default:
		break;
	}

	return status;
}

/*----------------------------------------------------------------------------*/
/*!
* \brief Private ioctl get int array handler.
*
* \param[in] pDev Net device requested.
* \param[out] pIwReq Pointer to iwreq structure.
* \param[in] prIwReqData The ioctl req structure, use the field of sub-command.
* \param[out] pcExtra The buffer with put the return value
*
* \retval 0 For success.
* \retval -EOPNOTSUPP If cmd is not supported.
* \retval -EFAULT For fail.
*
*/
/*----------------------------------------------------------------------------*/
static int
_priv_get_ints(IN struct net_device *prNetDev,
	      IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN OUT char *pcExtra)
{
	UINT_32 u4SubCmd;
	P_GLUE_INFO_T prGlueInfo;
	int status = 0;

	ASSERT(prNetDev);
	ASSERT(prIwReqInfo);
	ASSERT(prIwReqData);
	ASSERT(pcExtra);
	if (GLUE_CHK_PR3(prNetDev, prIwReqData, pcExtra) == FALSE)
		return -EINVAL;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	u4SubCmd = (UINT_32) prIwReqData->data.flags;

	switch (u4SubCmd) {
	case PRIV_CMD_GET_CH_LIST:
		{
			UINT_8 ucNumOfChannel, i;
			UINT_8 ucMaxChannelNum = 50;
			RF_CHANNEL_INFO_T aucChannelList[50];
			INT_32 ch[50];

			kalGetChannelList(prGlueInfo, BAND_NULL, ucMaxChannelNum, &ucNumOfChannel, aucChannelList);
			DBGLOG(RLM, INFO, "PRIV_CMD_GET_CH_LIST: return %d channels\n", ucNumOfChannel);
			if (ucNumOfChannel > 50)
				ucNumOfChannel = 50;

			for (i = 0; i < ucNumOfChannel; i++)
				ch[i] = (INT_32) aucChannelList[i].ucChannelNum;

			prIwReqData->data.length = ucNumOfChannel;
			if (copy_to_user(prIwReqData->data.pointer, ch, ucNumOfChannel * sizeof(INT_32)))
				return -EFAULT;
			else
				return status;
		}
	default:
		break;
	}

	return status;
}				/* priv_get_ints */

/*----------------------------------------------------------------------------*/
/*!
* \brief Private ioctl set structure handler.
*
* \param[in] pDev Net device requested.
* \param[in] prIwReqData Pointer to iwreq_data structure.
*
* \retval 0 For success.
* \retval -EOPNOTSUPP If cmd is not supported.
* \retval -EINVAL If a value is out of range.
*
*/
/*----------------------------------------------------------------------------*/
static int
_priv_set_struct(IN struct net_device *prNetDev,
		IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN char *pcExtra)
{
	UINT_32 u4SubCmd = 0;
	int status = 0;
	/* WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS; */
	UINT_32 u4CmdLen = 0;
	P_NDIS_TRANSPORT_STRUCT prNdisReq;

	P_GLUE_INFO_T prGlueInfo = NULL;
	UINT_32 u4BufLen = 0;

	ASSERT(prNetDev);
	/* ASSERT(prIwReqInfo); */
	ASSERT(prIwReqData);
	/* ASSERT(pcExtra); */

	kalMemZero(&aucOidBuf[0], sizeof(aucOidBuf));

	if (GLUE_CHK_PR2(prNetDev, prIwReqData) == FALSE)
		return -EINVAL;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	u4SubCmd = (UINT_32) prIwReqData->data.flags;

#if 0
	DBGLOG(INIT, INFO, "priv_set_struct(): prIwReqInfo->cmd(0x%X), u4SubCmd(%ld)\n",
	       prIwReqInfo->cmd, u4SubCmd;
#endif

	switch (u4SubCmd) {
#if 0				/* PTA_ENABLED */
	case PRIV_CMD_BT_COEXIST:
		u4CmdLen = prIwReqData->data.length * sizeof(UINT_32);
		ASSERT(sizeof(PARAM_CUSTOM_BT_COEXIST_T) >= u4CmdLen);
		if (sizeof(PARAM_CUSTOM_BT_COEXIST_T) < u4CmdLen)
			return -EFAULT;

		if (copy_from_user(&aucOidBuf[0], prIwReqData->data.pointer, u4CmdLen)) {
			status = -EFAULT;	/* return -EFAULT; */
			break;
		}

		rStatus = wlanSetInformation(prGlueInfo->prAdapter,
					     wlanoidSetBtCoexistCtrl, (PVOID)&aucOidBuf[0], u4CmdLen, &u4BufLen);
		if (rStatus != WLAN_STATUS_SUCCESS)
			status = -EFAULT;
		break;
#endif

	case PRIV_CUSTOM_BWCS_CMD:
		u4CmdLen = prIwReqData->data.length * sizeof(UINT_32);
		ASSERT(sizeof(PARAM_PTA_IPC_T) >= u4CmdLen);
		if (sizeof(PARAM_PTA_IPC_T) < u4CmdLen)
			return -EFAULT;
#if CFG_SUPPORT_BCM && CFG_SUPPORT_BCM_BWCS && CFG_SUPPORT_BCM_BWCS_DEBUG
		DBGLOG(REQ, INFO,
		       "ucCmdLen = %d, size of PTA_IPC_T = %d, prIwReqData->data = 0x%x.\n",
			u4CmdLen, sizeof(PARAM_PTA_IPC_T), prIwReqData->data);

		DBGLOG(REQ, INFO, "priv_set_struct(): prIwReqInfo->cmd(0x%X), u4SubCmd(%ld)\n",
				   prIwReqInfo->cmd, u4SubCmd;

		DBGLOG(REQ, INFO, "*pcExtra = 0x%x\n", *pcExtra;
#endif

		if (copy_from_user(&aucOidBuf[0], prIwReqData->data.pointer, u4CmdLen)) {
			status = -EFAULT;	/* return -EFAULT; */
			break;
		}
#if CFG_SUPPORT_BCM && CFG_SUPPORT_BCM_BWCS && CFG_SUPPORT_BCM_BWCS_DEBUG
		DBGLOG(REQ, INFO, "priv_set_struct(): BWCS CMD = %02x%02x%02x%02x\n",
				   aucOidBuf[2], aucOidBuf[3], aucOidBuf[4], aucOidBuf[5];
#endif

#if 0
		status = wlanSetInformation(prGlueInfo->prAdapter,
					    wlanoidSetBT, (PVOID)&aucOidBuf[0], u4CmdLen, &u4BufLen);
#endif

#if 1
		status = wlanoidSetBT(prGlueInfo->prAdapter, (PVOID)&aucOidBuf[0], u4CmdLen, &u4BufLen);
#endif

		if (status != WLAN_STATUS_SUCCESS)
			status = -EFAULT;

		break;

#if CFG_SUPPORT_WPS2
	case PRIV_CMD_WSC_PROBE_REQ:
		{
			/* retrieve IE for Probe Request */
			u4CmdLen = prIwReqData->data.length;
			if (u4CmdLen > GLUE_INFO_WSCIE_LENGTH) {
				DBGLOG(REQ, ERROR, "Input data length is invalid %u\n", u4CmdLen);
				return -EINVAL;
			}

			if (prIwReqData->data.length > 0) {
				if (copy_from_user(prGlueInfo->aucWSCIE,
						   prIwReqData->data.pointer,
						   u4CmdLen)) {
					status = -EFAULT;
					break;
				}
				prGlueInfo->u2WSCIELen = u4CmdLen;
			} else {
				prGlueInfo->u2WSCIELen = 0;
			}
		}
		break;
#endif
	case PRIV_CMD_OID:
		u4CmdLen = prIwReqData->data.length;
		if (u4CmdLen > CMD_OID_BUF_LENGTH) {
			DBGLOG(REQ, ERROR, "Input data length is invalid %u\n", u4CmdLen);
			return -EINVAL;
		}
		if (copy_from_user(&aucOidBuf[0], prIwReqData->data.pointer, u4CmdLen)) {
			status = -EFAULT;
			break;
		}
		if (!kalMemCmp(&aucOidBuf[0], pcExtra, u4CmdLen)) {
			/* ToDo:: DBGLOG */
			DBGLOG(REQ, INFO, "pcExtra buffer is valid\n");
		} else {
			DBGLOG(REQ, INFO, "pcExtra 0x%p\n", pcExtra);
		}
		/* Execute this OID */
		status = priv_set_ndis(prNetDev, (P_NDIS_TRANSPORT_STRUCT)&aucOidBuf[0], &u4BufLen);
		/* Copy result to user space */
		((P_NDIS_TRANSPORT_STRUCT) &aucOidBuf[0])->outNdisOidLength = u4BufLen;

		if (copy_to_user(prIwReqData->data.pointer,
				 &aucOidBuf[0], OFFSET_OF(NDIS_TRANSPORT_STRUCT, ndisOidContent))) {
			DBGLOG(REQ, INFO, "copy_to_user oidBuf fail\n");
			status = -EFAULT;
		}

		break;

	case PRIV_CMD_SW_CTRL:
		u4CmdLen = prIwReqData->data.length;
		prNdisReq = (P_NDIS_TRANSPORT_STRUCT) &aucOidBuf[0];

		if (u4CmdLen > sizeof(prNdisReq->ndisOidContent)) {
			DBGLOG(REQ, ERROR, "Input data length is invalid %u\n", u4CmdLen);
			return -EINVAL;
		}

		if (copy_from_user(&prNdisReq->ndisOidContent[0],
					prIwReqData->data.pointer,
					u4CmdLen)) {
			status = -EFAULT;
			break;
		}
		prNdisReq->ndisOidCmd = OID_CUSTOM_SW_CTRL;
		prNdisReq->inNdisOidlength = 8;
		prNdisReq->outNdisOidLength = 8;

		/* Execute this OID */
		status = priv_set_ndis(prNetDev, prNdisReq, &u4BufLen);
		break;


	case PRIV_CMD_GET_WIFI_TYPE:
		{
			int32_t i4ResultLen;

			u4CmdLen = prIwReqData->data.length;
			if (u4CmdLen >= CMD_OID_BUF_LENGTH) {
				DBGLOG(REQ, ERROR,
				       "u4CmdLen:%u >= CMD_OID_BUF_LENGTH:%d\n",
				       u4CmdLen, CMD_OID_BUF_LENGTH);
				return -EINVAL;
			}

			if (copy_from_user(&aucOidBuf[0],
					   prIwReqData->data.pointer,
					   u4CmdLen)) {
				DBGLOG(REQ, ERROR, "copy_from_user fail\n");
				return -EFAULT;
			}

			aucOidBuf[u4CmdLen] = 0;
			i4ResultLen = priv_driver_cmds(prNetDev, aucOidBuf,
						       u4CmdLen);
			if (i4ResultLen > 1) {
				if (copy_to_user(prIwReqData->data.pointer,
						 &aucOidBuf[0], i4ResultLen)) {
					DBGLOG(REQ, ERROR,
					       "copy_to_user fail\n");
					return -EFAULT;
				}
				prIwReqData->data.length = i4ResultLen;
			} else {
				DBGLOG(REQ, ERROR,
				       "i4ResultLen:%d <= 1\n", i4ResultLen);
				return -EFAULT;
			}

		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	return status;
}

/*----------------------------------------------------------------------------*/
/*!
* \brief Private ioctl get struct handler.
*
* \param[in] pDev Net device requested.
* \param[out] pIwReq Pointer to iwreq structure.
* \param[in] cmd Private sub-command.
*
* \retval 0 For success.
* \retval -EFAULT If copy from user space buffer fail.
* \retval -EOPNOTSUPP Parameter "cmd" not recognized.
*
*/
/*----------------------------------------------------------------------------*/
static int
_priv_get_struct(IN struct net_device *prNetDev,
		IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN OUT char *pcExtra)
{
	UINT_32 u4SubCmd = 0;
	P_NDIS_TRANSPORT_STRUCT prNdisReq = NULL;

	P_GLUE_INFO_T prGlueInfo = NULL;
	UINT_32 u4BufLen = 0;
	PUINT_32 pu4IntBuf = NULL;
	int status = 0;
	static UINT_8 aucBuffer[512];
	static UINT_8 aucBuffer2[512];
	UINT_8 *p_buffer = NULL;
	P_CMD_SW_DBG_CTRL_T pSwDbgCtrl;
	INT_32 i4Rssi = 0;
	UINT_32 u4Rate = 0;
	int n, pos = 0;
	PARAM_MAC_ADDRESS arBssid;
	PARAM_SSID_T ssid;
	int i = 0;
	P_BSS_INFO_T prBssInfo = NULL;
	P_RX_CTRL_T prRxCtrl = NULL;
	PARAM_GET_STA_STA_STATISTICS rQueryStaStatistics;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	P_STA_RECORD_T prStaRec = NULL;
	INT_8 noise = 0;
	char buf[512];
	P_WAKEUP_STATISTIC *prWakeupSta = NULL;

	kalMemZero(&aucOidBuf[0], sizeof(aucOidBuf));

	ASSERT(prNetDev);
	ASSERT(prIwReqData);
	if (!prNetDev || !prIwReqData) {
		DBGLOG(REQ, INFO, "priv_get_struct(): invalid param(0x%p, 0x%p)\n", prNetDev, prIwReqData);
		return -EINVAL;
	}

	u4SubCmd = (UINT_32) prIwReqData->data.flags;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));
	ASSERT(prGlueInfo);
	if (!prGlueInfo) {
		DBGLOG(REQ, INFO, "priv_get_struct(): invalid prGlueInfo(0x%p, 0x%p)\n",
				   prNetDev, *((P_GLUE_INFO_T *) netdev_priv(prNetDev)));
		return -EINVAL;
	}
#if 0
	DBGLOG(INIT, INFO, "priv_get_struct(): prIwReqInfo->cmd(0x%X), u4SubCmd(%ld)\n",
	       prIwReqInfo->cmd, u4SubCmd);
#endif
	memset(aucOidBuf, 0, sizeof(aucOidBuf));

	switch (u4SubCmd) {
	case PRIV_CMD_OID:
		if (copy_from_user(&aucOidBuf[0], prIwReqData->data.pointer, sizeof(NDIS_TRANSPORT_STRUCT))) {
			DBGLOG(REQ, INFO, "priv_get_struct() copy_from_user oidBuf fail\n");
			return -EFAULT;
		}

		prNdisReq = (P_NDIS_TRANSPORT_STRUCT) &aucOidBuf[0];
#if 0
		DBGLOG(INIT, INFO, "\n priv_get_struct cmd 0x%02x len:%d OID:0x%08x OID Len:%d\n",
		       cmd, pIwReq->u.data.length, ndisReq->ndisOidCmd, ndisReq->inNdisOidlength;
#endif
		if (priv_get_ndis(prNetDev, prNdisReq, &u4BufLen) == 0) {
			prNdisReq->outNdisOidLength = u4BufLen;
			if (copy_to_user(prIwReqData->data.pointer,
					 &aucOidBuf[0],
					 u4BufLen + sizeof(NDIS_TRANSPORT_STRUCT) -
					 sizeof(prNdisReq->ndisOidContent))) {
				DBGLOG(REQ, INFO, "priv_get_struct() copy_to_user oidBuf fail(1)\n");
				return -EFAULT;
			}
		} else {
			prNdisReq->outNdisOidLength = u4BufLen;
			if (copy_to_user(prIwReqData->data.pointer,
					 &aucOidBuf[0], OFFSET_OF(NDIS_TRANSPORT_STRUCT, ndisOidContent))) {
				DBGLOG(REQ, INFO, "priv_get_struct() copy_to_user oidBuf fail(2)\n");
			}
			return -EFAULT;
		}
		return 0;

	case PRIV_CMD_SW_CTRL:
		pu4IntBuf = (PUINT_32) prIwReqData->data.pointer;
		prNdisReq = (P_NDIS_TRANSPORT_STRUCT) &aucOidBuf[0];

		if (prIwReqData->data.length > sizeof(prNdisReq->ndisOidContent)) {
			DBGLOG(REQ, INFO, "priv_get_struct() exceeds length limit\n");
			return -EFAULT;
		}

		if (copy_from_user(&prNdisReq->ndisOidContent[0],
				   prIwReqData->data.pointer,
				   prIwReqData->data.length)) {
			DBGLOG(REQ, INFO, "priv_get_struct() copy_from_user oidBuf fail\n");
			return -EFAULT;
		}

		prNdisReq->ndisOidCmd = OID_CUSTOM_SW_CTRL;
		prNdisReq->inNdisOidlength = 8;
		prNdisReq->outNdisOidLength = 8;

		if (!priv_get_ndis(prNetDev, prNdisReq, &u4BufLen)) {
			prNdisReq->outNdisOidLength = u4BufLen;
			if (copy_to_user(prIwReqData->data.pointer,
					 &prNdisReq->ndisOidContent[4],
					 4 /* OFFSET_OF(NDIS_TRANSPORT_STRUCT, ndisOidContent) */)) {
				DBGLOG(REQ, INFO, "priv_get_struct() copy_to_user oidBuf fail(2)\n");
			}
		}
		return 0;
#if 0
	case PRIV_CMD_INT_STAT:
		kalMemZero(buf, 512);

		prWakeupSta = prGlueInfo->prAdapter->arWakeupStatistic;
		pos += snprintf(buf, sizeof(buf),
				"Abnormal Interrupt:%d\n"
				"Software Interrupt:%d\n"
				"TX Interrupt:%d\n"
				"RX data:%d\n"
				"RX Event:%d\n"
				"RX mgmt:%d\n"
				"RX others:%d\n",
				prWakeupSta[0].u2Count,
				prWakeupSta[1].u2Count,
				prWakeupSta[2].u2Count,
				prWakeupSta[3].u2Count,
				prWakeupSta[4].u2Count,
				prWakeupSta[5].u2Count,
				prWakeupSta[6].u2Count);
		for (i = 0; i < EVENT_ID_END; i++) {
			if (prGlueInfo->prAdapter->wake_event_count[i] > 0)
				pos += snprintf(buf + pos, sizeof(buf) - pos,
						"RX EVENT[0x%0x]:%d\n", i,
						prGlueInfo->prAdapter->wake_event_count[i]);
		}

		if (copy_to_user(prIwReqData->data.pointer, &buf[0], sizeof(buf)))
			return -EFAULT;
		else
			return status;
	case PRIV_CMD_STAT:
	{
		pSwDbgCtrl = (P_CMD_SW_DBG_CTRL_T)aucBuffer;
		p_buffer = &aucBuffer2[0];

		prRxCtrl = &prGlueInfo->prAdapter->rRxCtrl;

		kalMemZero(arBssid, MAC_ADDR_LEN);
		kalMemZero(&rQueryStaStatistics, sizeof(rQueryStaStatistics));

		if (kalIoctl(prGlueInfo, wlanoidQueryBssid,
			     &arBssid[0], sizeof(arBssid),
			     TRUE, TRUE, TRUE,
			     &u4BufLen) == WLAN_STATUS_SUCCESS) {

			COPY_MAC_ADDR(rQueryStaStatistics.aucMacAddr, arBssid);
			rQueryStaStatistics.ucReadClear = TRUE;

			rStatus = kalIoctl(prGlueInfo, wlanoidQueryStaStatistics,
					   &rQueryStaStatistics, sizeof(rQueryStaStatistics),
					   TRUE, FALSE, TRUE,
					   &u4BufLen);
		}

		if (kalIoctl(prGlueInfo, wlanoidQueryDbgCntr,
			     (PVOID)aucBuffer, sizeof(UINT_8) * 512,
			     TRUE, TRUE, TRUE,
			     &u4BufLen) == WLAN_STATUS_SUCCESS) {

			if (pSwDbgCtrl && prRxCtrl) {
				if (pSwDbgCtrl->u4Data == SWCR_DBG_TYPE_ALL) {
					n = sprintf(&p_buffer[pos],
						    "Tx success = %d\n",
						    rQueryStaStatistics.u4TransmitCount);
					pos += n;
					n = sprintf(&p_buffer[pos],
						    "Tx retry count = %d\n",
						    pSwDbgCtrl->u4DebugCnt[SWCR_DBG_ALL_TX_RETRY_CNT]);
					pos += n;
					n = sprintf(&p_buffer[pos],
						    "Tx fail to Rcv ACK after retry = %d\n",
						    rQueryStaStatistics.u4TxFailCount +
						    rQueryStaStatistics.u4TxLifeTimeoutCount);
					pos += n;
					n = sprintf(&p_buffer[pos],
						    "Rx success = %d\n",
						    RX_GET_CNT(prRxCtrl, RX_MPDU_TOTAL_COUNT));
					pos += n;
					n = sprintf(&p_buffer[pos],
						    "Rx with CRC = %d\n",
						    pSwDbgCtrl->u4DebugCnt[SWCR_DBG_ALL_RX_FCSERR_CNT]);
					pos += n;
					n = sprintf(&p_buffer[pos],
						    "Rx drop due to out of resource = %d\n",
						    pSwDbgCtrl->u4DebugCnt[SWCR_DBG_ALL_RX_FIFOFULL_CNT]);
					pos += n;
					n = sprintf(&p_buffer[pos],
						    "Rx duplicate frame = %d\n",
						    RX_GET_CNT(prRxCtrl, RX_DUP_DROP_COUNT));
					pos += n;
					n = sprintf(&p_buffer[pos],
						    "False CCA(total) =\n");
					pos += n;
					n = sprintf(&p_buffer[pos],
						    "False CCA(one-second) =\n");
					pos += n;
				}
			}
		}

		if (kalIoctl(prGlueInfo, wlanoidQueryRssi, &i4Rssi, sizeof(i4Rssi),
				TRUE, TRUE, TRUE, &u4BufLen) == WLAN_STATUS_SUCCESS) {
			prStaRec = cnmGetStaRecByAddress(prGlueInfo->prAdapter,
							 prGlueInfo->prAdapter->prAisBssInfo->ucBssIndex,
							 prGlueInfo->prAdapter->rWlanInfo.rCurrBssId.arMacAddress);
			if (prStaRec)
				noise = prStaRec->noise_avg - 127;

			n = sprintf(&p_buffer[pos], "RSSI = %d\n", i4Rssi);
			pos += n;
			n = sprintf(&p_buffer[pos], "P2P GO RSSI =\n");
			pos += n;
			n = sprintf(&p_buffer[pos], "SNR-A =\n");
			pos += n;
			n = sprintf(&p_buffer[pos], "SNR-B (if available) =\n");
			pos += n;
			n = sprintf(&p_buffer[pos], "NoiseLevel-A = %d\n", noise);
			pos += n;
			n = sprintf(&p_buffer[pos], "NoiseLevel-B =\n");
			pos += n;
		}

		status = kalIoctl(prGlueInfo, wlanoidQueryLinkSpeed, &u4Rate,
						sizeof(u4Rate), TRUE, TRUE, TRUE, &u4BufLen);

		/* STA stats */
		if (kalIoctl(prGlueInfo, wlanoidQueryBssid,
			     &arBssid[0], sizeof(arBssid),
			     TRUE, TRUE, TRUE,
			     &u4BufLen) == WLAN_STATUS_SUCCESS) {

			prBssInfo =
				&(prGlueInfo->prAdapter->rWifiVar.arBssInfoPool[KAL_NETWORK_TYPE_AIS_INDEX]);

			kalIoctl(prGlueInfo, wlanoidQuerySsid,
				 &ssid, sizeof(ssid),
				 TRUE, TRUE, TRUE,
				 &u4BufLen);

			n = sprintf(&p_buffer[pos], "\n[STA] connected AP MAC Address = ");
			pos += n;

			for (i = 0; i < PARAM_MAC_ADDR_LEN; i++) {

				n = sprintf(&p_buffer[pos], "%02x", arBssid[i]);
				pos += n;
				if (i != PARAM_MAC_ADDR_LEN - 1) {

					n = sprintf(&p_buffer[pos], ":");
					pos += n;
				}
			}
			n = sprintf(&p_buffer[pos], "\n");
			pos += n;

			n = sprintf(&p_buffer[pos], "PhyMode:");
			pos += n;
			switch (prBssInfo->ucPhyTypeSet) {
			case PHY_TYPE_SET_802_11B:
				n = sprintf(&p_buffer[pos], "802.11b\n");
				pos += n;
				break;
			case PHY_TYPE_SET_802_11ABG:
			case PHY_TYPE_SET_802_11BG:
				n = sprintf(&p_buffer[pos], "802.11g\n");
				pos += n;
				break;
			case PHY_TYPE_SET_802_11A:
				n = sprintf(&p_buffer[pos], "802.11a\n");
				pos += n;
				break;
			case PHY_TYPE_SET_802_11ABGN:
			case PHY_TYPE_SET_802_11BGN:
			case PHY_TYPE_SET_802_11AN:
			case PHY_TYPE_SET_802_11GN:
				n = sprintf(&p_buffer[pos], "802.11n\n");
				pos += n;
				break;
			case PHY_TYPE_SET_802_11ABGNAC:
			case PHY_TYPE_SET_802_11ANAC:
			case PHY_TYPE_SET_802_11AC:
				n = sprintf(&p_buffer[pos], "802.11ac\n");
				pos += n;
				break;
			default:
				break;
			}

			n = sprintf(&p_buffer[pos], "RSSI =\n");
			pos += n;
			n = sprintf(&p_buffer[pos], "Last TX Rate = %d\n", u4Rate*100);
			pos += n;

			if (prStaRec) {
				n = sprintf(&p_buffer[pos], "Last RX Rate = %d\n",
					prStaRec->u2LastPhyRate * 100000);
			} else {
				n = sprintf(&p_buffer[pos], "Last RX Rate =\n");
			}
			pos += n;
		} else {
			n = sprintf(&p_buffer[pos], "\n[STA] Not connected\n");
			pos += n;
		}

		DBGLOG(REQ, INFO, "%s() stat length %d\n", __func__, pos);

		if (copy_to_user(prIwReqData->data.pointer, &aucBuffer2[0], pos + 1)) {
			DBGLOG(REQ, ERROR, "%s() copy_to_user() fail\n", __func__);
			return -EFAULT;
		} else
			return 0;
	}
	case PRIV_CMD_CONNSTATUS:
	{
		PARAM_MAC_ADDRESS arBssid;
		PARAM_SSID_T ssid;
		char buffer[128];
		char *pc;
		WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
		int i, n;

		kalMemZero(arBssid, MAC_ADDR_LEN);
		rStatus = kalIoctl(prGlueInfo, wlanoidQueryBssid,
				   &arBssid[0], sizeof(arBssid),
				   TRUE, TRUE, TRUE,
				   &u4BufLen);

		kalMemZero(buffer, 128);
		pc = &buffer[0];
		if (rStatus == WLAN_STATUS_SUCCESS) {
			kalIoctl(prGlueInfo, wlanoidQuerySsid,
				 &ssid, sizeof(ssid),
				 TRUE, TRUE, TRUE,
				 &u4BufLen);

			n = sprintf(pc, "connStatus: Connected (AP: %s [", ssid.aucSsid);
			pc += n;

			for (i = 0; i < PARAM_MAC_ADDR_LEN; i++) {
				n = sprintf(pc, "%02x", arBssid[i]);
				pc += n;
				if (i != PARAM_MAC_ADDR_LEN - 1) {
					n = sprintf(pc, ":");
					pc += n;
				}
			}
			n = sprintf(pc, "])");
		} else
			n = sprintf(pc, "connStatus: Not connected");

		if (copy_to_user(prIwReqData->data.pointer, &buffer[0], 128))
			return -EFAULT;
		else
			return status;
	}
#endif
	default:
		DBGLOG(REQ, WARN, "get struct cmd:0x%x\n", u4SubCmd);
		break;
	}
	return -EOPNOTSUPP;
}				/* priv_get_struct */

/*----------------------------------------------------------------------------*/
/*!
* \brief The routine handles a set operation for a single OID.
*
* \param[in] pDev Net device requested.
* \param[in] ndisReq Ndis request OID information copy from user.
* \param[out] outputLen_p If the call is successful, returns the number of
*                         bytes written into the query buffer. If the
*                         call failed due to invalid length of the query
*                         buffer, returns the amount of storage needed..
*
* \retval 0 On success.
* \retval -EOPNOTSUPP If cmd is not supported.
*
*/
/*----------------------------------------------------------------------------*/
static int
priv_set_ndis(IN struct net_device *prNetDev, IN NDIS_TRANSPORT_STRUCT * prNdisReq, OUT PUINT_32 pu4OutputLen)
{
	P_WLAN_REQ_ENTRY prWlanReqEntry = NULL;
	WLAN_STATUS status = WLAN_STATUS_SUCCESS;
	P_GLUE_INFO_T prGlueInfo = NULL;
	UINT_32 u4SetInfoLen = 0;

	ASSERT(prNetDev);
	ASSERT(prNdisReq);
	ASSERT(pu4OutputLen);

	if (!prNetDev || !prNdisReq || !pu4OutputLen) {
		DBGLOG(REQ, INFO, "priv_set_ndis(): invalid param(0x%p, 0x%p, 0x%p)\n",
				   prNetDev, prNdisReq, pu4OutputLen);
		return -EINVAL;
	}

	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));
	ASSERT(prGlueInfo);
	if (!prGlueInfo) {
		DBGLOG(REQ, INFO, "priv_set_ndis(): invalid prGlueInfo(0x%p, 0x%p)\n",
				   prNetDev, *((P_GLUE_INFO_T *) netdev_priv(prNetDev)));
		return -EINVAL;
	}
#if 0
	DBGLOG(INIT, INFO, "priv_set_ndis(): prNdisReq->ndisOidCmd(0x%lX)\n", prNdisReq->ndisOidCmd);
#endif

	if (reqSearchSupportedOidEntry(prNdisReq->ndisOidCmd, &prWlanReqEntry) == FALSE) {
		/* WARNLOG(("Set OID: 0x%08lx (unknown)\n", prNdisReq->ndisOidCmd)); */
		return -EOPNOTSUPP;
	}

	if (prWlanReqEntry->pfOidSetHandler == NULL) {
		/* WARNLOG(("Set %s: Null set handler\n", prWlanReqEntry->pucOidName)); */
		return -EOPNOTSUPP;
	}
#if 0
	DBGLOG(INIT, INFO, "priv_set_ndis(): %s\n", prWlanReqEntry->pucOidName);
#endif

	if (prWlanReqEntry->fgSetBufLenChecking) {
		if (prNdisReq->inNdisOidlength != prWlanReqEntry->u4InfoBufLen) {
			DBGLOG(REQ, WARN, "Set %s: Invalid length (current=%u, needed=%u)\n",
					   prWlanReqEntry->pucOidName,
					   prNdisReq->inNdisOidlength, prWlanReqEntry->u4InfoBufLen);

			*pu4OutputLen = prWlanReqEntry->u4InfoBufLen;
			return -EINVAL;
		}
	}

	if (prWlanReqEntry->eOidMethod == ENUM_OID_GLUE_ONLY) {
		/* GLUE sw info only */
		status = prWlanReqEntry->pfOidSetHandler(prGlueInfo,
							 prNdisReq->ndisOidContent,
							 prNdisReq->inNdisOidlength, &u4SetInfoLen);
	} else if (prWlanReqEntry->eOidMethod == ENUM_OID_GLUE_EXTENSION) {
		/* multiple sw operations */
		status = prWlanReqEntry->pfOidSetHandler(prGlueInfo,
							 prNdisReq->ndisOidContent,
							 prNdisReq->inNdisOidlength, &u4SetInfoLen);
	} else if (prWlanReqEntry->eOidMethod == ENUM_OID_DRIVER_CORE) {
		/* driver core */

		status = kalIoctl(prGlueInfo,
				  (PFN_OID_HANDLER_FUNC) prWlanReqEntry->pfOidSetHandler,
				  prNdisReq->ndisOidContent,
				  prNdisReq->inNdisOidlength, FALSE, FALSE, TRUE, &u4SetInfoLen);
	} else {
		DBGLOG(REQ, INFO, "priv_set_ndis(): unsupported OID method:0x%x\n", prWlanReqEntry->eOidMethod);
		return -EOPNOTSUPP;
	}

	*pu4OutputLen = u4SetInfoLen;

	switch (status) {
	case WLAN_STATUS_SUCCESS:
		break;

	case WLAN_STATUS_INVALID_LENGTH:
		/* WARNLOG(("Set %s: Invalid length (current=%ld, needed=%ld)\n", */
		/* prWlanReqEntry->pucOidName, */
		/* prNdisReq->inNdisOidlength, */
		/* u4SetInfoLen)); */
		break;
	}

	if (status != WLAN_STATUS_SUCCESS)
		return -EFAULT;

	return 0;
}				/* priv_set_ndis */

/*----------------------------------------------------------------------------*/
/*!
* \brief The routine handles a query operation for a single OID. Basically we
*   return information about the current state of the OID in question.
*
* \param[in] pDev Net device requested.
* \param[in] ndisReq Ndis request OID information copy from user.
* \param[out] outputLen_p If the call is successful, returns the number of
*                        bytes written into the query buffer. If the
*                        call failed due to invalid length of the query
*                        buffer, returns the amount of storage needed..
*
* \retval 0 On success.
* \retval -EOPNOTSUPP If cmd is not supported.
* \retval -EINVAL invalid input parameters
*
*/
/*----------------------------------------------------------------------------*/
static int
priv_get_ndis(IN struct net_device *prNetDev, IN NDIS_TRANSPORT_STRUCT * prNdisReq, OUT PUINT_32 pu4OutputLen)
{
	P_WLAN_REQ_ENTRY prWlanReqEntry = NULL;
	UINT_32 u4BufLen = 0;
	WLAN_STATUS status = WLAN_STATUS_SUCCESS;
	P_GLUE_INFO_T prGlueInfo = NULL;

	ASSERT(prNetDev);
	ASSERT(prNdisReq);
	ASSERT(pu4OutputLen);

	if (!prNetDev || !prNdisReq || !pu4OutputLen) {
		DBGLOG(REQ, INFO, "priv_get_ndis(): invalid param(0x%p, 0x%p, 0x%p)\n",
				   prNetDev, prNdisReq, pu4OutputLen);
		return -EINVAL;
	}

	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));
	ASSERT(prGlueInfo);
	if (!prGlueInfo) {
		DBGLOG(REQ, INFO, "priv_get_ndis(): invalid prGlueInfo(0x%p, 0x%p)\n",
				   prNetDev, *((P_GLUE_INFO_T *) netdev_priv(prNetDev)));
		return -EINVAL;
	}
#if 0
	DBGLOG(INIT, INFO, "priv_get_ndis(): prNdisReq->ndisOidCmd(0x%lX)\n", prNdisReq->ndisOidCmd);
#endif

	if (reqSearchSupportedOidEntry(prNdisReq->ndisOidCmd, &prWlanReqEntry) == FALSE) {
		/* WARNLOG(("Query OID: 0x%08lx (unknown)\n", prNdisReq->ndisOidCmd)); */
		return -EOPNOTSUPP;
	}

	if (prWlanReqEntry->pfOidQueryHandler == NULL) {
		/* WARNLOG(("Query %s: Null query handler\n", prWlanReqEntry->pucOidName)); */
		return -EOPNOTSUPP;
	}
#if 0
	DBGLOG(INIT, INFO, "priv_get_ndis(): %s\n", prWlanReqEntry->pucOidName);
#endif

	if (prWlanReqEntry->fgQryBufLenChecking) {
		if (prNdisReq->inNdisOidlength < prWlanReqEntry->u4InfoBufLen) {
			/* Not enough room in InformationBuffer. Punt */
			/* WARNLOG(("Query %s: Buffer too short (current=%ld, needed=%ld)\n", */
			/* prWlanReqEntry->pucOidName, */
			/* prNdisReq->inNdisOidlength, */
			/* prWlanReqEntry->u4InfoBufLen)); */

			*pu4OutputLen = prWlanReqEntry->u4InfoBufLen;

			status = WLAN_STATUS_INVALID_LENGTH;
			return -EINVAL;
		}
	}

	if (prWlanReqEntry->eOidMethod == ENUM_OID_GLUE_ONLY) {
		/* GLUE sw info only */
		status = prWlanReqEntry->pfOidQueryHandler(prGlueInfo,
							   prNdisReq->ndisOidContent,
							   prNdisReq->inNdisOidlength, &u4BufLen);
	} else if (prWlanReqEntry->eOidMethod == ENUM_OID_GLUE_EXTENSION) {
		/* multiple sw operations */
		status = prWlanReqEntry->pfOidQueryHandler(prGlueInfo,
							   prNdisReq->ndisOidContent,
							   prNdisReq->inNdisOidlength, &u4BufLen);
	} else if (prWlanReqEntry->eOidMethod == ENUM_OID_DRIVER_CORE) {
		/* driver core */

		status = kalIoctl(prGlueInfo,
				  (PFN_OID_HANDLER_FUNC) prWlanReqEntry->pfOidQueryHandler,
				  prNdisReq->ndisOidContent, prNdisReq->inNdisOidlength, TRUE, TRUE, TRUE, &u4BufLen);
	} else {
		DBGLOG(REQ, INFO, "priv_set_ndis(): unsupported OID method:0x%x\n", prWlanReqEntry->eOidMethod);
		return -EOPNOTSUPP;
	}

	*pu4OutputLen = u4BufLen;

	switch (status) {
	case WLAN_STATUS_SUCCESS:
		break;

	case WLAN_STATUS_INVALID_LENGTH:
		/* WARNLOG(("Set %s: Invalid length (current=%ld, needed=%ld)\n", */
		/* prWlanReqEntry->pucOidName, */
		/* prNdisReq->inNdisOidlength, */
		/* u4BufLen)); */
		break;
	}

	if (status != WLAN_STATUS_SUCCESS)
		return -EOPNOTSUPP;

	return 0;
}				/* priv_get_ndis */

#if CFG_SUPPORT_QA_TOOL
/*----------------------------------------------------------------------------*/
/*!
* \brief The routine handles ATE set operation.
*
* \param[in] pDev Net device requested.
* \param[in] ndisReq Ndis request OID information copy from user.
* \param[out] outputLen_p If the call is successful, returns the number of
*                         bytes written into the query buffer. If the
*                         call failed due to invalid length of the query
*                         buffer, returns the amount of storage needed..
*
* \retval 0 On success.
* \retval -EOPNOTSUPP If cmd is not supported.
* \retval -EFAULT If copy from user space buffer fail.
*
*/
/*----------------------------------------------------------------------------*/
int
priv_ate_set(IN struct net_device *prNetDev,
	     IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN char *pcExtra)
{
	P_GLUE_INFO_T GlueInfo;
	INT_32 i4Status;
	UINT_8 *InBuf;
	/* UINT_8 *addr_str, *value_str; */
	UINT_32 InBufLen;
	UINT_32 u4SubCmd;
	/* BOOLEAN isWrite = 0;
	 * UINT_32 u4BufLen = 0;
	 * P_NDIS_TRANSPORT_STRUCT prNdisReq;
	 * UINT_32 pu4IntBuf[2];
	 */
	UINT_32 u4CopySize = sizeof(aucOidBuf);

	/* sanity check */
	ASSERT(prNetDev);
	ASSERT(prIwReqInfo);
	ASSERT(prIwReqData);
	ASSERT(pcExtra);

	/* init */
	if (GLUE_CHK_PR3(prNetDev, prIwReqData, pcExtra) == FALSE)
		return -EINVAL;

	GlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	u4SubCmd = (UINT_32) prIwReqData->data.flags;

	DBGLOG(REQ, INFO, " priv_ate_set u4SubCmd = %d\n", u4SubCmd);

	switch (u4SubCmd) {
	case PRIV_QACMD_SET:
		DBGLOG(REQ, INFO, " priv_ate_set PRIV_QACMD_SET\n");
		u4CopySize = (prIwReqData->data.length < u4CopySize)
			? prIwReqData->data.length : (u4CopySize - 1);
		InBuf = aucOidBuf;
		InBufLen = prIwReqData->data.length;
		i4Status = 0;

		if (copy_from_user(InBuf, prIwReqData->data.pointer, u4CopySize))
			return -EFAULT;
		aucOidBuf[u4CopySize] = '\0';
		DBGLOG(REQ, INFO, "PRIV_QACMD_SET: priv_set_string=(%s)(%u,%u)\n",
			aucOidBuf, u4CopySize,
			(UINT_32)prIwReqData->data.length);
		i4Status = AteCmdSetHandle(prNetDev, InBuf, InBufLen);
		break;

	default:
		return -EOPNOTSUPP;
	}
	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
/*!
* \brief This routine is called to search desired OID.
*
* \param rOid[in]               Desired NDIS_OID
* \param ppWlanReqEntry[out]    Found registered OID entry
*
* \retval TRUE: Matched OID is found
* \retval FALSE: No matched OID is found
*/
/*----------------------------------------------------------------------------*/
static BOOLEAN reqSearchSupportedOidEntry(IN UINT_32 rOid, OUT P_WLAN_REQ_ENTRY *ppWlanReqEntry)
{
	INT_32 i, j, k;

	i = 0;
	j = NUM_SUPPORTED_OIDS - 1;

	while (i <= j) {
		k = (i + j) / 2;

		if (rOid == arWlanOidReqTable[k].rOid) {
			*ppWlanReqEntry = &arWlanOidReqTable[k];
			return TRUE;
		} else if (rOid < arWlanOidReqTable[k].rOid) {
			j = k - 1;
		} else {
			i = k + 1;
		}
	}

	return FALSE;
}				/* reqSearchSupportedOidEntry */

int
priv_get_string(IN struct net_device *prNetDev,
		IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN OUT char *pcExtra)
{
	UINT_32 u4SubCmd = 0;
	UINT_32 u4TotalLen = 2000;
	P_GLUE_INFO_T prGlueInfo = NULL;
	UINT_32 u4BufLen = 0;
	int i,pos = 0;
	char *buf = pcExtra;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;

	ASSERT(prNetDev);
	ASSERT(prIwReqData);
	if (!prNetDev || !prIwReqData) {
		DBGLOG(REQ, INFO, "priv_get_string(): invalid param(0x%p, 0x%p)\n", prNetDev, prIwReqData);
		return -EINVAL;
	}

	u4SubCmd = (UINT_32) prIwReqData->data.flags;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));
	ASSERT(prGlueInfo);
	if (!prGlueInfo) {
		DBGLOG(REQ, INFO, "priv_get_string(): invalid prGlueInfo(0x%p, 0x%p)\n",
				   prNetDev, *((P_GLUE_INFO_T *) netdev_priv(prNetDev)));
		return -EINVAL;
	}

	if (pcExtra) {
		pcExtra[u4TotalLen] = '\0';
	}

	pos += scnprintf(buf + pos, u4TotalLen - pos, "\n");

	switch (u4SubCmd) {
	case PRIV_CMD_INT_STAT:
	{
		P_WAKEUP_STATISTIC *prWakeupSta = NULL;
		prWakeupSta = g_arWakeupStatistic;
		pos += scnprintf(buf + pos, u4TotalLen - pos,
				"Abnormal Interrupt:%u\n"
				"Software Interrupt:%u\n"
				"TX Interrupt:%u\n"
				"RX data:%u\n"
				"RX Event:%u\n"
				"RX mgmt:%u\n"
				"RX others:%u\n",
				prWakeupSta[0].u4Count,
				prWakeupSta[1].u4Count,
				prWakeupSta[2].u4Count,
				prWakeupSta[3].u4Count,
				prWakeupSta[4].u4Count,
				prWakeupSta[5].u4Count,
				prWakeupSta[6].u4Count);
		for (i = 0; i < EVENT_ID_END; i++) {
			if (g_wake_event_count[i] > 0)
				pos += scnprintf(buf + pos, u4TotalLen - pos,
						"RX EVENT[0x%0x]:%u\n", i,
						g_wake_event_count[i]);
		}
		break;
	}
	case PRIV_CMD_STAT:
	{
		static UINT_8 aucBuffer[512];
		P_CMD_SW_DBG_CTRL_T pSwDbgCtrl;
		INT_32 i4Rssi;
		UINT_32 u4Rate = 0;
		INT_8 noise = 0;

		PARAM_MAC_ADDRESS arBssid;
		PARAM_SSID_T ssid;
		P_BSS_INFO_T prBssInfo;
		P_STA_RECORD_T prStaRec = NULL;
		P_RX_CTRL_T prRxCtrl = NULL;
		PARAM_GET_STA_STA_STATISTICS rQueryStaStatistics;
		pSwDbgCtrl = (P_CMD_SW_DBG_CTRL_T)aucBuffer;
		prRxCtrl = &prGlueInfo->prAdapter->rRxCtrl;

		kalMemZero(arBssid, MAC_ADDR_LEN);
		kalMemZero(&rQueryStaStatistics, sizeof(rQueryStaStatistics));

		if (kalIoctl(prGlueInfo, wlanoidQueryBssid,
			     &arBssid[0], sizeof(arBssid),
			     TRUE, TRUE, TRUE,
			     &u4BufLen) == WLAN_STATUS_SUCCESS) {

			COPY_MAC_ADDR(rQueryStaStatistics.aucMacAddr, arBssid);
			rQueryStaStatistics.ucReadClear = TRUE;

			rStatus = kalIoctl(prGlueInfo, wlanoidQueryStaStatistics,
					   &rQueryStaStatistics, sizeof(rQueryStaStatistics),
					   TRUE, FALSE, TRUE,
					   &u4BufLen);
		}

		if (kalIoctl(prGlueInfo, wlanoidQueryDbgCntr,
			     (PVOID)aucBuffer, sizeof(UINT_8) * 512,
			     TRUE, TRUE, TRUE,
			     &u4BufLen) == WLAN_STATUS_SUCCESS) {

			if (pSwDbgCtrl && prRxCtrl) {
				if (pSwDbgCtrl->u4Data == SWCR_DBG_TYPE_ALL) {
					pos += scnprintf(buf + pos, u4TotalLen - pos,
						    "Tx success = %d\n",
						    rQueryStaStatistics.u4TransmitCount);

					pos += scnprintf(buf + pos, u4TotalLen - pos,
						    "Tx retry count = %d\n",
						    pSwDbgCtrl->u4DebugCnt[SWCR_DBG_ALL_TX_RETRY_CNT]);

					pos += scnprintf(buf + pos, u4TotalLen - pos,
						    "Tx fail to Rcv ACK after retry = %d\n",
						    rQueryStaStatistics.u4TxFailCount +
						    rQueryStaStatistics.u4TxLifeTimeoutCount);

					pos += scnprintf(buf + pos, u4TotalLen - pos,
						    "Rx success = %d\n",
						    RX_GET_CNT(prRxCtrl, RX_MPDU_TOTAL_COUNT));

					pos += scnprintf(buf + pos, u4TotalLen - pos,
						    "Rx with CRC = %d\n",
						    pSwDbgCtrl->u4DebugCnt[SWCR_DBG_ALL_RX_FCSERR_CNT]);

					pos += scnprintf(buf + pos, u4TotalLen - pos,
						    "Rx drop due to out of resource = %d\n",
						    pSwDbgCtrl->u4DebugCnt[SWCR_DBG_ALL_RX_FIFOFULL_CNT]);

					pos += scnprintf(buf + pos, u4TotalLen - pos,
						    "Rx duplicate frame = %d\n",
						    RX_GET_CNT(prRxCtrl, RX_DUP_DROP_COUNT));

					pos += scnprintf(buf + pos, u4TotalLen - pos,
						    "False CCA(total) =\n");

					pos += scnprintf(buf + pos, u4TotalLen - pos,
						    "False CCA(one-second) =\n");
				}
			}
		}

		if (kalIoctl(prGlueInfo, wlanoidQueryRssi, &i4Rssi, sizeof(i4Rssi),
				TRUE, TRUE, TRUE, &u4BufLen) == WLAN_STATUS_SUCCESS) {
			prStaRec = cnmGetStaRecByAddress(prGlueInfo->prAdapter,
							 prGlueInfo->prAdapter->prAisBssInfo->ucBssIndex,
							 prGlueInfo->prAdapter->rWlanInfo.rCurrBssId.arMacAddress);
			if (prStaRec)
				noise = prStaRec->noise_avg - 127;

			pos += scnprintf(buf + pos, u4TotalLen - pos, "RSSI = %d\n", i4Rssi);

			pos += scnprintf(buf + pos, u4TotalLen - pos, "P2P GO RSSI =\n");

			pos += scnprintf(buf + pos, u4TotalLen - pos, "SNR-A =\n");

			pos += scnprintf(buf + pos, u4TotalLen - pos, "SNR-B (if available) =\n");

			pos += scnprintf(buf + pos, u4TotalLen - pos, "NoiseLevel-A = %d\n", noise);

			pos += scnprintf(buf + pos, u4TotalLen - pos, "NoiseLevel-B =\n");
		}

		kalIoctl(prGlueInfo, wlanoidQueryLinkSpeed, &u4Rate,
						sizeof(u4Rate), TRUE, TRUE, TRUE, &u4BufLen);

		/* STA stats */
		if (kalIoctl(prGlueInfo, wlanoidQueryBssid,
			     &arBssid[0], sizeof(arBssid),
			     TRUE, TRUE, TRUE,
			     &u4BufLen) == WLAN_STATUS_SUCCESS) {

			prBssInfo =
				&(prGlueInfo->prAdapter->rWifiVar.arBssInfoPool[KAL_NETWORK_TYPE_AIS_INDEX]);

			kalIoctl(prGlueInfo, wlanoidQuerySsid,
				 &ssid, sizeof(ssid),
				 TRUE, TRUE, TRUE,
				 &u4BufLen);

			pos += scnprintf(buf + pos, u4TotalLen - pos, "\n[STA] connected AP MAC Address = ");

			for (i = 0; i < PARAM_MAC_ADDR_LEN; i++) {

				pos += scnprintf(buf + pos, u4TotalLen - pos, "%02x", arBssid[i]);
				if (i != PARAM_MAC_ADDR_LEN - 1) {

					pos += scnprintf(buf + pos, u4TotalLen - pos, ":");
				}
			}
			pos += scnprintf(buf + pos, u4TotalLen - pos, "\n");

			pos += scnprintf(buf + pos, u4TotalLen - pos, "PhyMode:");
			switch (prBssInfo->ucPhyTypeSet) {
			case PHY_TYPE_SET_802_11B:
				pos += scnprintf(buf + pos, u4TotalLen - pos, "802.11b\n");
				break;
			case PHY_TYPE_SET_802_11ABG:
			case PHY_TYPE_SET_802_11BG:
				pos += scnprintf(buf + pos, u4TotalLen - pos, "802.11g\n");
				break;
			case PHY_TYPE_SET_802_11A:
				pos += scnprintf(buf + pos, u4TotalLen - pos, "802.11a\n");
				break;
			case PHY_TYPE_SET_802_11ABGN:
			case PHY_TYPE_SET_802_11BGN:
			case PHY_TYPE_SET_802_11AN:
			case PHY_TYPE_SET_802_11GN:
				pos += scnprintf(buf + pos, u4TotalLen - pos, "802.11n\n");
				break;
			case PHY_TYPE_SET_802_11ABGNAC:
			case PHY_TYPE_SET_802_11ANAC:
			case PHY_TYPE_SET_802_11AC:
				pos += scnprintf(buf + pos, u4TotalLen - pos, "802.11ac\n");
				break;
			default:
				break;
			}

			pos += scnprintf(buf + pos, u4TotalLen - pos, "RSSI =\n");

			pos += scnprintf(buf + pos, u4TotalLen - pos, "Last TX Rate = %d\n", u4Rate*100);


			if (prStaRec) {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "Last RX Rate = %d\n",
					prStaRec->u2LastPhyRate * 100000);
			} else {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "Last RX Rate =\n");
			}
		} else {
			pos += scnprintf(buf + pos, u4TotalLen - pos, "\n[STA] Not connected\n");
		}

		break;
	}
	case PRIV_CMD_GET_BAND_WITH:
	{
		UINT_8 rQueryBandWith;

		rStatus = kalIoctl(prGlueInfo, wlanoidQueryBandWidth,
				   &rQueryBandWith, sizeof(rQueryBandWith),
				   TRUE, FALSE, TRUE,
				   &u4BufLen);

		if (rStatus == WLAN_STATUS_SUCCESS) {
			pos += scnprintf(buf + pos, u4TotalLen - pos,
						    "bandwidth = %d\n",rQueryBandWith);
		}
		else
			pos += scnprintf(buf + pos, u4TotalLen - pos,
						    "get bandwidth fail\n");
		break;
	}
	case PRIV_CMD_CONNSTATUS:
	{
		PARAM_MAC_ADDRESS arBssid;
		PARAM_SSID_T ssid;

		kalMemZero(arBssid, MAC_ADDR_LEN);
		rStatus = kalIoctl(prGlueInfo, wlanoidQueryBssid,
				   &arBssid[0], sizeof(arBssid),
				   TRUE, TRUE, TRUE,
				   &u4BufLen);

		if (rStatus == WLAN_STATUS_SUCCESS) {
			kalIoctl(prGlueInfo, wlanoidQuerySsid,
				 &ssid, sizeof(ssid),
				 TRUE, TRUE, TRUE,
				 &u4BufLen);

			pos += scnprintf(buf + pos, u4TotalLen - pos, "connStatus: Connected (AP: XXXXXX [");

			for (i = 0; i < PARAM_MAC_ADDR_LEN; i++) {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "%02x", arBssid[i]);
				if (i != PARAM_MAC_ADDR_LEN - 1) {
					pos += scnprintf(buf + pos, u4TotalLen - pos, ":");
				}
			}
			pos += scnprintf(buf + pos, u4TotalLen - pos, "])");
		} else {
			pos += scnprintf(buf + pos, u4TotalLen - pos, "connStatus: Not connected");
		}
		break;
	}
#if CFG_SUPPORT_EXCEPTION_STATISTICS
	case PRIV_CMD_EXCEPTION_STAT:
	{
		pos += scnprintf(buf + pos, u4TotalLen -pos,
				"TotalBeaconTimeout:%d\n",
				prGlueInfo->prAdapter->total_beacon_timeout_count);
		for (i = 0; i < BEACON_TIMEOUT_TYPE_NUM; i++) {
			if (prGlueInfo->prAdapter->beacon_timeout_count[i] > 0)
				pos += scnprintf(buf + pos, u4TotalLen - pos,
						"BeaconTimeout Reason(0x%0x):%d\n", i,
						prGlueInfo->prAdapter->beacon_timeout_count[i]);
		}
		pos += scnprintf(buf + pos, u4TotalLen - pos,
				"TotalTxDoneFail:%d\n",
				prGlueInfo->prAdapter->total_tx_done_fail_count);
		for (i = 0; i < TX_RESULT_NUM; i++) {
			if (prGlueInfo->prAdapter->tx_done_fail_count[i] > 0)
				pos += scnprintf(buf + pos, u4TotalLen - pos,
						"TxDoneFail Reason(0x%0x):%d\n", i,
						prGlueInfo->prAdapter->tx_done_fail_count[i]);
		}
		pos += scnprintf(buf + pos, u4TotalLen - pos,
				"TotalRxDeauth:%d\n",
				prGlueInfo->prAdapter->total_deauth_rx_count);
		for (i = 0; i < (REASON_CODE_BEACON_TIMEOUT + 1); i++) {
			if (prGlueInfo->prAdapter->deauth_rx_count[i] > 0)
				pos += scnprintf(buf + pos, u4TotalLen - pos,
						"RxDeauth Reason(0x%0x):%d\n", i,
						prGlueInfo->prAdapter->deauth_rx_count[i]);
		}
		pos += scnprintf(buf + pos, u4TotalLen - pos,
				"TotalScanDoneTimeout:%d\n",
				prGlueInfo->prAdapter->total_scandone_timeout_count);
		pos += scnprintf(buf + pos, u4TotalLen - pos,
				"TotalTxMgmtTimeout:%d\n",
				prGlueInfo->prAdapter->total_mgmtTX_timeout_count);
		pos += scnprintf(buf + pos, u4TotalLen - pos,
				"TotalRxMgmtTimeout:%d\n",
				prGlueInfo->prAdapter->total_mgmtRX_timeout_count);
		break;
	}
#endif

#if CFG_SHOW_NVRAM
	case PRIV_CMD_SHOW_NVRAM:
	{
		P_ADAPTER_T prAdapter = prGlueInfo->prAdapter;
		P_REG_INFO_T prRegInfo = &prGlueInfo->rRegInfo;
		TX_PWR_PARAM_T rTxPwr = prRegInfo->rTxPwr;
		P_NEW_EFUSE_MAPPING2NVRAM_T prEfuseMapping = prRegInfo->prOldEfuseMapping;

		if (!prAdapter || !prRegInfo)
			return -EFAULT;

		if (prRegInfo->au2CountryCode[0] == 0)
			pos += scnprintf(buf + pos, u4TotalLen - pos, "[Country] %x%x\n", prRegInfo->au2CountryCode[0], prRegInfo->au2CountryCode[1]);
		else
			pos += scnprintf(buf + pos, u4TotalLen - pos, "[Country] %c%c\n", prRegInfo->au2CountryCode[0], prRegInfo->au2CountryCode[1]);

		pos += scnprintf(buf + pos, u4TotalLen - pos, "[Support 5GHz]\n");

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\tSupport 5G Band:%d\n", prRegInfo->ucSupport5GBand);

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\tEnable 5G Band:%d\n", prRegInfo->ucEnable5GBand);

		pos += scnprintf(buf + pos, u4TotalLen - pos, "[Band Edge Power Used]\n");


		if (prRegInfo->fg2G4BandEdgePwrUsed == 1) {
			pos += scnprintf(buf + pos, u4TotalLen - pos, "\tBand edge power in 2G4:%x \n", prRegInfo->fg2G4BandEdgePwrUsed);

			if (!prAdapter->fgTestMode) {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tMaxPwrCCK/OFDM20/OFDM40 in 2G4:(%x,%x,%x)\n",
					prRegInfo->cBandEdgeMaxPwrCCK, prRegInfo->cBandEdgeMaxPwrOFDM20,
					prRegInfo->cBandEdgeMaxPwrOFDM40);
			} else {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tMaxPwrCCK/OFDM20/OFDM40 in 2G4:(%x,%x,%x)\n",
					MAX_TX_POWER, MAX_TX_POWER, MAX_TX_POWER);
			}
		} else {
			pos += scnprintf(buf + pos, u4TotalLen - pos, "\tBand edge power in 2G4:%x \n", prRegInfo->fg2G4BandEdgePwrUsed);
		}

		if (prEfuseMapping){
			if (prEfuseMapping->r5GBandEdgePwr.uc5GBandEdgePwrUsed == 0x1) {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tBand edge power in 5G:%x \n", prEfuseMapping->r5GBandEdgePwr.uc5GBandEdgePwrUsed);
				if (!prAdapter->fgTestMode) {
					pos += scnprintf(buf + pos, u4TotalLen - pos, "\tMaxPwrOFDM20/OFDM40/OFDM80 in 5G:(%x,%x,%x)\n",
						prEfuseMapping->r5GBandEdgePwr.c5GBandEdgeMaxPwrOFDM20,
						prEfuseMapping->r5GBandEdgePwr.c5GBandEdgeMaxPwrOFDM40,
						prEfuseMapping->r5GBandEdgePwr.c5GBandEdgeMaxPwrOFDM80);
				} else {
					pos += scnprintf(buf + pos, u4TotalLen - pos, "\tMaxPwrOFDM20/OFDM40/OFDM80 in 5G:(%x,%x,%x)\n", MAX_TX_POWER, MAX_TX_POWER, MAX_TX_POWER);
				}
			} else {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tBand edge power in 5G:%x \n", prEfuseMapping->r5GBandEdgePwr.uc5GBandEdgePwrUsed);
			}
			pos += scnprintf(buf + pos, u4TotalLen - pos, "[Channel Offset]\n");

			if (prEfuseMapping->ucChannelOffsetVaild == 0x1) {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tEnable in 2G4:%x \n", prEfuseMapping->ucChannelOffsetVaild);
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\taucChOffset:(%x,%x,%x) \n", prEfuseMapping->aucChOffset[0],
					prEfuseMapping->aucChOffset[1], prEfuseMapping->aucChOffset[2]);
			} else {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tEnable in 2G4:%x \n", prEfuseMapping->ucChannelOffsetVaild);
			}

			if (prEfuseMapping->uc5GChannelOffsetVaild == 0x1) {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tEnable in 5G:%x \n", prEfuseMapping->uc5GChannelOffsetVaild);

				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tauc5GChOffset:(%x,%x,%x,%x,%x,%x,%x,%x) \n", prEfuseMapping->auc5GChOffset[0],
					prEfuseMapping->auc5GChOffset[1], prEfuseMapping->auc5GChOffset[2],
					prEfuseMapping->auc5GChOffset[3], prEfuseMapping->auc5GChOffset[4],
					prEfuseMapping->auc5GChOffset[5], prEfuseMapping->auc5GChOffset[6],
					prEfuseMapping->auc5GChOffset[7]);
			} else {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tEnable in 5G:%x \n", prEfuseMapping->uc5GChannelOffsetVaild);
		}
		pos += scnprintf(buf + pos, u4TotalLen - pos, "\tacAllChannelOffset:(%x) \n", prEfuseMapping->acAllChannelOffset);
		}
		pos += scnprintf(buf + pos, u4TotalLen - pos, "[Power Table In Use]\n");

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\tucTxPwrValid:%d\n", prRegInfo->ucTxPwrValid);

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\t2G4Cck:(%x)\n", rTxPwr.cTxPwr2G4Cck);

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\t2G4OFDM_BPSK/QPSK/16QAM/48M/54M:(%x,%x,%x,%x,%x)\n",
			rTxPwr.cTxPwr2G4OFDM_BPSK, rTxPwr.cTxPwr2G4OFDM_QPSK, rTxPwr.cTxPwr2G4OFDM_16QAM,
			rTxPwr.cTxPwr2G4OFDM_48Mbps, rTxPwr.cTxPwr2G4OFDM_54Mbps);

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\t2G4HT20_BPSK/QPSK/16QAM/MCS5/MCS6/MCS7:(%x,%x,%x,%x,%x,%x)\n",
			rTxPwr.cTxPwr2G4HT20_BPSK, rTxPwr.cTxPwr2G4HT20_QPSK, rTxPwr.cTxPwr2G4HT20_16QAM,
			rTxPwr.cTxPwr2G4HT20_MCS5, rTxPwr.cTxPwr2G4HT20_MCS6, rTxPwr.cTxPwr2G4HT20_MCS7);

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\t2G4HT40_BPSK/QPSK/16QAM/MCS5/MCS6/MCS7:(%x,%x,%x,%x,%x,%x)\n",
			rTxPwr.cTxPwr2G4HT40_BPSK, rTxPwr.cTxPwr2G4HT40_QPSK, rTxPwr.cTxPwr2G4HT40_16QAM,
			rTxPwr.cTxPwr2G4HT40_MCS5, rTxPwr.cTxPwr2G4HT40_MCS6, rTxPwr.cTxPwr2G4HT40_MCS7);

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\t5GOFDM_BPSK/QPSK/16QAM/48M/54M:(%x,%x,%x,%x,%x)\n",
			rTxPwr.cTxPwr5GOFDM_BPSK, rTxPwr.cTxPwr5GOFDM_QPSK,	rTxPwr.cTxPwr5GOFDM_16QAM,
			rTxPwr.cTxPwr5GOFDM_48Mbps, rTxPwr.cTxPwr5GOFDM_54Mbps);

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\t5GHT20_BPSK/QPSK/16QAM/MCS5/MCS6/MCS7:(%x,%x,%x,%x,%x,%x)\n",
			rTxPwr.cTxPwr5GHT20_BPSK, rTxPwr.cTxPwr5GHT20_QPSK, rTxPwr.cTxPwr5GHT20_16QAM,
			rTxPwr.cTxPwr5GHT20_MCS5, rTxPwr.cTxPwr5GHT20_MCS6, rTxPwr.cTxPwr5GHT20_MCS7);

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\t5GHT40_BPSK/QPSK/16QAM/MCS5/MCS6/MCS7:(%x,%x,%x,%x,%x,%x)\n",
			rTxPwr.cTxPwr5GHT40_BPSK, rTxPwr.cTxPwr5GHT40_QPSK, rTxPwr.cTxPwr5GHT40_16QAM,
			rTxPwr.cTxPwr5GHT40_MCS5, rTxPwr.cTxPwr5GHT40_MCS6, rTxPwr.cTxPwr5GHT40_MCS7);

		if (prEfuseMapping) {
			if (prEfuseMapping->uc11AcTxPwrValid == 0x1) {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tuc11AcTxPwrValid:%d\n", prEfuseMapping->uc11AcTxPwrValid);
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\t11AC_BPSK/QPSK/16QAM/MCS5_MCS6/MCS7/MCS8/MCS9/VHT40/VHT80/VHT160:(%x,%x,%x,%x,%x,%x,%x,%x,%x,%x)\n",
					prEfuseMapping->r11AcTxPwr.c11AcTxPwr_BPSK, prEfuseMapping->r11AcTxPwr.c11AcTxPwr_QPSK,
					prEfuseMapping->r11AcTxPwr.c11AcTxPwr_16QAM, prEfuseMapping->r11AcTxPwr.c11AcTxPwr_MCS5_MCS6,
					prEfuseMapping->r11AcTxPwr.c11AcTxPwr_MCS7, prEfuseMapping->r11AcTxPwr.c11AcTxPwr_MCS8,
					prEfuseMapping->r11AcTxPwr.c11AcTxPwr_MCS9, prEfuseMapping->r11AcTxPwr.c11AcTxPwrVht40_OFFSET,
					prEfuseMapping->r11AcTxPwr.c11AcTxPwrVht80_OFFSET, prEfuseMapping->r11AcTxPwr.c11AcTxPwrVht160_OFFSET);
			} else {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tuc11AcTxPwrValid:%d\n", prEfuseMapping->uc11AcTxPwrValid);
			}

			if (prEfuseMapping->uc11AcTxPwrValid2G == 0x1) {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tuc11AcTxPwrValid2G:%d\n", prEfuseMapping->uc11AcTxPwrValid2G);
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\t11AC_BPSK/QPSK/16QAM/MCS5_MCS6/MCS7/MCS8/MCS9/VHT40/VHT80/VHT160:(%x,%x,%x,%x,%x,%x,%x,%x,%x,%x)\n",
					prEfuseMapping->r11AcTxPwr2G.c11AcTxPwr_BPSK, prEfuseMapping->r11AcTxPwr2G.c11AcTxPwr_QPSK,
					prEfuseMapping->r11AcTxPwr2G.c11AcTxPwr_16QAM, prEfuseMapping->r11AcTxPwr2G.c11AcTxPwr_MCS5_MCS6,
					prEfuseMapping->r11AcTxPwr2G.c11AcTxPwr_MCS7, prEfuseMapping->r11AcTxPwr2G.c11AcTxPwr_MCS8,
					prEfuseMapping->r11AcTxPwr2G.c11AcTxPwr_MCS9, prEfuseMapping->r11AcTxPwr2G.c11AcTxPwrVht40_OFFSET,
					prEfuseMapping->r11AcTxPwr2G.c11AcTxPwrVht80_OFFSET, prEfuseMapping->r11AcTxPwr2G.c11AcTxPwrVht160_OFFSET);
			} else {
				pos += scnprintf(buf + pos, u4TotalLen - pos, "\tuc11AcTxPwrValid2G:%d\n", prEfuseMapping->uc11AcTxPwrValid2G);
			}
		}
		pos += scnprintf(buf + pos, u4TotalLen - pos, "[Fixed 20M]\n");

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\tin 2G4 BW:%d\n", prRegInfo->uc2G4BwFixed20M);

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\tin 5G BW:%d\n", prRegInfo->uc5GBwFixed20M);

		pos += scnprintf(buf + pos, u4TotalLen - pos, "[Rssi Compensation]\n");

		pos += scnprintf(buf + pos, u4TotalLen - pos, "\tValid bit:%d\n", prRegInfo->ucRssiPathCompasationUsed);
		pos += scnprintf(buf + pos, u4TotalLen - pos, "\tRssi Compensation(2G4,5G):(%d,%d)\n",
			prRegInfo->rRssiPathCompasation.c2GRssiCompensation, prRegInfo->rRssiPathCompasation.c5GRssiCompensation);
		break;
	}
#endif
	default:
		DBGLOG(REQ, WARN, "get struct cmd:0x%x\n", u4SubCmd);
		break;
	}

	DBGLOG(REQ, TRACE, "%s i4BytesWritten = %d\n", __func__, pos);
	if (pos > 0) {

		if (pos > 2000)
			pos = 2000;
		prIwReqData->data.length = pos;	/* the iwpriv will use the length */

	} else if (pos == 0) {
		prIwReqData->data.length = pos;
	}
	return 0;

}				/* priv_get_string */

/*----------------------------------------------------------------------------*/
/*!
* \brief Private ioctl driver handler.
*
* \param[in] pDev Net device requested.
* \param[out] pIwReq Pointer to iwreq structure.
* \param[in] cmd Private sub-command.
*
* \retval 0 For success.
* \retval -EFAULT If copy from user space buffer fail.
* \retval -EOPNOTSUPP Parameter "cmd" not recognized.
*
*/
/*----------------------------------------------------------------------------*/
int
priv_set_driver(IN struct net_device *prNetDev,
		IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN OUT char *pcExtra)
{
	UINT_32 u4SubCmd = 0;
	UINT_16 u2Cmd = 0;

	P_GLUE_INFO_T prGlueInfo = NULL;
	INT_32 i4BytesWritten = 0;

	ASSERT(prNetDev);
	ASSERT(prIwReqData);
	if (!prNetDev || !prIwReqData) {
		DBGLOG(REQ, INFO, "priv_set_driver(): invalid param(0x%p, 0x%p)\n", prNetDev, prIwReqData);
		return -EINVAL;
	}

	u2Cmd = prIwReqInfo->cmd;
	DBGLOG(REQ, TRACE, "prIwReqInfo->cmd %u\n", u2Cmd);

	u4SubCmd = (UINT_32) prIwReqData->data.flags;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));
	ASSERT(prGlueInfo);
	if (!prGlueInfo) {
		DBGLOG(REQ, INFO, "priv_set_driver(): invalid prGlueInfo(0x%p, 0x%p)\n",
				   prNetDev, *((P_GLUE_INFO_T *) netdev_priv(prNetDev)));
		return -EINVAL;
	}

	/* trick,hack in ./net/wireless/wext-priv.c ioctl_private_iw_point */
	/* because the cmd number is odd (get), the input string will not be copy_to_user */

	DBGLOG(REQ, TRACE, "prIwReqData->data.length %u\n", prIwReqData->data.length);

	/* Use GET type becauase large data by iwpriv. */

	ASSERT(IW_IS_GET(u2Cmd));
	if (prIwReqData->data.length != 0) {
		if (!access_ok(VERIFY_READ, prIwReqData->data.pointer, prIwReqData->data.length)) {
			DBGLOG(REQ, INFO, "%s access_ok Read fail written = %d\n", __func__, i4BytesWritten);
			return -EFAULT;
		}
		if (copy_from_user(pcExtra, prIwReqData->data.pointer, prIwReqData->data.length)) {
			DBGLOG(REQ, INFO,
			       "%s copy_form_user fail written = %d\n", __func__, prIwReqData->data.length);
			return -EFAULT;
		}
	}

	if (pcExtra) {
		pcExtra[2000] = '\0';
		DBGLOG(REQ, INFO, "pcExtra %s\n", pcExtra);
		/* Please check max length in rIwPrivTable */
		DBGLOG(REQ, TRACE, "%s prIwReqData->data.length = %d\n", __func__, prIwReqData->data.length);
		i4BytesWritten = priv_driver_cmds(prNetDev, pcExtra, 2000 /*prIwReqData->data.length */);
		DBGLOG(REQ, TRACE, "%s i4BytesWritten = %d\n", __func__, i4BytesWritten);
	}

	DBGLOG(REQ, TRACE, "pcExtra done\n");

	if (i4BytesWritten > 0) {

		if (i4BytesWritten > 2000)
			i4BytesWritten = 2000;
		prIwReqData->data.length = i4BytesWritten;	/* the iwpriv will use the length */

	} else if (i4BytesWritten == 0) {
		prIwReqData->data.length = i4BytesWritten;
	}
#if 0
	/* trick,hack in ./net/wireless/wext-priv.c ioctl_private_iw_point */
	/* because the cmd number is even (set), the return string will not be copy_to_user */
	ASSERT(IW_IS_SET(u2Cmd));
	if (!access_ok(VERIFY_WRITE, prIwReqData->data.pointer, i4BytesWritten)) {
		DBGLOG(REQ, INFO, "%s access_ok Write fail written = %d\n", __func__, i4BytesWritten);
		return -EFAULT;
	}
	if (copy_to_user(prIwReqData->data.pointer, pcExtra, i4BytesWritten)) {
		DBGLOG(REQ, INFO, "%s copy_to_user fail written = %d\n", __func__, i4BytesWritten);
		return -EFAULT;
	}
#endif

	return 0;

}				/* priv_set_driver */

#if 0
/*----------------------------------------------------------------------------*/
/*!
* \brief This routine is called to query the radio configuration used in IBSS
*        mode and RF test mode.
*
* \param[in] prGlueInfo         Pointer to the GLUE_INFO_T structure.
* \param[out] pvQueryBuffer     Pointer to the buffer that holds the result of the query.
* \param[in] u4QueryBufferLen   The length of the query buffer.
* \param[out] pu4QueryInfoLen   If the call is successful, returns the number of
*                               bytes written into the query buffer. If the call
*                               failed due to invalid length of the query buffer,
*                               returns the amount of storage needed.
*
* \retval WLAN_STATUS_SUCCESS
* \retval WLAN_STATUS_INVALID_LENGTH
*/
/*----------------------------------------------------------------------------*/
static WLAN_STATUS
reqExtQueryConfiguration(IN P_GLUE_INFO_T prGlueInfo,
			 OUT PVOID pvQueryBuffer, IN UINT_32 u4QueryBufferLen, OUT PUINT_32 pu4QueryInfoLen)
{
	P_PARAM_802_11_CONFIG_T prQueryConfig = (P_PARAM_802_11_CONFIG_T) pvQueryBuffer;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	UINT_32 u4QueryInfoLen = 0;

	DEBUGFUNC("wlanoidQueryConfiguration");

	ASSERT(prGlueInfo);
	ASSERT(pu4QueryInfoLen);

	*pu4QueryInfoLen = sizeof(PARAM_802_11_CONFIG_T);
	if (u4QueryBufferLen < sizeof(PARAM_802_11_CONFIG_T))
		return WLAN_STATUS_INVALID_LENGTH;

	ASSERT(pvQueryBuffer);

	kalMemZero(prQueryConfig, sizeof(PARAM_802_11_CONFIG_T));

	/* Update the current radio configuration. */
	prQueryConfig->u4Length = sizeof(PARAM_802_11_CONFIG_T);

#if defined(_HIF_SDIO)
	rStatus = sdio_io_ctrl(prGlueInfo,
			       wlanoidSetBeaconInterval,
			       &prQueryConfig->u4BeaconPeriod, sizeof(UINT_32), TRUE, TRUE, &u4QueryInfoLen);
#else
	rStatus = wlanQueryInformation(prGlueInfo->prAdapter,
				       wlanoidQueryBeaconInterval,
				       &prQueryConfig->u4BeaconPeriod, sizeof(UINT_32), &u4QueryInfoLen);
#endif
	if (rStatus != WLAN_STATUS_SUCCESS)
		return rStatus;
#if defined(_HIF_SDIO)
	rStatus = sdio_io_ctrl(prGlueInfo,
			       wlanoidQueryAtimWindow,
			       &prQueryConfig->u4ATIMWindow, sizeof(UINT_32), TRUE, TRUE, &u4QueryInfoLen);
#else
	rStatus = wlanQueryInformation(prGlueInfo->prAdapter,
				       wlanoidQueryAtimWindow,
				       &prQueryConfig->u4ATIMWindow, sizeof(UINT_32), &u4QueryInfoLen);
#endif
	if (rStatus != WLAN_STATUS_SUCCESS)
		return rStatus;
#if defined(_HIF_SDIO)
	rStatus = sdio_io_ctrl(prGlueInfo,
			       wlanoidQueryFrequency,
			       &prQueryConfig->u4DSConfig, sizeof(UINT_32), TRUE, TRUE, &u4QueryInfoLen);
#else
	rStatus = wlanQueryInformation(prGlueInfo->prAdapter,
				       wlanoidQueryFrequency,
				       &prQueryConfig->u4DSConfig, sizeof(UINT_32), &u4QueryInfoLen);
#endif
	if (rStatus != WLAN_STATUS_SUCCESS)
		return rStatus;

	prQueryConfig->rFHConfig.u4Length = sizeof(PARAM_802_11_CONFIG_FH_T);

	return rStatus;

}				/* end of reqExtQueryConfiguration() */

/*----------------------------------------------------------------------------*/
/*!
* \brief This routine is called to set the radio configuration used in IBSS
*        mode.
*
* \param[in] prGlueInfo     Pointer to the GLUE_INFO_T structure.
* \param[in] pvSetBuffer    A pointer to the buffer that holds the data to be set.
* \param[in] u4SetBufferLen The length of the set buffer.
* \param[out] pu4SetInfoLen If the call is successful, returns the number of
*                           bytes read from the set buffer. If the call failed
*                           due to invalid length of the set buffer, returns
*                           the amount of storage needed.
*
* \retval WLAN_STATUS_SUCCESS
* \retval WLAN_STATUS_INVALID_LENGTH
* \retval WLAN_STATUS_NOT_ACCEPTED
*/
/*----------------------------------------------------------------------------*/
static WLAN_STATUS
reqExtSetConfiguration(IN P_GLUE_INFO_T prGlueInfo,
		       IN PVOID pvSetBuffer, IN UINT_32 u4SetBufferLen, OUT PUINT_32 pu4SetInfoLen)
{
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	P_PARAM_802_11_CONFIG_T prNewConfig = (P_PARAM_802_11_CONFIG_T) pvSetBuffer;
	UINT_32 u4SetInfoLen = 0;

	DEBUGFUNC("wlanoidSetConfiguration");

	ASSERT(prGlueInfo);
	ASSERT(pu4SetInfoLen);

	*pu4SetInfoLen = sizeof(PARAM_802_11_CONFIG_T);

	if (u4SetBufferLen < *pu4SetInfoLen)
		return WLAN_STATUS_INVALID_LENGTH;

	/* OID_802_11_CONFIGURATION. If associated, NOT_ACCEPTED shall be returned. */
	if (prGlueInfo->eParamMediaStateIndicated == PARAM_MEDIA_STATE_CONNECTED)
		return WLAN_STATUS_NOT_ACCEPTED;

	ASSERT(pvSetBuffer);

#if defined(_HIF_SDIO)
	rStatus = sdio_io_ctrl(prGlueInfo,
			       wlanoidSetBeaconInterval,
			       &prNewConfig->u4BeaconPeriod, sizeof(UINT_32), FALSE, TRUE, &u4SetInfoLen);
#else
	rStatus = wlanSetInformation(prGlueInfo->prAdapter,
				     wlanoidSetBeaconInterval,
				     &prNewConfig->u4BeaconPeriod, sizeof(UINT_32), &u4SetInfoLen);
#endif
	if (rStatus != WLAN_STATUS_SUCCESS)
		return rStatus;
#if defined(_HIF_SDIO)
	rStatus = sdio_io_ctrl(prGlueInfo,
			       wlanoidSetAtimWindow,
			       &prNewConfig->u4ATIMWindow, sizeof(UINT_32), FALSE, TRUE, &u4SetInfoLen);
#else
	rStatus = wlanSetInformation(prGlueInfo->prAdapter,
				     wlanoidSetAtimWindow, &prNewConfig->u4ATIMWindow, sizeof(UINT_32), &u4SetInfoLen);
#endif
	if (rStatus != WLAN_STATUS_SUCCESS)
		return rStatus;
#if defined(_HIF_SDIO)
	rStatus = sdio_io_ctrl(prGlueInfo,
			       wlanoidSetFrequency,
			       &prNewConfig->u4DSConfig, sizeof(UINT_32), FALSE, TRUE, &u4SetInfoLen);
#else
	rStatus = wlanSetInformation(prGlueInfo->prAdapter,
				     wlanoidSetFrequency, &prNewConfig->u4DSConfig, sizeof(UINT_32), &u4SetInfoLen);
#endif

	if (rStatus != WLAN_STATUS_SUCCESS)
		return rStatus;

	return rStatus;

}				/* end of reqExtSetConfiguration() */

#endif
/*----------------------------------------------------------------------------*/
/*!
* \brief This routine is called to set beacon detection function enable/disable state
*        This is mainly designed for usage under BT inquiry state (disable function).
*
* \param[in] pvAdapter Pointer to the Adapter structure
* \param[in] pvSetBuffer A pointer to the buffer that holds the data to be set
* \param[in] u4SetBufferLen The length of the set buffer
* \param[out] pu4SetInfoLen If the call is successful, returns the number of
*   bytes read from the set buffer. If the call failed due to invalid length of
*   the set buffer, returns the amount of storage needed.
*
* \retval WLAN_STATUS_SUCCESS
* \retval WLAN_STATUS_INVALID_DATA If new setting value is wrong.
* \retval WLAN_STATUS_INVALID_LENGTH
*
*/
/*----------------------------------------------------------------------------*/
static WLAN_STATUS
reqExtSetAcpiDevicePowerState(IN P_GLUE_INFO_T prGlueInfo,
			      IN PVOID pvSetBuffer, IN UINT_32 u4SetBufferLen, OUT PUINT_32 pu4SetInfoLen)
{
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;

	ASSERT(prGlueInfo);
	ASSERT(pvSetBuffer);
	ASSERT(pu4SetInfoLen);

	/* WIFI is enabled, when ACPI is D0 (ParamDeviceStateD0 = 1). And vice versa */

	/* rStatus = wlanSetInformation(prGlueInfo->prAdapter, */
	/* wlanoidSetAcpiDevicePowerState, */
	/* pvSetBuffer, */
	/* u4SetBufferLen, */
	/* pu4SetInfoLen); */
	return rStatus;
}

#define CMD_START				"START"
#define CMD_STOP				"STOP"
#define CMD_SCAN_ACTIVE			"SCAN-ACTIVE"
#define CMD_SCAN_PASSIVE		"SCAN-PASSIVE"
#define CMD_RSSI				"RSSI"
#define CMD_LINKSPEED			"LINKSPEED"
#define CMD_RXFILTER_START		"RXFILTER-START"
#define CMD_RXFILTER_STOP		"RXFILTER-STOP"
#define CMD_RXFILTER_ADD		"RXFILTER-ADD"
#define CMD_RXFILTER_REMOVE		"RXFILTER-REMOVE"
#define CMD_BTCOEXSCAN_START	"BTCOEXSCAN-START"
#define CMD_BTCOEXSCAN_STOP		"BTCOEXSCAN-STOP"
#define CMD_BTCOEXMODE			"BTCOEXMODE"
#define CMD_SETSUSPENDOPT		"SETSUSPENDOPT"
#define CMD_SETSUSPENDMODE		"SETSUSPENDMODE"
#define CMD_P2P_DEV_ADDR		"P2P_DEV_ADDR"
#define CMD_SETFWPATH			"SETFWPATH"
#define CMD_SETBAND				"SETBAND"
#define CMD_GETBAND				"GETBAND"
#define CMD_SET_TXPOWER			"SET_TXPOWER"
#define CMD_COUNTRY				"COUNTRY"
#define CMD_P2P_SET_NOA			"P2P_SET_NOA"
#define CMD_P2P_GET_NOA			"P2P_GET_NOA"
#define CMD_P2P_SET_PS			"P2P_SET_PS"
#define CMD_SET_AP_WPS_P2P_IE	"SET_AP_WPS_P2P_IE"
#define CMD_SETROAMMODE	"SETROAMMODE"
#define CMD_MIRACAST		"MIRACAST"
#if CFG_SUPPORT_ANT_DIVERSITY
#define CMD_ANT_SWITCH_TEST     "ANT_SWITCH_TEST"
#define CMD_NEW_ORIENTATION     "NEW_ORIENTATION"
#endif
#define CMD_PNOSSIDCLR_SET	"PNOSSIDCLR"
#define CMD_PNOSETUP_SET	"PNOSETUP "
#define CMD_PNOENABLE_SET	"PNOFORCE"
#define CMD_PNODEBUG_SET	"PNODEBUG"
#define CMD_WLS_BATCHING	"WLS_BATCHING"

#define CMD_OKC_SET_PMK		"SET_PMK"
#define CMD_OKC_ENABLE		"OKC_ENABLE"

#define CMD_SETMONITOR		"MONITOR"
#define CMD_SETBUFMODE		"BUFFER_MODE"

#if CFG_SUPPORT_QA_TOOL
#define CMD_GET_RX_STATISTICS	"GET_RX_STATISTICS"
#endif

/* miracast related definition */
#define MIRACAST_MODE_OFF	0
#define MIRACAST_MODE_SOURCE	1
#define MIRACAST_MODE_SINK	2

#ifndef MIRACAST_AMPDU_SIZE
#define MIRACAST_AMPDU_SIZE	8
#endif

#ifndef MIRACAST_MCHAN_ALGO
#define MIRACAST_MCHAN_ALGO     1
#endif

#ifndef MIRACAST_MCHAN_BW
#define MIRACAST_MCHAN_BW       25
#endif

#define	CMD_BAND_AUTO	0
#define	CMD_BAND_5G		1
#define	CMD_BAND_2G		2
#define	CMD_BAND_ALL	3

/* Mediatek private command */
#define CMD_SET_SW_CTRL	        "SET_SW_CTRL"
#define CMD_GET_SW_CTRL         "GET_SW_CTRL"
#define CMD_SET_CFG             "SET_CFG"
#define CMD_GET_CFG             "GET_CFG"
#define CMD_SET_CHIP            "SET_CHIP"
#define CMD_GET_CHIP            "GET_CHIP"
#define CMD_SET_DBG_LEVEL       "SET_DBG_LEVEL"
#define CMD_GET_DBG_LEVEL       "GET_DBG_LEVEL"
#define CMD_ADD_TS		"addts"
#define CMD_DEL_TS		"delts"
#define CMD_DUMP_TS		"dumpts"
#define CMD_RM_IT		"RM-IT"
#define CMD_DUMP_UAPSD		"dumpuapsd"
#define CMD_FW_EVENT		"FW-EVENT "
#define CMD_FW_PARAM            "set_fw_param "
#define PRIV_CMD_SIZE 512

static UINT_8 g_ucMiracastMode = MIRACAST_MODE_OFF;

typedef struct cmd_tlv {
	char prefix;
	char version;
	char subver;
	char reserved;
} cmd_tlv_t;

typedef struct priv_driver_cmd_s {
	char buf[PRIV_CMD_SIZE];
	int used_len;
	int total_len;
} priv_driver_cmd_t;


static int priv_driver_get_sw_ctrl(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	UINT_32 u4BufLen = 0;
	INT_32 i4BytesWritten = 0;
	INT_32 i4Argc = 0;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX];
	INT_32 u4Ret = 0;

	PARAM_CUSTOM_SW_CTRL_STRUCT_T rSwCtrlInfo;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	if (i4Argc >= 2) {
		/* rSwCtrlInfo.u4Id = kalStrtoul(apcArgv[1], NULL, 0); */
		rSwCtrlInfo.u4Data = 0;
		u4Ret = kalkStrtou32(apcArgv[1], 0, &(rSwCtrlInfo.u4Id));
		if (u4Ret)
			DBGLOG(REQ, LOUD, "parse rSwCtrlInfo error u4Ret=%d\n", u4Ret);

		DBGLOG(REQ, LOUD, "id is %x\n", rSwCtrlInfo.u4Id);

		rStatus = kalIoctl(prGlueInfo,
				   wlanoidQuerySwCtrlRead,
				   &rSwCtrlInfo, sizeof(rSwCtrlInfo), TRUE, TRUE, TRUE, &u4BufLen);

		DBGLOG(REQ, LOUD, "rStatus %u\n", rStatus);
		if (rStatus != WLAN_STATUS_SUCCESS)
			return -1;

		i4BytesWritten = snprintf(pcCommand, i4TotalLen, "0x%08x", (unsigned int)rSwCtrlInfo.u4Data);
		DBGLOG(REQ, INFO, "%s: command result is %s\n", __func__, pcCommand);
	}

	return i4BytesWritten;

}				/* priv_driver_get_sw_ctrl */

int priv_driver_set_sw_ctrl(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	UINT_32 u4BufLen = 0;
	INT_32 i4BytesWritten = 0;
	INT_32 i4Argc = 0;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX] = { 0 };
	INT_32 u4Ret = 0;

	PARAM_CUSTOM_SW_CTRL_STRUCT_T rSwCtrlInfo;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	if (i4Argc >= 3) {
		/*
		 * rSwCtrlInfo.u4Id = kalStrtoul(apcArgv[1], NULL, 0);
		 * rSwCtrlInfo.u4Data = kalStrtoul(apcArgv[2], NULL, 0);
		 */
		u4Ret = kalkStrtou32(apcArgv[1], 0, &(rSwCtrlInfo.u4Id));
		if (u4Ret)
			DBGLOG(REQ, LOUD, "parse rSwCtrlInfo error u4Ret=%d\n", u4Ret);
		u4Ret = kalkStrtou32(apcArgv[2], 0, &(rSwCtrlInfo.u4Data));
		if (u4Ret)
			DBGLOG(REQ, LOUD, "parse rSwCtrlInfo error u4Ret=%d\n", u4Ret);

		rStatus = kalIoctl(prGlueInfo,
				   wlanoidSetSwCtrlWrite,
				   &rSwCtrlInfo, sizeof(rSwCtrlInfo), FALSE, FALSE, TRUE, &u4BufLen);

		if (rStatus != WLAN_STATUS_SUCCESS)
			return -1;

	}

	return i4BytesWritten;

}				/* priv_driver_set_sw_ctrl */

#if CFG_SUPPORT_QA_TOOL
#if CFG_SUPPORT_BUFFER_MODE
static int priv_driver_set_efuse_buffer_mode(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	P_ADAPTER_T prAdapter = NULL;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	UINT_32 u4BufLen = 0;
	INT_32 i4Argc = 0;
	INT_32 i4BytesWritten = 0;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX];
	PARAM_CUSTOM_EFUSE_BUFFER_MODE_T rSetEfuseBufModeInfo;
#if (CFG_EFUSE_BUFFER_MODE_DELAY_CAL == 0)
	int i = 0;
#endif
	PUINT_8 pucConfigBuf;
	UINT_32 u4ConfigReadLen;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;

	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));
	prAdapter = prGlueInfo->prAdapter;

	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);

	pucConfigBuf = (PUINT_8) kalMemAlloc(2048, VIR_MEM_TYPE);
	kalMemZero(pucConfigBuf, 2048);
	u4ConfigReadLen = 0;

	if (pucConfigBuf) {
		if (kalReadToFile("/MT6632_eFuse_usage_table.xlsm.bin", pucConfigBuf, 2048, &u4ConfigReadLen) == 0) {
			/* ToDo:: Nothing */
		} else {
			DBGLOG(INIT, INFO, "can't find file\n");
			return -1;
		}

		kalMemFree(pucConfigBuf, VIR_MEM_TYPE, 2048);
	}
	/* pucConfigBuf */
	kalMemZero(&rSetEfuseBufModeInfo, sizeof(PARAM_CUSTOM_EFUSE_BUFFER_MODE_T));

	rSetEfuseBufModeInfo.ucSourceMode = 1;
	rSetEfuseBufModeInfo.ucCount = (UINT_8)EFUSE_CONTENT_SIZE;

#if (CFG_EFUSE_BUFFER_MODE_DELAY_CAL == 0)
	for (i = 0; i < EFUSE_CONTENT_SIZE; i++) {
		rSetEfuseBufModeInfo.aBinContent[i].u2Addr = i;
		rSetEfuseBufModeInfo.aBinContent[i].ucValue = *(pucConfigBuf + i);
	}

	for (i = 0; i < 20; i++)
		DBGLOG(INIT, INFO, "%x\n", rSetEfuseBufModeInfo.aBinContent[i].ucValue);
#endif

	rStatus = kalIoctl(prGlueInfo,
			   wlanoidSetEfusBufferMode,
			   &rSetEfuseBufModeInfo, sizeof(PARAM_CUSTOM_EFUSE_BUFFER_MODE_T), FALSE, FALSE, TRUE,
			   &u4BufLen);

	i4BytesWritten =
	    snprintf(pcCommand, i4TotalLen, "set buffer mode %s",
		     (rStatus == WLAN_STATUS_SUCCESS) ? "success" : "fail");

	return i4BytesWritten;
}
#endif /* CFG_SUPPORT_BUFFER_MODE */

static int priv_driver_get_rx_statistics(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	UINT_32 u4BufLen = 0;
	INT_32 i4BytesWritten = 0;
	INT_32 i4Argc = 0;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX];
	INT_32 u4Ret = 0;
	PARAM_CUSTOM_ACCESS_RX_STAT rRxStatisticsTest;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	DBGLOG(INIT, ERROR, " priv_driver_get_rx_statistics\n");

	if (i4Argc >= 2) {
		u4Ret = kalkStrtou32(apcArgv[1], 0, &(rRxStatisticsTest.u4SeqNum));
		rRxStatisticsTest.u4TotalNum = sizeof(PARAM_RX_STAT_T) / 4;

		rStatus = kalIoctl(prGlueInfo,
				   wlanoidQueryRxStatistics,
				   &rRxStatisticsTest, sizeof(rRxStatisticsTest), TRUE, TRUE, TRUE, &u4BufLen);

		DBGLOG(REQ, LOUD, "rStatus %u\n", rStatus);
		if (rStatus != WLAN_STATUS_SUCCESS)
			return -1;
	}

	return i4BytesWritten;
}
#endif /* CFG_SUPPORT_QA_TOOL */

int priv_driver_set_cfg(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	P_GLUE_INFO_T prGlueInfo = NULL;
	UINT_32 u4BufLen = 0;
	INT_32 i4BytesWritten = 0;
	INT_32 i4Argc = 0;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX] = { 0 };

	PARAM_CUSTOM_KEY_CFG_STRUCT_T rKeyCfgInfo;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	kalMemZero(&rKeyCfgInfo, sizeof(rKeyCfgInfo));

	if (i4Argc >= 3) {
		/* wlanCfgSet(prAdapter, apcArgv[1], apcArgv[2], 0); */
		/* Call by  wlanoid because the set_cfg will trigger callback */
		kalStrnCpy(rKeyCfgInfo.aucKey, apcArgv[1], WLAN_CFG_KEY_LEN_MAX - 1);
		kalStrnCpy(rKeyCfgInfo.aucValue, apcArgv[2], WLAN_CFG_VALUE_LEN_MAX - 1);
		rStatus = kalIoctl(prGlueInfo,
				   wlanoidSetKeyCfg, &rKeyCfgInfo, sizeof(rKeyCfgInfo), FALSE, FALSE, TRUE, &u4BufLen);

		if (rStatus != WLAN_STATUS_SUCCESS)
			return -1;
	}

	return i4BytesWritten;

}				/* priv_driver_set_cfg  */

int priv_driver_get_cfg(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	P_ADAPTER_T prAdapter = NULL;
	INT_32 i4BytesWritten = 0;
	INT_32 i4Argc = 0;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX] = { 0 };
	CHAR aucValue[WLAN_CFG_VALUE_LEN_MAX];

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;

	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);
	prAdapter = prGlueInfo->prAdapter;

	if (i4Argc >= 2) {
		/* by wlanoid ? */
		if (wlanCfgGet(prAdapter, apcArgv[1], aucValue, "", 0) == WLAN_STATUS_SUCCESS) {
			kalStrnCpy(pcCommand, aucValue, WLAN_CFG_VALUE_LEN_MAX);
			i4BytesWritten = kalStrnLen(pcCommand, WLAN_CFG_VALUE_LEN_MAX);
		}
	}

	return i4BytesWritten;

}				/* priv_driver_get_cfg  */

int priv_driver_set_chip_config(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	P_GLUE_INFO_T prGlueInfo = NULL;
	UINT_32 u4BufLen = 0;
	INT_32 i4BytesWritten = 0;
	UINT_32 u4CmdLen = 0;
	UINT_32 u4PrefixLen = 0;
	/* INT_32 i4Argc = 0; */
	/* PCHAR  apcArgv[WLAN_CFG_ARGV_MAX] = {0}; */

	PARAM_CUSTOM_CHIP_CONFIG_STRUCT_T rChipConfigInfo;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	/* wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv); */
	/* DBGLOG(REQ, LOUD,("argc is %i\n",i4Argc)); */
	u4CmdLen = kalStrnLen(pcCommand, i4TotalLen);
	u4PrefixLen = kalStrLen(CMD_SET_CHIP) + 1 /*space */;

	kalMemZero(&rChipConfigInfo, sizeof(rChipConfigInfo));

	/* if(i4Argc >= 2) { */
	if (u4CmdLen > u4PrefixLen) {

		rChipConfigInfo.ucType = CHIP_CONFIG_TYPE_WO_RESPONSE;
		/* rChipConfigInfo.u2MsgSize = kalStrnLen(apcArgv[1],CHIP_CONFIG_RESP_SIZE); */
		rChipConfigInfo.u2MsgSize = u4CmdLen - u4PrefixLen;
		/* kalStrnCpy(rChipConfigInfo.aucCmd,apcArgv[1],CHIP_CONFIG_RESP_SIZE); */
		kalStrnCpy(rChipConfigInfo.aucCmd, pcCommand + u4PrefixLen, CHIP_CONFIG_RESP_SIZE - 1);

		rStatus = kalIoctl(prGlueInfo,
				   wlanoidSetChipConfig,
				   &rChipConfigInfo, sizeof(rChipConfigInfo), FALSE, FALSE, TRUE, &u4BufLen);

		if (rStatus != WLAN_STATUS_SUCCESS) {
			DBGLOG(REQ, INFO, "%s: kalIoctl ret=%d\n", __func__, rStatus);
			i4BytesWritten = -1;
		}
	}

	return i4BytesWritten;

}				/* priv_driver_set_chip_config  */

int priv_driver_get_chip_config(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	P_GLUE_INFO_T prGlueInfo = NULL;
	INT_32 i4BytesWritten = 0;
	UINT_32 u4BufLen = 0;
	UINT_32 u2MsgSize = 0;
	UINT_32 u4CmdLen = 0;
	UINT_32 u4PrefixLen = 0;
	/* INT_32 i4Argc = 0; */
	/* PCHAR  apcArgv[WLAN_CFG_ARGV_MAX]; */

	PARAM_CUSTOM_CHIP_CONFIG_STRUCT_T rChipConfigInfo;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;

	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	/* wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv); */
	/* DBGLOG(REQ, LOUD,("argc is %i\n",i4Argc)); */

	u4CmdLen = kalStrnLen(pcCommand, i4TotalLen);
	u4PrefixLen = kalStrLen(CMD_GET_CHIP) + 1 /*space */;

	/* if(i4Argc >= 2) { */
	if (u4CmdLen > u4PrefixLen) {
		rChipConfigInfo.ucRespType = CHIP_CONFIG_TYPE_WO_RESPONSE;
		rChipConfigInfo.ucType = CHIP_CONFIG_TYPE_ASCII;
		/* rChipConfigInfo.u2MsgSize = kalStrnLen(apcArgv[1],CHIP_CONFIG_RESP_SIZE); */
		rChipConfigInfo.u2MsgSize = u4CmdLen - u4PrefixLen;
		/* kalStrnCpy(rChipConfigInfo.aucCmd,apcArgv[1],CHIP_CONFIG_RESP_SIZE); */
		kalStrnCpy(rChipConfigInfo.aucCmd, pcCommand + u4PrefixLen, CHIP_CONFIG_RESP_SIZE - 1);
		rStatus = kalIoctl(prGlueInfo,
				   wlanoidQueryChipConfig,
				   &rChipConfigInfo, sizeof(rChipConfigInfo), TRUE, TRUE, TRUE, &u4BufLen);

		if (rStatus != WLAN_STATUS_SUCCESS) {
			DBGLOG(REQ, INFO, "%s: kalIoctl ret=%d\n", __func__, rStatus);
			return -1;
		}

		/* Check respType */
		u2MsgSize = rChipConfigInfo.u2MsgSize;
		DBGLOG(REQ, INFO, "%s: RespTyep  %u\n", __func__, rChipConfigInfo.ucRespType);
		DBGLOG(REQ, INFO, "%s: u2MsgSize %u\n", __func__, rChipConfigInfo.u2MsgSize);

		if (u2MsgSize > sizeof(rChipConfigInfo.aucCmd)) {
			DBGLOG(REQ, INFO, "%s: u2MsgSize error ret=%u\n", __func__, rChipConfigInfo.u2MsgSize);
			return -1;
		}

		if (u2MsgSize > 0) {

			if (rChipConfigInfo.ucRespType == CHIP_CONFIG_TYPE_ASCII) {
				i4BytesWritten =
				    snprintf(pcCommand + i4BytesWritten, i4TotalLen, "%s", rChipConfigInfo.aucCmd);
			} else {
				UINT_32 u4Length;
				UINT_32 u4Line;

				if (rChipConfigInfo.ucRespType == CHIP_CONFIG_TYPE_MEM8) {
					PUINT_8 pucStartAddr = NULL;

					pucStartAddr = (PUINT_8) rChipConfigInfo.aucCmd;
					/* align 16 bytes because one print line is 16 bytes */
					u4Length = (((u2MsgSize + 15) >> 4)) << 4;
					u4Line = 0;
					priv_driver_get_chip_config_16(pucStartAddr, u4Length, u4Line, i4TotalLen,
								       i4BytesWritten, pcCommand);
				} else {
					PUINT_32 pu4StartAddr = NULL;

					pu4StartAddr = (PUINT_32) rChipConfigInfo.aucCmd;
					/* align 16 bytes because one print line is 16 bytes */
					u4Length = (((u2MsgSize + 15) >> 4)) << 4;
					u4Line = 0;

					if (IS_ALIGN_4((ULONG) pu4StartAddr)) {
						priv_driver_get_chip_config_4(pu4StartAddr, u4Length, u4Line,
									      i4TotalLen, i4BytesWritten, pcCommand);
					} else {
						DBGLOG(REQ, INFO,
						       "%s: rChipConfigInfo.aucCmd is not 4 bytes alignment %p\n",
							__func__, rChipConfigInfo.aucCmd);
					}
				}	/* ChipConfigInfo.ucRespType */
			}
		}
		/* u2MsgSize > 0 */
		DBGLOG(REQ, INFO, "%s: command result is %s\n", __func__, pcCommand);
	}
	/* i4Argc */
	return i4BytesWritten;

}				/* priv_driver_get_chip_config  */

static void
priv_driver_get_chip_config_16(PUINT_8 pucStartAddr, UINT_32 u4Length, UINT_32 u4Line, int i4TotalLen,
			       INT_32 i4BytesWritten, char *pcCommand)
{

	while (u4Length >= 16) {
		if (i4TotalLen > i4BytesWritten) {
			i4BytesWritten +=
			    snprintf(pcCommand + i4BytesWritten,
				     i4TotalLen - i4BytesWritten,
			       "%04x %02x %02x %02x %02x %02x %02x %02x %02x-%02x %02x %02x %02x %02x %02x %02x %02x\n",
				     u4Line, pucStartAddr[0],
				     pucStartAddr[1],
				     pucStartAddr[2],
				     pucStartAddr[3],
				     pucStartAddr[4],
				     pucStartAddr[5],
				     pucStartAddr[6],
				     pucStartAddr[7],
				     pucStartAddr[8],
				     pucStartAddr[9],
				     pucStartAddr[10],
				     pucStartAddr[11],
				     pucStartAddr[12], pucStartAddr[13], pucStartAddr[14], pucStartAddr[15]);
		}

		pucStartAddr += 16;
		u4Length -= 16;
		u4Line += 16;
	}			/* u4Length */
}

static void
priv_driver_get_chip_config_4(PUINT_32 pu4StartAddr, UINT_32 u4Length, UINT_32 u4Line, int i4TotalLen,
			      INT_32 i4BytesWritten, char *pcCommand)
{
	while (u4Length >= 16) {
		if (i4TotalLen > i4BytesWritten) {
			i4BytesWritten +=
			    snprintf(pcCommand +
				     i4BytesWritten,
				     i4TotalLen -
				     i4BytesWritten,
				     "%04x %08x %08x %08x %08x\n",
				     u4Line, pu4StartAddr[0], pu4StartAddr[1], pu4StartAddr[2], pu4StartAddr[3]);
		}

		pu4StartAddr += 4;
		u4Length -= 16;
		u4Line += 4;
	}			/* u4Length */
}

int priv_driver_get_linkspeed(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	UINT_32 u4BufLen = 0;
	UINT_32 u4Rate = 0;
	UINT_32 u4LinkSpeed = 0;
	INT_32 i4BytesWritten = 0;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	if (!netif_carrier_ok(prNetDev))
		return -1;

	rStatus = kalIoctl(prGlueInfo, wlanoidQueryLinkSpeed, &u4Rate, sizeof(u4Rate), TRUE, TRUE, TRUE, &u4BufLen);

	if (rStatus != WLAN_STATUS_SUCCESS)
		return -1;

	u4LinkSpeed = u4Rate * 100;
	i4BytesWritten = snprintf(pcCommand, i4TotalLen, "LinkSpeed %u", (unsigned int)u4LinkSpeed);
	DBGLOG(REQ, INFO, "%s: command result is %s\n", __func__, pcCommand);
	return i4BytesWritten;

}				/* priv_driver_get_linkspeed */

int priv_driver_set_band(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	P_ADAPTER_T prAdapter = NULL;
	P_GLUE_INFO_T prGlueInfo = NULL;
	INT_32 i4Argc = 0;
	UINT_32 ucBand = 0;
	UINT_8 ucBssIndex;
	ENUM_BAND_T eBand = BAND_NULL;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX];
	INT_32 u4Ret = 0;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	prAdapter = prGlueInfo->prAdapter;
	if (i4Argc >= 2) {
		/* ucBand = kalStrtoul(apcArgv[1], NULL, 0); */
		u4Ret = kalkStrtou32(apcArgv[1], 0, &ucBand);
		if (u4Ret)
			DBGLOG(REQ, LOUD, "parse ucBand error u4Ret=%d\n", u4Ret);

		ucBssIndex = wlanGetAisBssIndex(prGlueInfo->prAdapter);
		if (ucBssIndex >= BSS_INFO_NUM)
			return -1;
		eBand = BAND_NULL;
		if (ucBand == CMD_BAND_5G)
			eBand = BAND_5G;
		else if (ucBand == CMD_BAND_2G)
			eBand = BAND_2G4;
		prAdapter->aePreferBand[ucBssIndex] = eBand;
		/* XXX call wlanSetPreferBandByNetwork directly in different thread */
		/* wlanSetPreferBandByNetwork (prAdapter, eBand, ucBssIndex); */
	}

	return 0;
}

int priv_driver_set_txpower(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	UINT_32 u4BufLen = 0;
	INT_32 i4Argc = 0;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX];
	P_SET_TXPWR_CTRL_T prTxpwr;
	UINT_16 i;
	INT_32 u4Ret = 0;
	INT_32 ai4Setting[4];

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	prTxpwr = &prGlueInfo->rTxPwr;

	if (i4Argc >= 3 && i4Argc <= 5) {
		for (i = 0; i < (i4Argc - 1); i++) {
			/* ai4Setting[i] = kalStrtol(apcArgv[i + 1], NULL, 0); */
			u4Ret = kalkStrtos32(apcArgv[i + 1], 0, &(ai4Setting[i]));
			if (u4Ret)
				DBGLOG(REQ, LOUD, "parse apcArgv error u4Ret=%d\n", u4Ret);
		}
	} else {
		DBGLOG(REQ, INFO, "set_txpower wrong argc : %d\n", i4Argc);
		return -1;
	}

	/*
	 * ai4Setting[0]
	 * 0 : Set TX power offset for specific network
	 * 1 : Set TX power offset policy when multiple networks are in the same channel
	 * 2 : Set TX power limit for specific channel in 2.4GHz band
	 * 3 : Set TX power limit of specific sub-band in 5GHz band
	 * 4 : Enable or reset setting
	 */
	if (ai4Setting[0] == 0 && (i4Argc - 1) == 4 /* argc num */) {
		/* ai4Setting[1] : 0 (All networks), 1 (legacy STA), 2 (Hotspot AP), 3 (P2P), 4 (BT over Wi-Fi) */
		/* ai4Setting[2] : 0 (All bands),1 (2.4G), 2 (5G) */
		/* ai4Setting[3] : -30 ~ 20 in unit of 0.5dBm (default: 0) */
		if (ai4Setting[1] == 1 || ai4Setting[1] == 0) {
			if (ai4Setting[2] == 0 || ai4Setting[2] == 1)
				prTxpwr->c2GLegacyStaPwrOffset = ai4Setting[3];
			if (ai4Setting[2] == 0 || ai4Setting[2] == 2)
				prTxpwr->c5GLegacyStaPwrOffset = ai4Setting[3];
		}
		if (ai4Setting[1] == 2 || ai4Setting[1] == 0) {
			if (ai4Setting[2] == 0 || ai4Setting[2] == 1)
				prTxpwr->c2GHotspotPwrOffset = ai4Setting[3];
			if (ai4Setting[2] == 0 || ai4Setting[2] == 2)
				prTxpwr->c5GHotspotPwrOffset = ai4Setting[3];
		}
		if (ai4Setting[1] == 3 || ai4Setting[1] == 0) {
			if (ai4Setting[2] == 0 || ai4Setting[2] == 1)
				prTxpwr->c2GP2pPwrOffset = ai4Setting[3];
			if (ai4Setting[2] == 0 || ai4Setting[2] == 2)
				prTxpwr->c5GP2pPwrOffset = ai4Setting[3];
		}
		if (ai4Setting[1] == 4 || ai4Setting[1] == 0) {
			if (ai4Setting[2] == 0 || ai4Setting[2] == 1)
				prTxpwr->c2GBowPwrOffset = ai4Setting[3];
			if (ai4Setting[2] == 0 || ai4Setting[2] == 2)
				prTxpwr->c5GBowPwrOffset = ai4Setting[3];
		}
	} else if (ai4Setting[0] == 1 && (i4Argc - 1) == 2) {
		/* ai4Setting[1] : 0 (highest power is used) (default), 1 (lowest power is used) */
		prTxpwr->ucConcurrencePolicy = ai4Setting[1];
	} else if (ai4Setting[0] == 2 && (i4Argc - 1) == 3) {
		/* ai4Setting[1] : 0 (all channels in 2.4G), 1~14 */
		/* ai4Setting[2] : 10 ~ 46 in unit of 0.5dBm (default: 46) */
		if (ai4Setting[1] == 0) {
			for (i = 0; i < 14; i++)
				prTxpwr->acTxPwrLimit2G[i] = ai4Setting[2];
		} else if (ai4Setting[1] <= 14)
			prTxpwr->acTxPwrLimit2G[ai4Setting[1] - 1] = ai4Setting[2];
	} else if (ai4Setting[0] == 3 && (i4Argc - 1) == 3) {
		/*
		 * ai4Setting[1] : 0 (all sub-bands in 5G),
		 * 1 (5000 ~ 5250MHz),
		 * 2 (5255 ~ 5350MHz),
		 * 3 (5355 ~ 5725MHz),
		 * 4 (5730 ~ 5825MHz)
		 */
		/* ai4Setting[2] : 10 ~ 46 in unit of 0.5dBm (default: 46) */
		if (ai4Setting[1] == 0) {
			for (i = 0; i < 4; i++)
				prTxpwr->acTxPwrLimit5G[i] = ai4Setting[2];
		} else if (ai4Setting[1] <= 4)
			prTxpwr->acTxPwrLimit5G[ai4Setting[1] - 1] = ai4Setting[2];
	} else if (ai4Setting[0] == 4 && (i4Argc - 1) == 2) {
		/* ai4Setting[1] : 1 (enable), 0 (reset and disable) */
		if (ai4Setting[1] == 0)
			wlanDefTxPowerCfg(prGlueInfo->prAdapter);

		rStatus = kalIoctl(prGlueInfo,
				   wlanoidSetTxPower, prTxpwr, sizeof(SET_TXPWR_CTRL_T), FALSE, FALSE, TRUE, &u4BufLen);

		if (rStatus != WLAN_STATUS_SUCCESS)
			return -1;
	} else
		return -EFAULT;

	return 0;
}

int priv_driver_set_country(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{

	P_GLUE_INFO_T prGlueInfo = NULL;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	UINT_32 u4BufLen = 0;
	INT_32 i4Argc = 0;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX];
	UINT_8 aucCountry[2];

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	if (i4Argc >= 2) {
		/* command like "COUNTRY US", "COUNTRY EU" and "COUNTRY JP" */
		aucCountry[0] = apcArgv[1][0];
		aucCountry[1] = apcArgv[1][1];
		if ('X' == aucCountry[0] && 'X' == aucCountry[1])
			aucCountry[0] = aucCountry[1] = 'W';
		rStatus = kalIoctl(prGlueInfo, wlanoidSetCountryCode, &aucCountry[0], 2, FALSE, FALSE, TRUE, &u4BufLen);

		if (rStatus != WLAN_STATUS_SUCCESS)
			return -1;
			/*Indicate channel change notificaiton to wpa_supplicant via cfg80211*/
		if ('W' == aucCountry[0] && 'W' == aucCountry[1])
			aucCountry[0] = aucCountry[1] = 'X';
		wlanRegulatoryHint(&aucCountry[0]);
	}
	return 0;
}

int priv_driver_set_miracast(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{

	P_ADAPTER_T prAdapter = NULL;
	P_GLUE_INFO_T prGlueInfo = NULL;
	UINT_32 i4BytesWritten = 0;
	/* WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS; */
	/* UINT_32 u4BufLen = 0; */
	INT_32 i4Argc = 0;
	UINT_32 ucMode = 0;
	P_WFD_CFG_SETTINGS_T prWfdCfgSettings = (P_WFD_CFG_SETTINGS_T) NULL;
	P_MSG_WFD_CONFIG_SETTINGS_CHANGED_T prMsgWfdCfgUpdate = (P_MSG_WFD_CONFIG_SETTINGS_CHANGED_T) NULL;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX];
	INT_32 u4Ret = 0;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;

	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	prAdapter = prGlueInfo->prAdapter;
	if (i4Argc >= 2) {
		/* ucMode = kalStrtoul(apcArgv[1], NULL, 0); */
		u4Ret = kalkStrtou32(apcArgv[1], 0, &ucMode);
		if (u4Ret)
			DBGLOG(REQ, LOUD, "parse ucMode error u4Ret=%d\n", u4Ret);

		if (g_ucMiracastMode == (UINT_8)ucMode) {
			/* XXX: continue or skip */
			/* XXX: continue or skip */
		}

		g_ucMiracastMode = (UINT_8)ucMode;
		prMsgWfdCfgUpdate = cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(MSG_WFD_CONFIG_SETTINGS_CHANGED_T));

		if (prMsgWfdCfgUpdate != NULL) {

			prWfdCfgSettings = &(prAdapter->rWifiVar.rWfdConfigureSettings);
			prMsgWfdCfgUpdate->rMsgHdr.eMsgId = MID_MNY_P2P_WFD_CFG_UPDATE;
			prMsgWfdCfgUpdate->prWfdCfgSettings = prWfdCfgSettings;

			if (ucMode == MIRACAST_MODE_OFF) {
				prWfdCfgSettings->ucWfdEnable = 0;
				snprintf(pcCommand, i4TotalLen, CMD_SET_CHIP " mira 0");
			} else if (ucMode == MIRACAST_MODE_SOURCE) {
				prWfdCfgSettings->ucWfdEnable = 1;
				snprintf(pcCommand, i4TotalLen, CMD_SET_CHIP " mira 1");
			} else if (ucMode == MIRACAST_MODE_SINK) {
				prWfdCfgSettings->ucWfdEnable = 2;
				snprintf(pcCommand, i4TotalLen, CMD_SET_CHIP " mira 2");
			} else {
				prWfdCfgSettings->ucWfdEnable = 0;
				snprintf(pcCommand, i4TotalLen, CMD_SET_CHIP " mira 0");
			}

			mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prMsgWfdCfgUpdate, MSG_SEND_METHOD_BUF);

			priv_driver_set_chip_config(prNetDev, pcCommand, i4TotalLen);
		} /* prMsgWfdCfgUpdate */
		else {
			ASSERT(FALSE);
			i4BytesWritten = -1;
		}
	}

	/* i4Argc */
	return i4BytesWritten;
}

int priv_driver_set_suspend_mode(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	INT_32 i4Argc = 0;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX];
	BOOLEAN fgEnable;
	UINT_32 u4Enable;
	INT_32 u4Ret = 0;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	if (i4Argc >= 2) {
		/* fgEnable = (kalStrtoul(apcArgv[1], NULL, 0) == 1) ? TRUE : FALSE; */
		u4Ret = kalkStrtou32(apcArgv[1], 0, &u4Enable);
		if (u4Ret)
			DBGLOG(REQ, LOUD, "parse u4Enable error u4Ret=%d\n", u4Ret);
		if (u4Enable == 1)
			fgEnable = TRUE;
		else
			fgEnable = FALSE;

		DBGLOG(REQ, INFO, "%s: Set suspend mode [%u]\n", __func__, fgEnable);

		if (prGlueInfo->fgIsInSuspendMode == fgEnable) {
			DBGLOG(REQ, INFO, "%s: Already in suspend mode, SKIP!\n", __func__);
			return 0;
		}

		/*need to update wifi on time statistics*/
		updateWifiOnTimeStatistics();

		prGlueInfo->fgIsInSuspendMode = fgEnable;

		wlanSetSuspendMode(prGlueInfo, fgEnable);
		p2pSetSuspendMode(prGlueInfo, fgEnable);
	}

	return 0;
}

static int priv_driver_get_wifi_type(IN struct net_device *prNetDev,
				     IN char *pcCommand, IN int i4TotalLen)
{
	struct PARAM_GET_WIFI_TYPE rParamGetWifiType;
	P_GLUE_INFO_T prGlueInfo = NULL;
	uint32_t rStatus;
	uint32_t u4BytesWritten = 0;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE) {
		DBGLOG(REQ, ERROR, "GLUE_CHK_PR2 fail\n");
		return -1;
	}

	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	rParamGetWifiType.prNetDev = prNetDev;

	rStatus = kalIoctl(prGlueInfo,
			   wlanoidGetWifiType,
			   (void *)&rParamGetWifiType,
			   sizeof(void *),
			   FALSE,
			   FALSE,
			   FALSE,
			   &u4BytesWritten);

	if (rStatus == WLAN_STATUS_SUCCESS) {
		if (u4BytesWritten > 0) {
			if (u4BytesWritten > i4TotalLen)
				u4BytesWritten = i4TotalLen;
			kalMemCopy(pcCommand, rParamGetWifiType.arWifiTypeName,
				   u4BytesWritten);
		}
	} else {
		DBGLOG(REQ, ERROR, "rStatus=%x\n", rStatus);
		u4BytesWritten = 0;
	}

	return (int)u4BytesWritten;
}

#if CFG_SUPPORT_ANT_DIVERSITY
int priv_driver_new_orientation(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	INT_32 i4Argc = 0;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX];
	INT_32 i4BytesWritten = 0;
	UINT_32 u4BufLen = 0;
	UINT_32 u4Orientation = 5;
	WLAN_STATUS rStatus;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	if (i4Argc != 2 || kalkStrtou32(apcArgv[1], 0, &u4Orientation) || u4Orientation >= 4) {
		prGlueInfo->rNewOrientationStatus = WLAN_STATUS_BUFFER_TOO_SHORT;
		return kalSnprintf(pcCommand, i4TotalLen, "parse parameter [%d][%d] error!\n", i4Argc, u4Orientation);
	}

	if (GLUE_NOT_SUPPORT_DUAL_ANT(prGlueInfo)) {
		prGlueInfo->rNewOrientationStatus = WLAN_STATUS_NOT_SUPPORTED;
		return kalSnprintf(pcCommand, i4TotalLen, "%s\n", "current board didn't support dual antenna!");
	}

	rStatus = kalIoctl(prGlueInfo, wlanoidSwitchAntenna,
			&u4Orientation, sizeof(UINT_32), TRUE, TRUE, TRUE, &u4BufLen);
	prGlueInfo->rNewOrientationStatus = rStatus;
	switch(rStatus) {
		case WLAN_STATUS_INVALID_DATA:
			i4BytesWritten = kalSnprintf(pcCommand, i4TotalLen, "%s\n", "didn't need to switch");
			break;
		case WLAN_STATUS_RESOURCES:
			i4BytesWritten = kalSnprintf(pcCommand, i4TotalLen, "%s\n", "during normal scan");
			break;
		case WLAN_STATUS_MEDIA_DISCONNECT:
			i4BytesWritten = kalSnprintf(pcCommand, i4TotalLen, "%s\n", "media disconnect");
			break;
		case WLAN_STATUS_SUCCESS:
			i4BytesWritten = kalSnprintf(pcCommand, i4TotalLen, "%s\n", "cmd execute success");
			break;
		default:
			i4BytesWritten = kalSnprintf(pcCommand, i4TotalLen, "%s\n", "unexpected case");
			break;
	}

	return i4BytesWritten;
}

static struct PARAM_ANT_SWITCH_TYPE rCmdBuf;

int priv_driver_ant_switch_test(IN struct net_device *prNetDev,
		IN char *pcCommand, IN int i4TotalLen)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	struct PARAM_ANT_SWITCH_TYPE *prCmdBuf;
	INT_32 i4Argc = 0;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX];
	UINT_32 u4BufLen = 0;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	UINT_8 ucIsRightPanel = 0;
	UINT_8 ucPanelIdmeIndex = 0;
	UINT_8 ucPIPattern = 0;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));
	prCmdBuf = &rCmdBuf;

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	if (i4Argc < 2)
		return kalSnprintf(pcCommand, i4TotalLen,
				"arg count [%d] error!", i4Argc);
	if (kalkStrtou32(apcArgv[1], 0, &(prCmdBuf->u4Cmd))) {
		DBGLOG(REQ, INFO, "parse u4Enable error!\n");
		return kalSnprintf(pcCommand, i4TotalLen, "%s",
				"parse parameter error!\n");
	}
	if (i4Argc == 2 || kalkStrtou32(apcArgv[2], 0, &(prCmdBuf->u4Data)))
		prCmdBuf->u4Data = 0;

	switch(prCmdBuf->u4Cmd) {
	case ENUM_QUERY_BORAD_ID:
		ucIsRightPanel = (prGlueInfo->ucASGoodPanel & \
			BIT(prGlueInfo->ucPanelType)) != 0 ? 1 : 0;
		ucPanelIdmeIndex = (ucIsRightPanel << 1) | \
			prGlueInfo->fgIsAntSwitchDisable;
		ucPIPattern = prGlueInfo->ucPanelIdmePattern & \
			BIT(ucPanelIdmeIndex);
		return kalSnprintf(pcCommand, i4TotalLen,
			"\nBoard ID:[%s]\n"
			"HW[%u] PI[0x%02X] GP[0x%02X] RP[%u] PII[%u]\n"
			"PN[%u] AS[%u] PP[%u] SDA[%u] DA[%u]",
			prGlueInfo->idme_board_id,
			prGlueInfo->fgIsHwSupportAntSw,
			prGlueInfo->ucPanelIdmePattern,
			prGlueInfo->ucASGoodPanel, ucIsRightPanel,
			ucPanelIdmeIndex, prGlueInfo->ucPanelType,
			prGlueInfo->fgIsAntSwitchDisable, ucPIPattern,
			prGlueInfo->fgIsHwSupportAntSw && ucPIPattern,
			GLUE_NOT_SUPPORT_DUAL_ANT(prGlueInfo));
	case ENUM_AS_SET_PANEL_IDME:
		prGlueInfo->ucPanelIdmePattern = prCmdBuf->u4Data;
		return kalSnprintf(pcCommand, i4TotalLen,
				"Panel IDME Pattern [0x%02x]",
				prGlueInfo->ucPanelIdmePattern);
	case ENUM_AS_SET_GOOD_PANEL:
		prGlueInfo->ucASGoodPanel = prCmdBuf->u4Data;
		return kalSnprintf(pcCommand, i4TotalLen,
				"Good Panel Pattern [0x%02x]",
				prGlueInfo->ucASGoodPanel);
	case ENUM_AS_SET_PANEL_TYPE:
		prGlueInfo->ucPanelType= prCmdBuf->u4Data;
		return kalSnprintf(pcCommand, i4TotalLen,
				"Panel Type [%u]",
				prGlueInfo->ucPanelType);
	case ENUM_AS_SET_HW_SUPPORT:
		prGlueInfo->fgIsHwSupportAntSw = prCmdBuf->u4Data;
		return kalSnprintf(pcCommand, i4TotalLen,
				"Is HW Support [%u]",
				prGlueInfo->fgIsHwSupportAntSw);
	case ENUM_AS_SET_IDME_SUPPORT:
		prGlueInfo->fgIsAntSwitchDisable = prCmdBuf->u4Data;
		return kalSnprintf(pcCommand, i4TotalLen,
				"Ant SW Support [%u]",
				prGlueInfo->fgIsAntSwitchDisable);
	default:
		break;
	}

	if (GLUE_NOT_SUPPORT_DUAL_ANT(prGlueInfo))
		return kalSnprintf(pcCommand, i4TotalLen,
				"didn't support dual antenna [%u/%u/%u]",
				prGlueInfo->fgIsHwSupportAntSw,
				prGlueInfo->ucPanelType,
				prGlueInfo->fgIsAntSwitchDisable);

	kalMemZero(prCmdBuf->aucRet, sizeof(prCmdBuf->aucRet));
	prCmdBuf->u4RetSize = sizeof(prCmdBuf->aucRet);

	switch(prCmdBuf->u4Cmd) {
	case ENUM_QUERY_CURRENT_ANT:
	case ENUM_SET_ANT_NUM:
	case ENUM_AS_TEST_SCAN:
	case ENUM_AS_TEST_SW_SUCCESS:
	case ENUM_AS_TEST_SW_FAIL:
	case ENUM_AS_TEST_CUR_ANT_NO_RSP:
	case ENUM_AS_TEST_TARGET_ANT_NO_RSP:
	case ENUM_AS_TEST_RANDOM_RCPI:
	case ENUM_AS_TEST_QUERY_ANT_CAP:
	case ENUM_AS_DUMP_RCPI_INFO:
	case ENUM_AS_DUMP_OTHER_STATISTICS:
	case ENUM_AS_SET_DELTA_RSSI:
	case ENUM_AS_DUMP_MATRIX:
	case ENUM_AS_CLEAR_METRICS:
		rStatus = kalIoctl(prGlueInfo, wlanoidAntSwitchTest,
				prCmdBuf, sizeof(struct PARAM_ANT_SWITCH_TYPE),
				TRUE, TRUE, TRUE, &u4BufLen);
		break;
	case ENUM_ANT_SW_ENABLE:
		prGlueInfo->fgIsAntSwitchDisable = prCmdBuf->u4Data ?
				TRUE : FALSE;
		kalSnprintf(prCmdBuf->aucRet, prCmdBuf->u4RetSize,
				"AntSwitchDisable was %u",
				prGlueInfo->fgIsAntSwitchDisable);
		break;
	case ENUM_AS_TEST_ENABLE_AS_QUERY:
		prGlueInfo->fgIsEnableAntSwQuery= prCmdBuf->u4Data ?
				TRUE : FALSE;
		kalSnprintf(prCmdBuf->aucRet, prCmdBuf->u4RetSize,
				"Enable Antenna Switch Query was %u",
				prGlueInfo->fgIsEnableAntSwQuery);
		break;
	default:
		kalSnprintf(prCmdBuf->aucRet, prCmdBuf->u4RetSize,
				"didn't implement test case [%u]",
				prCmdBuf->u4Cmd);
		break;
	}
	DBGLOG(REQ, TRACE, "[AntS] [%u:%u] %s", prCmdBuf->u4Cmd,
			prCmdBuf->u4Data, prCmdBuf->aucRet);
	return kalSnprintf(pcCommand, i4TotalLen, "%s", prCmdBuf->aucRet);
}
#endif
#if CFG_SUPPORT_SNIFFER
int priv_driver_set_monitor(IN struct net_device *prNetDev, IN char *pcCommand, IN int i4TotalLen)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	P_ADAPTER_T prAdapter = NULL;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	UINT_32 u4BufLen = 0;
	INT_32 i4Argc = 0;
	INT_32 i4BytesWritten = 0;
	PCHAR apcArgv[WLAN_CFG_ARGV_MAX];
	PARAM_CUSTOM_MONITOR_SET_STRUCT_T rMonitorSetInfo;
	UINT_8 ucEnable = 0;
	UINT_8 ucPriChannel = 0;
	UINT_8 ucChannelWidth = 0;
	UINT_8 ucExt = 0;
	UINT_8 ucSco = 0;
	UINT_8 ucChannelS1 = 0;
	UINT_8 ucChannelS2 = 0;
	BOOLEAN fgIsLegalChannel = FALSE;
	BOOLEAN fgError = FALSE;
	BOOLEAN fgEnable = FALSE;
	ENUM_BAND_T eBand = BAND_NULL;
	UINT_32 u4Parse = 0;
	INT_32 u4Ret = 0;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));
	prAdapter = prGlueInfo->prAdapter;

	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);

	if (i4Argc >= 5) {
		/*
		 * ucEnable = (UINT_8) (kalStrtoul(apcArgv[1], NULL, 0));
		 * ucPriChannel = (UINT_8) (kalStrtoul(apcArgv[2], NULL, 0));
		 * ucChannelWidth = (UINT_8) (kalStrtoul(apcArgv[3], NULL, 0));
		 * ucExt = (UINT_8) (kalStrtoul(apcArgv[4], NULL, 0));
		 */
		u4Ret = kalkStrtou32(apcArgv[1], 0, &u4Parse);
		if (u4Ret)
			DBGLOG(REQ, LOUD, "parse apcArgv error u4Ret=%d\n", u4Ret);
		ucEnable = (UINT_8)u4Parse;
		u4Ret = kalkStrtou32(apcArgv[2], 0, &u4Parse);
		if (u4Ret)
			DBGLOG(REQ, LOUD, "parse apcArgv error u4Ret=%d\n", u4Ret);
		ucPriChannel = (UINT_8)u4Parse;
		u4Ret = kalkStrtou32(apcArgv[3], 0, &u4Parse);
		if (u4Ret)
			DBGLOG(REQ, LOUD, "parse apcArgv error u4Ret=%d\n", u4Ret);
		ucChannelWidth = (UINT_8)u4Parse;
		u4Ret = kalkStrtou32(apcArgv[4], 0, &u4Parse);
		if (u4Ret)
			DBGLOG(REQ, LOUD, "parse apcArgv error u4Ret=%d\n", u4Ret);
		ucExt = (UINT_8)u4Parse;

		eBand = (ucPriChannel <= 14) ? BAND_2G4 : BAND_5G;
		fgIsLegalChannel = rlmDomainIsLegalChannel(prAdapter, eBand, ucPriChannel);

		if (fgIsLegalChannel == FALSE) {
			i4BytesWritten = snprintf(pcCommand, i4TotalLen, "Illegal primary channel %d", ucPriChannel);
			return i4BytesWritten;
		}

		switch (ucChannelWidth) {
		case 160:
			ucChannelWidth = (UINT_8) CW_160MHZ;
			ucSco = (UINT_8) CHNL_EXT_SCN;

			if (ucPriChannel >= 36 && ucPriChannel <= 64)
				ucChannelS2 = 50;
			else if (ucPriChannel >= 100 && ucPriChannel <= 128)
				ucChannelS2 = 114;
			else
				fgError = TRUE;
			break;

		case 80:
			ucChannelWidth = (UINT_8) CW_80MHZ;
			ucSco = (UINT_8) CHNL_EXT_SCN;

			if (ucPriChannel >= 36 && ucPriChannel <= 48)
				ucChannelS1 = 42;
			else if (ucPriChannel >= 52 && ucPriChannel <= 64)
				ucChannelS1 = 58;
			else if (ucPriChannel >= 100 && ucPriChannel <= 112)
				ucChannelS1 = 106;
			else if (ucPriChannel >= 116 && ucPriChannel <= 128)
				ucChannelS1 = 122;
			else if (ucPriChannel >= 132 && ucPriChannel <= 144)
				ucChannelS1 = 138;
			else if (ucPriChannel >= 149 && ucPriChannel <= 161)
				ucChannelS1 = 155;
			else
				fgError = TRUE;
			break;

		case 40:
			ucChannelWidth = (UINT_8) CW_20_40MHZ;
			ucSco = (ucExt) ? (UINT_8) CHNL_EXT_SCA : (UINT_8) CHNL_EXT_SCB;
			break;

		case 20:
			ucChannelWidth = (UINT_8) CW_20_40MHZ;
			ucSco = (UINT_8) CHNL_EXT_SCN;
			break;

		default:
			fgError = TRUE;
			break;
		}

		if (fgError) {
			i4BytesWritten =
			    snprintf(pcCommand, i4TotalLen, "Invalid primary channel %d with bandwidth %d",
				     ucPriChannel, ucChannelWidth);
			return i4BytesWritten;
		}

		fgEnable = (ucEnable) ? TRUE : FALSE;

		if (prGlueInfo->fgIsEnableMon != fgEnable) {
			prGlueInfo->fgIsEnableMon = fgEnable;
			schedule_work(&prGlueInfo->monWork);
		}

		kalMemZero(&rMonitorSetInfo, sizeof(rMonitorSetInfo));

		rMonitorSetInfo.ucEnable = ucEnable;
		rMonitorSetInfo.ucPriChannel = ucPriChannel;
		rMonitorSetInfo.ucSco = ucSco;
		rMonitorSetInfo.ucChannelWidth = ucChannelWidth;
		rMonitorSetInfo.ucChannelS1 = ucChannelS1;
		rMonitorSetInfo.ucChannelS2 = ucChannelS2;

		rStatus = kalIoctl(prGlueInfo,
				   wlanoidSetMonitor,
				   &rMonitorSetInfo, sizeof(rMonitorSetInfo), FALSE, FALSE, TRUE, &u4BufLen);

		i4BytesWritten =
		    snprintf(pcCommand, i4TotalLen, "set monitor config %s",
			     (rStatus == WLAN_STATUS_SUCCESS) ? "success" : "fail");

		return i4BytesWritten;
	}

	i4BytesWritten = snprintf(pcCommand, i4TotalLen, "monitor [Enable][PriChannel][ChannelWidth][Sco]");

	return i4BytesWritten;
}
#endif

#if CFG_SUPPORT_FW_ACTIVE_TIME_STATISTICS
static int priv_driver_fw_active_time_statistics(IN struct net_device *prNetDev, IN PCHAR pcCommand,
	IN int i4TotalLen)
{
	struct CMD_FW_ACTIVE_TIME_STATISTICS rCmdFwActiveTime = {0};
	INT_32 i4Argc = 0;
	PINT_8 apcArgv[WLAN_CFG_ARGV_MAX] = { 0 };
	INT_32 i4ArgNum = 2;
	INT_32 i4BytesWritten = 0;
	P_GLUE_INFO_T prGlueInfo = NULL;
	INT_32 rStatus = WLAN_STATUS_SUCCESS;
	UINT_32 u4BufLen = 0;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	prGlueInfo = *((struct _GLUE_INFO_T **) netdev_priv(prNetDev));
	if (NULL == prGlueInfo) {
		DBGLOG(REQ, ERROR, "invalid parameter\n");
		return -1;
	}

	if (i4Argc >= i4ArgNum) {
		if (strnicmp(apcArgv[1], "start", strlen("start")) == 0) {
			rCmdFwActiveTime.u4Action = FW_ACTIVE_TIME_STATISTICS_ACTION_START;
			rStatus = kalIoctl(prGlueInfo, wlanoidSetFwActiveTimeStatistics,
				&rCmdFwActiveTime,
				sizeof(struct CMD_FW_ACTIVE_TIME_STATISTICS),
				FALSE, FALSE, TRUE, &u4BufLen);
			if (rStatus != WLAN_STATUS_SUCCESS) {
				DBGLOG(REQ, WARN, "fail to enable fw active time statistics\n");
				return -1;
			}
			g_FwActiveTimeStatus = 1;
		} else if (strnicmp(apcArgv[1], "stop", strlen("stop")) == 0) {
			rCmdFwActiveTime.u4Action = FW_ACTIVE_TIME_STATISTICS_ACTION_STOP;
			rStatus = kalIoctl(prGlueInfo, wlanoidSetFwActiveTimeStatistics,
				&rCmdFwActiveTime,
				sizeof(struct CMD_FW_ACTIVE_TIME_STATISTICS),
				FALSE, FALSE, TRUE, &u4BufLen);
			if (rStatus != WLAN_STATUS_SUCCESS) {
				DBGLOG(REQ, WARN, "fail to disable fw active time statistics\n");
				return -1;
			}
			g_FwActiveTimeStatus = 0;
		} else if (strnicmp(apcArgv[1], "get", strlen("get")) == 0) {
			rCmdFwActiveTime.u4Action = FW_ACTIVE_TIME_STATISTICS_ACTION_GET;
			rStatus = kalIoctl(prGlueInfo, wlanoidGetFwActiveTimeStatistics,
				&rCmdFwActiveTime,
				sizeof(struct CMD_FW_ACTIVE_TIME_STATISTICS),
				TRUE, TRUE, TRUE, &u4BufLen);

			/*construct statistics to upper layer*/
			if (rStatus != WLAN_STATUS_SUCCESS) {
				DBGLOG(REQ, WARN, "unable to get fw active time statistics\n");
				return -1;
			}

			/*update driver statistics*/
			g_FwActiveTime.u4TimeDuringScreenOn += rCmdFwActiveTime.u4TimeDuringScreenOn;
			g_FwActiveTime.u4TimeDuringScreenOff += rCmdFwActiveTime.u4TimeDuringScreenOff;
			g_FwActiveTime.u4HwTimeDuringScreenOn += rCmdFwActiveTime.u4HwTimeDuringScreenOn;
			g_FwActiveTime.u4HwTimeDuringScreenOff += rCmdFwActiveTime.u4HwTimeDuringScreenOff;

			i4BytesWritten = snprintf(pcCommand, i4TotalLen,
				"TimeDuringScreenOn[%u] TimeDuringScreenOff[%u] ",
				g_FwActiveTime.u4TimeDuringScreenOn,
				g_FwActiveTime.u4TimeDuringScreenOff);
			i4BytesWritten += snprintf(pcCommand + i4BytesWritten, i4TotalLen - i4BytesWritten,
				"HwTimeDuringScreenOn[%u] HwTimeDuringScreenOff[%u]\n",
				g_FwActiveTime.u4HwTimeDuringScreenOn,
				g_FwActiveTime.u4HwTimeDuringScreenOff);
			return i4BytesWritten;

		} else {
			DBGLOG(REQ, ERROR, "invalid parameter\n");
			return -1;
		}
	} else {
		DBGLOG(REQ, ERROR, "invalid parameter\n");
		return -1;
	}

	return 0;
}

#endif

static int priv_driver_wifi_on_time_statistics(IN struct net_device *prNetDev, IN PCHAR pcCommand,
	IN int i4TotalLen)
{
	INT_32 i4Argc = 0;
	INT_8 *apcArgv[WLAN_CFG_ARGV_MAX] = { 0 };
	INT_32 i4ArgNum = 2;
	INT_32 i4BytesWritten = 0;
	P_GLUE_INFO_T prGlueInfo = NULL;

	ASSERT(prNetDev);
	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;

	DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);
	wlanCfgParseArgument(pcCommand, &i4Argc, apcArgv);
	DBGLOG(REQ, LOUD, "argc is %i\n", i4Argc);

	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));
	if (NULL == prGlueInfo) {
		DBGLOG(REQ, ERROR, "invalid parameter\n");
		return -1;
	}

	if (i4Argc >= i4ArgNum) {
		if (strnicmp(apcArgv[1], "get", strlen("get")) == 0) {
			/*need to update wifi on time statistics*/
			updateWifiOnTimeStatistics();

			/*construct statistics to upper layer*/
			i4BytesWritten = snprintf(pcCommand, i4TotalLen,
				"TimeDuringScreenOn[%u] TimeDuringScreenOff[%u] ",
				wifiOnTimeStatistics.u4WifiOnTimeDuringScreenOn,
				wifiOnTimeStatistics.u4WifiOnTimeDuringScreenOff);
			return i4BytesWritten;

		} else {
			DBGLOG(REQ, ERROR, "invalid parameter\n");
			return -1;
		}
	} else {
		DBGLOG(REQ, ERROR, "invalid parameter\n");
		return -1;
	}

	return 0;
}

#if CFG_SUPPORT_BATCH_SCAN
#define CMD_BATCH_SET           "WLS_BATCHING SET"
#define CMD_BATCH_GET           "WLS_BATCHING GET"
#define CMD_BATCH_STOP          "WLS_BATCHING STOP"
#endif

typedef int(*PRIV_CMD_FUNCTION) (
		IN struct net_device *prNetDev,
		IN char *pcCommand,
		IN int i4TotalLen);

struct PRIV_CMD_HANDLER {
	uint8_t *pcCmdStr;
	PRIV_CMD_FUNCTION pfHandler;
};

struct PRIV_CMD_HANDLER priv_cmd_handlers[] = {
	{CMD_GET_WIFI_TYPE, priv_driver_get_wifi_type},
};

INT_32 priv_driver_cmds(IN struct net_device *prNetDev, IN PCHAR pcCommand, IN INT_32 i4TotalLen)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	INT_32 i4BytesWritten = 0;
	INT_32 i4CmdFound = 0;
	int i;

	if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
		return -1;
	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	for (i = 0; i < sizeof(priv_cmd_handlers) / sizeof(struct
			PRIV_CMD_HANDLER); i++) {
		if (strnicmp(pcCommand,
				priv_cmd_handlers[i].pcCmdStr,
				strlen(priv_cmd_handlers[i].pcCmdStr)) == 0) {

			if (priv_cmd_handlers[i].pfHandler != NULL) {
				i4BytesWritten =
					priv_cmd_handlers[i].pfHandler(
					prNetDev,
					pcCommand,
					i4TotalLen);
			}
			i4CmdFound = 1;
		}
	}

	if (i4CmdFound == 0) {
		i4CmdFound = 1;
		if (strncasecmp(pcCommand, CMD_RSSI, strlen(CMD_RSSI)) == 0) {
			/*
			 * i4BytesWritten =
			 * wl_android_get_rssi(net, command, i4TotalLen);
			 */
		} else if (strncasecmp(pcCommand, CMD_LINKSPEED, strlen(CMD_LINKSPEED)) == 0) {
			i4BytesWritten = priv_driver_get_linkspeed(prNetDev, pcCommand, i4TotalLen);
		} else if (strncasecmp(pcCommand, CMD_PNOSSIDCLR_SET, strlen(CMD_PNOSSIDCLR_SET)) == 0) {
			/* ToDo:: Nothing */
		} else if (strncasecmp(pcCommand, CMD_PNOSETUP_SET, strlen(CMD_PNOSETUP_SET)) == 0) {
			/* ToDo:: Nothing */
		} else if (strncasecmp(pcCommand, CMD_PNOENABLE_SET, strlen(CMD_PNOENABLE_SET)) == 0) {
			/* ToDo:: Nothing */
		} else if (strncasecmp(pcCommand, CMD_SETSUSPENDOPT, strlen(CMD_SETSUSPENDOPT)) == 0) {
			/* i4BytesWritten = wl_android_set_suspendopt(net, pcCommand, i4TotalLen); */
		} else if (strncasecmp(pcCommand, CMD_SETSUSPENDMODE, strlen(CMD_SETSUSPENDMODE)) == 0) {
			i4BytesWritten = priv_driver_set_suspend_mode(prNetDev, pcCommand, i4TotalLen);
		} else if (strncasecmp(pcCommand, CMD_SETBAND, strlen(CMD_SETBAND)) == 0) {
			i4BytesWritten = priv_driver_set_band(prNetDev, pcCommand, i4TotalLen);
		} else if (strncasecmp(pcCommand, CMD_GETBAND, strlen(CMD_GETBAND)) == 0) {
			/* i4BytesWritten = wl_android_get_band(net, pcCommand, i4TotalLen); */
		} else if (strncasecmp(pcCommand, CMD_SET_TXPOWER, strlen(CMD_SET_TXPOWER)) == 0) {
			i4BytesWritten = priv_driver_set_txpower(prNetDev, pcCommand, i4TotalLen);
		} else if (strncasecmp(pcCommand, CMD_COUNTRY, strlen(CMD_COUNTRY)) == 0) {
			i4BytesWritten = priv_driver_set_country(prNetDev, pcCommand, i4TotalLen);
		} else if (strncasecmp(pcCommand, CMD_MIRACAST, strlen(CMD_MIRACAST)) == 0) {
			i4BytesWritten = priv_driver_set_miracast(prNetDev, pcCommand, i4TotalLen);
		}
		/* Mediatek private command */
		else if (strncasecmp(pcCommand, CMD_SET_SW_CTRL, strlen(CMD_SET_SW_CTRL)) == 0) {
			i4BytesWritten = priv_driver_set_sw_ctrl(prNetDev, pcCommand, i4TotalLen);
		} else if (strncasecmp(pcCommand, CMD_GET_SW_CTRL, strlen(CMD_GET_SW_CTRL)) == 0) {
			i4BytesWritten = priv_driver_get_sw_ctrl(prNetDev, pcCommand, i4TotalLen);
		} else if (strncasecmp(pcCommand, CMD_SET_CFG, strlen(CMD_SET_CFG)) == 0) {
			i4BytesWritten = priv_driver_set_cfg(prNetDev, pcCommand, i4TotalLen);
		} else if (strncasecmp(pcCommand, CMD_GET_CFG, strlen(CMD_GET_CFG)) == 0) {
			i4BytesWritten = priv_driver_get_cfg(prNetDev, pcCommand, i4TotalLen);
		} else if (strncasecmp(pcCommand, CMD_SET_CHIP, strlen(CMD_SET_CHIP)) == 0) {
			i4BytesWritten = priv_driver_set_chip_config(prNetDev, pcCommand, i4TotalLen);
		} else if (strncasecmp(pcCommand, CMD_GET_CHIP, strlen(CMD_GET_CHIP)) == 0) {
			i4BytesWritten = priv_driver_get_chip_config(prNetDev, pcCommand, i4TotalLen);
		}

#if CFG_SUPPORT_QA_TOOL
		else if (strncasecmp(pcCommand, CMD_GET_RX_STATISTICS, strlen(CMD_GET_RX_STATISTICS)) == 0)
			i4BytesWritten = priv_driver_get_rx_statistics(prNetDev, pcCommand, i4TotalLen);
#if CFG_SUPPORT_BUFFER_MODE
		else if (strncasecmp(pcCommand, CMD_SETBUFMODE, strlen(CMD_SETBUFMODE)) == 0)
			i4BytesWritten = priv_driver_set_efuse_buffer_mode(prNetDev, pcCommand, i4TotalLen);
#endif
#endif

#if CFG_SUPPORT_BATCH_SCAN
		else if (strncasecmp(pcCommand, CMD_BATCH_SET, strlen(CMD_BATCH_SET)) == 0) {
			kalIoctl(prGlueInfo,
				 wlanoidSetBatchScanReq,
				 (PVOID) pcCommand, i4TotalLen, FALSE, FALSE, TRUE, &i4BytesWritten);
		} else if (strncasecmp(pcCommand, CMD_BATCH_GET, strlen(CMD_BATCH_GET)) == 0) {
			/* strcpy(pcCommand, "BATCH SCAN DATA FROM FIRMWARE"); */
			/* i4BytesWritten = strlen("BATCH SCAN DATA FROM FIRMWARE") + 1; */
			/* i4BytesWritten = priv_driver_get_linkspeed (prNetDev, pcCommand, i4TotalLen); */

			UINT_32 u4BufLen;
			int i;
			/* int rlen=0; */

			for (i = 0; i < CFG_BATCH_MAX_MSCAN; i++) {
				g_rEventBatchResult[i].ucScanCount = i + 1;	/* for get which mscan */
				kalIoctl(prGlueInfo,
					 wlanoidQueryBatchScanResult,
					 (PVOID)&g_rEventBatchResult[i],
					 sizeof(EVENT_BATCH_RESULT_T), TRUE, TRUE, TRUE, &u4BufLen);
			}

#if 0
			DBGLOG(SCN, INFO, "Batch Scan Results, scan count = %u\n", g_rEventBatchResult.ucScanCount);
			for (i = 0; i < g_rEventBatchResult.ucScanCount; i++) {
				prEntry = &g_rEventBatchResult.arBatchResult[i];
				DBGLOG(SCN, INFO, "Entry %u\n", i);
				DBGLOG(SCN, INFO, "	 BSSID = " MACSTR "\n", MAC2STR(prEntry->aucBssid));
				DBGLOG(SCN, INFO, "	 SSID = %s\n", prEntry->aucSSID);
				DBGLOG(SCN, INFO, "	 SSID len = %u\n", prEntry->ucSSIDLen);
				DBGLOG(SCN, INFO, "	 RSSI = %d\n", prEntry->cRssi);
				DBGLOG(SCN, INFO, "	 Freq = %u\n", prEntry->ucFreq);
			}
#endif

			batchConvertResult(&g_rEventBatchResult[0], pcCommand, i4TotalLen, &i4BytesWritten);

			/* Dump for debug */
			/*
			 * print_hex_dump(KERN_INFO,
			 * "BATCH", DUMP_PREFIX_ADDRESS, 16, 1, pcCommand, i4BytesWritten, TRUE);
			 */

		} else if (strncasecmp(pcCommand, CMD_BATCH_STOP, strlen(CMD_BATCH_STOP)) == 0) {
			kalIoctl(prGlueInfo,
				 wlanoidSetBatchScanReq,
				 (PVOID) pcCommand, i4TotalLen, FALSE, FALSE, TRUE, &i4BytesWritten);
		}
#endif
#if CFG_SUPPORT_SNIFFER
		else if (strncasecmp(pcCommand, CMD_SETMONITOR, strlen(CMD_SETMONITOR)) == 0)
			i4BytesWritten = priv_driver_set_monitor(prNetDev, pcCommand, i4TotalLen);
#endif
		else if (!strncasecmp(pcCommand, CMD_DUMP_TS, strlen(CMD_DUMP_TS)) ||
			 !strncasecmp(pcCommand, CMD_ADD_TS, strlen(CMD_ADD_TS)) ||
			 !strncasecmp(pcCommand, CMD_DEL_TS, strlen(CMD_DEL_TS))) {
			kalIoctl(prGlueInfo, wlanoidTspecOperation, (PVOID)pcCommand,
				 i4TotalLen, FALSE, FALSE, FALSE, &i4BytesWritten);
		} else if (kalStrStr(pcCommand, "-IT ")) {
			kalIoctl(prGlueInfo, wlanoidPktProcessIT, (PVOID)pcCommand,
				 i4TotalLen, FALSE, FALSE, FALSE, &i4BytesWritten);
		} else if (!strncasecmp(pcCommand, CMD_FW_EVENT, 9)) {
			kalIoctl(prGlueInfo, wlanoidFwEventIT, (PVOID)(pcCommand + 9),
				 i4TotalLen, FALSE, FALSE, FALSE, &i4BytesWritten);
		} else if (!strncasecmp(pcCommand, CMD_DUMP_UAPSD, strlen(CMD_DUMP_UAPSD)))
			kalIoctl(prGlueInfo, wlanoidDumpUapsdSetting, (PVOID)pcCommand,
				 i4TotalLen, FALSE, FALSE, FALSE, &i4BytesWritten);
		else if (!strncasecmp(pcCommand, CMD_FW_PARAM, strlen(CMD_FW_PARAM))) {
			kalIoctl(prGlueInfo, wlanoidSetFwParam, (PVOID)(pcCommand + 13),
				 i4TotalLen - 13, FALSE, FALSE, FALSE, &i4BytesWritten);
#if CFG_SUPPORT_ANT_DIVERSITY
		} else if (!strncasecmp(pcCommand, CMD_ANT_SWITCH_TEST, strlen(CMD_ANT_SWITCH_TEST))) {
		    i4BytesWritten = priv_driver_ant_switch_test(prNetDev, pcCommand, i4TotalLen);
		} else if (!strncasecmp(pcCommand, CMD_NEW_ORIENTATION, strlen(CMD_NEW_ORIENTATION))) {
		    i4BytesWritten = priv_driver_new_orientation(prNetDev, pcCommand, i4TotalLen);
#endif
		}
#if CFG_SUPPORT_FW_ACTIVE_TIME_STATISTICS
		else if (!strncasecmp(pcCommand, CMD_FW_ACTIVE_STATISTICS, strlen(CMD_FW_ACTIVE_STATISTICS))) {
			i4BytesWritten = priv_driver_fw_active_time_statistics(prNetDev, pcCommand, i4TotalLen);
		}
#endif
		else if (!strncasecmp(pcCommand, CMD_WIFI_ON_TIME_STATISTICS, strlen(CMD_WIFI_ON_TIME_STATISTICS))) {
			i4BytesWritten = priv_driver_wifi_on_time_statistics(prNetDev, pcCommand, i4TotalLen);
		}

		else
			i4CmdFound = 0;
	}
	/* i4CmdFound */
	if (i4CmdFound == 0)
		DBGLOG(REQ, INFO, "Unknown driver command %s - ignored\n", pcCommand);

	if (i4BytesWritten >= 0) {
		if ((i4BytesWritten == 0) && (i4TotalLen > 0)) {
			/* reset the command buffer */
			pcCommand[0] = '\0';
		}

		if (i4BytesWritten >= i4TotalLen) {
			DBGLOG(REQ, INFO,
			       "%s: i4BytesWritten %d > i4TotalLen < %d\n", __func__, i4BytesWritten, i4TotalLen);
			i4BytesWritten = i4TotalLen;
		} else {
			pcCommand[i4BytesWritten] = '\0';
			i4BytesWritten++;
		}
	}

	return i4BytesWritten;

}				/* priv_driver_cmds */

int priv_support_driver_cmd(IN struct net_device *prNetDev, IN OUT struct ifreq *prReq, IN int i4Cmd)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	int ret = 0;
	char *pcCommand = NULL;
	priv_driver_cmd_t *priv_cmd = NULL;
	int i4BytesWritten = 0;
	int i4TotalLen = 0;

	if (!prReq->ifr_data) {
		ret = -EINVAL;
		goto exit;
	}

	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prNetDev));

	if (!prGlueInfo) {
		DBGLOG(REQ, WARN, "No glue info\n");
		ret = -EFAULT;
		goto exit;
	}
	if (prGlueInfo->u4ReadyFlag == 0) {
		ret = -EINVAL;
		goto exit;
	}

	priv_cmd = kzalloc(sizeof(priv_driver_cmd_t), GFP_KERNEL);
	if (!priv_cmd) {
		DBGLOG(REQ, WARN, "%s, alloc mem failed\n", __func__);
		return -ENOMEM;
	}

	if (copy_from_user(priv_cmd, prReq->ifr_data, sizeof(priv_driver_cmd_t))) {
		DBGLOG(REQ, ERROR, "%s: copy_from_user fail\n", __func__);
		ret = -EFAULT;
		goto exit;
	}

	i4TotalLen = priv_cmd->total_len;

	if (i4TotalLen <= 0 || i4TotalLen > PRIV_CMD_SIZE) {
		ret = -EINVAL;
		DBGLOG(REQ, ERROR, "%s: i4TotalLen invalid\n", __func__);
		goto exit;
	}
	priv_cmd->buf[PRIV_CMD_SIZE - 1] = '\0';
	pcCommand = priv_cmd->buf;

	DBGLOG(REQ, INFO, "%s: driver cmd \"%s\" on %s\n", __func__, pcCommand, prReq->ifr_name);

	i4BytesWritten = priv_driver_cmds(prNetDev, pcCommand, i4TotalLen);

	if (i4BytesWritten < 0) {
		DBGLOG(REQ, ERROR, "%s: command %s failed; Written is %d\n",
			__func__, pcCommand, i4BytesWritten);
		ret = -EFAULT;
	}

exit:
	kfree(priv_cmd);

	return ret;
}				/* priv_support_driver_cmd */

static int compat_priv(IN struct net_device *prNetDev,
	     IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN char *pcExtra,
	     int (*priv_func)(IN struct net_device *prNetDev,
	     IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN char *pcExtra))
{
	struct iw_point *prIwp;
	int ret = 0;
#ifdef CONFIG_COMPAT
	struct compat_iw_point *iwp_compat = NULL;
	struct iw_point iwp;
#endif

	if (!prIwReqData)
		return -EINVAL;

#ifdef CONFIG_COMPAT
	if (prIwReqInfo->flags & IW_REQUEST_FLAG_COMPAT) {
		iwp_compat = (struct compat_iw_point *) &prIwReqData->data;
		iwp.pointer = compat_ptr(iwp_compat->pointer);
		iwp.length = iwp_compat->length;
		iwp.flags = iwp_compat->flags;
		prIwp = &iwp;
	} else
#endif
	prIwp = &prIwReqData->data;


	ret = priv_func(prNetDev, prIwReqInfo, (union iwreq_data *)prIwp, pcExtra);

#ifdef CONFIG_COMPAT
	if (prIwReqInfo->flags & IW_REQUEST_FLAG_COMPAT) {
		iwp_compat->pointer = ptr_to_compat(iwp.pointer);
		iwp_compat->length = iwp.length;
		iwp_compat->flags = iwp.flags;
	}
#endif
	return ret;
}

int
priv_set_int(IN struct net_device *prNetDev,
	     IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN char *pcExtra)
{
	return compat_priv(prNetDev, prIwReqInfo, prIwReqData, pcExtra, _priv_set_int);
}

int
priv_get_int(IN struct net_device *prNetDev,
	     IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN OUT char *pcExtra)
{
	return compat_priv(prNetDev, prIwReqInfo, prIwReqData, pcExtra, _priv_get_int);
}

int
priv_set_ints(IN struct net_device *prNetDev,
	      IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN char *pcExtra)
{
	return compat_priv(prNetDev, prIwReqInfo, prIwReqData, pcExtra, _priv_set_ints);
}

int
priv_get_ints(IN struct net_device *prNetDev,
	      IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN OUT char *pcExtra)
{
	return compat_priv(prNetDev, prIwReqInfo, prIwReqData, pcExtra, _priv_get_ints);
}

int
priv_set_struct(IN struct net_device *prNetDev,
		IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN char *pcExtra)
{
	return compat_priv(prNetDev, prIwReqInfo, prIwReqData, pcExtra, _priv_set_struct);
}

int
priv_get_struct(IN struct net_device *prNetDev,
		IN struct iw_request_info *prIwReqInfo, IN union iwreq_data *prIwReqData, IN OUT char *pcExtra)
{
	return compat_priv(prNetDev, prIwReqInfo, prIwReqData, pcExtra, _priv_get_struct);
}
