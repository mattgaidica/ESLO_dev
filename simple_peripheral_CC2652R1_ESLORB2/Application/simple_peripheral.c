/******************************************************************************

 @file  simple_peripheral.c

 Group: WCS, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************/
#include <stdint.h>
#include <unistd.h>

#include <string.h>
#include <math.h> // atan2(x,y), M_PI

#include <ti/drivers/ADC.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/NVS.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/UART.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#if (!(defined __TI_COMPILER_VERSION__) && !(defined __GNUC__))
#include <intrinsics.h>
#endif

#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include <simple_gatt_profile.h>

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

#include <ti_drivers_config.h>
#include "simple_peripheral.h"
#include "ti_ble_config.h"

#define PTM_MODE 1 // disables NPI below
#ifdef PTM_MODE
#include "npi_task.h"               // To allow RX event registration
#include "npi_ble.h"                // To enable transmission of messages to UART
#include "icall_hci_tl.h"   // To allow ICall HCI Transport Layer
#endif // PTM_MODE

/***** ESLO *****/
/* AXY */

/* NAND */
#include <SPI_NAND.h>
#include <ESLO.h>
#include <Serialize.h>

/* ADS129X */
#include <ADS129X.h>
#include <Definitions.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// How often to perform periodic event (in ms)
#define SP_PERIODIC_EVT_PERIOD               3000
#define ES_PERIODIC_EVT_PERIOD				 60000
#define ES_AXY_PERIOD				 		 1000
#define ES_ADV_SLEEP_PERIOD					 60000
#define ES_ADV_AWAKE_PERIOD					 5000

// Task configuration
#define SP_TASK_PRIORITY                     1

#ifndef SP_TASK_STACK_SIZE
#define SP_TASK_STACK_SIZE                   1024
#endif

// Application events
#define SP_STATE_CHANGE_EVT                  0
#define SP_CHAR_CHANGE_EVT                   1
#define SP_KEY_CHANGE_EVT                    2
#define SP_ADV_EVT                           3
#define SP_PAIR_STATE_EVT                    4
#define SP_PASSCODE_EVT                      5
#define SP_PERIODIC_EVT                      6
#define SP_READ_RPA_EVT                      7
#define SP_SEND_PARAM_UPDATE_EVT             8
#define SP_CONN_EVT                          9
#define ES_EEG_NOTIF						 10
#define ES_XL_NOTIF							 11
#define ES_PERIODIC_EVT						 12
#define ES_EXPORT_DATA						 13
#define ES_AXY_EVT						     14
#define ES_EXPORT_POST						 15
#define ES_EXPORT_DONE					     16
#define ES_ADV_SLEEP					     17

// Internal Events for RTOS application
#define SP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define SP_ALL_EVENTS                        (SP_ICALL_EVT             | \
                                              SP_QUEUE_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define SP_ADDR_STR_SIZE     15

// For storing the active connections
#define SP_RSSI_TRACK_CHNLS        1            // Max possible channels can be GAP_BONDINGS_MAX
#define SP_MAX_RSSI_STORE_DEPTH    5
#define SP_INVALID_HANDLE          0xFFFF
#define RSSI_2M_THRSHLD           -30           
#define RSSI_1M_THRSHLD           -40           
#define RSSI_S2_THRSHLD           -50           
#define RSSI_S8_THRSHLD           -60           
#define SP_PHY_NONE                LL_PHY_NONE  // No PHY set
#define AUTO_PHY_UPDATE            0xFF

// Spin if the expression is not true
#define SIMPLEPERIPHERAL_ASSERT(expr) if (!(expr)) simple_peripheral_spin();

/*********************************************************************
 * TYPEDEFS
 */

// Auto connect availble groups
enum {
	AUTOCONNECT_DISABLE = 0,              // Disable
	AUTOCONNECT_GROUP_A = 1,              // Group A
	AUTOCONNECT_GROUP_B = 2               // Group B
};

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct {
	uint8_t event;                // event type
	void *pData;               // pointer to message
} spEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct {
	uint8_t state;
	uint16_t connHandle;
	uint8_t status;
} spPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct {
	uint8_t deviceAddr[B_ADDR_LEN];
	uint16_t connHandle;
	uint8_t uiInputs;
	uint8_t uiOutputs;
	uint32_t numComparison;
} spPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct {
	uint32_t event;
	void *pBuf;
} spGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct {
	uint8_t event;                //
	uint8_t data[];
} spClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct {
	List_Elem elem;
	uint16_t connHandle;
} spConnHandleEntry_t;

// Connected device information
typedef struct {
	uint16_t connHandle;                        // Connection Handle
	spClockEventData_t *pParamUpdateEventData;
	Clock_Struct *pUpdateClock;                      // pointer to clock struct
	int8_t rssiArr[SP_MAX_RSSI_STORE_DEPTH];
	uint8_t rssiCntr;
	int8_t rssiAvg;
	bool phyCngRq;           // Set to true if PHY change request is in progress
	uint8_t currPhy;
	uint8_t rqPhy;
	uint8_t phyRqFailCnt;                      // PHY change request count
	bool isAutoPHYEnable;                   // Flag to indicate auto phy change
} spConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task configuration
Task_Struct spTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(spTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t spTaskStack[SP_TASK_STACK_SIZE];

#define APP_EVT_EVENT_MAX 0x9
char *appEventStrings[] = { "APP_STATE_CHANGE_EVT     ",
		"APP_CHAR_CHANGE_EVT      ", "APP_KEY_CHANGE_EVT       ",
		"APP_ADV_EVT              ", "APP_PAIR_STATE_EVT       ",
		"APP_PASSCODE_EVT         ", "APP_READ_RPA_EVT         ",
		"APP_PERIODIC_EVT         ", "APP_SEND_PARAM_UPDATE_EVT",
		"APP_CONN_EVT             ", };

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkNotifyVitals;
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass periodic event ID to clock handler
spClockEventData_t argPeriodic = { .event = SP_PERIODIC_EVT };

// Memory to pass RPA read event ID to clock handler
spClockEventData_t argRpaRead = { .event = SP_READ_RPA_EVT };

// Per-handle connection info
static spConnRec_t connList[MAX_NUM_BLE_CONNS];

// Current connection handle as chosen by menu
static uint16_t menuConnHandle = LINKDB_CONNHANDLE_INVALID;

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Auto connect Disabled/Enabled {0 - Disabled, 1- Group A , 2-Group B, ...}
uint8_t autoConnect = AUTOCONNECT_DISABLE;

// Advertising handles
static uint8 advHandleLegacy;
static uint8 advHandleLongRange;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = { 0 };

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimplePeripheral_init(void);
static void SimplePeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg);
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf,
		uintptr_t arg);
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData);
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg);
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId);
static void SimplePeripheral_notifyVitals(void);
static void SimplePeripheral_updateRPA(void);
static void SimplePeripheral_clockHandler(UArg arg);
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr,
		uint16_t connHandle, uint8_t uiInputs, uint8_t uiOutputs,
		uint32_t numComparison);
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
		uint8_t status);
static void SimplePeripheral_processPairState(spPairStateData_t *pPairState);
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData);
static void SimplePeripheral_charValueChangeCB(uint8_t paramId);
static status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData);
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void SimplePeripheral_initPHYRSSIArray(void);
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
static uint8_t SimplePeripheral_addConn(uint16_t connHandle);
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle);
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle);
static void SimplePeripheral_processParamUpdate(uint16_t connHandle);
static status_t SimplePeripheral_startAutoPhyChange(uint16_t connHandle);
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle);
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
		uint8_t txPhy, uint8_t rxPhy, uint16_t phyOpts);
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle);
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t SimplePeripheral_BondMgrCBs = { SimplePeripheral_passcodeCb, // Passcode callback
		SimplePeripheral_pairStateCb       // Pairing/Bonding state Callback
		};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t SimplePeripheral_simpleProfileCBs = {
		SimplePeripheral_charValueChangeCB // Simple GATT Characteristic value change callback
		};

/*********************************************************************
 * ESLO FUNCTIONS
 */

static uint8_t updateEEGFromSettings(bool actOnInterrupt);
static void eegInterrupt(bool enableInterrupt);

static uint8_t updateXlFromSettings(bool actOnInterrupt);
static void xlInterrupt(bool enableInterrupt);

static uint8_t USE_EEG(uint8_t *esloSettings);
static uint8_t USE_AXY(uint8_t *esloSettings);

static void mapEsloSettings(uint8_t *esloSettingsNew);
static void ESLO_performPeriodicTask();

static void esloSetVersion();
static void exportDataBLE();
static void readBatt();
static void readTherm();
static void esloRecoverSession();
static void esloUpdateNVS();
static void esloResetVersion();
static void WatchdogCallbackFxn();
static void advSleep();

// !! MOVED TO ESLO.H, consider doing for all
//NVS_Handle nvsHandle;
//NVS_Attrs regionAttrs;
//NVS_Params nvsParams;
//static uint32_t nvsBuffer[3]; // esloSignature, esloVersion, esloAddr
//uint32_t ESLOSignature = 0xE123E123; // something unique

static Clock_Struct clkESLOPeriodic;
spClockEventData_t argESLOPeriodic = { .event = ES_PERIODIC_EVT };

static Clock_Struct clkESLOAxy;
spClockEventData_t argESLOAxy = { .event = ES_AXY_EVT };

static Clock_Struct clkESLOAdvSleep;
spClockEventData_t argESLOAdvSleep = { .event = ES_ADV_SLEEP };
uint8_t isAsleep = 0;

uint8_t esloSettings[SIMPLEPROFILE_CHAR3_LEN] = { 0 };
uint8_t esloSettingsSleep[SIMPLEPROFILE_CHAR3_LEN] = { 0 };

bool isPaired = false;

uint32_t lowVoltage; // acts as boolean, use int32 to unwrap easily in app
uint32_t vbatt_uV;
ADC_Handle adc_vBatt;
ADC_Params adcParams_vBatt;

int32_t temp_uC;
ADC_Handle adc_therm;
ADC_Params adcParams_therm;

int_fast16_t adcRes;
uint16_t adcValue;

uint32_t absoluteTime = 0;
uint32_t esloVersion = 0x00000000;
uint32_t axyCount = 0;
uint32_t eegCount = 0;
ReturnType ret; // NAND

#define PACKET_SZ_EEG SIMPLEPROFILE_CHAR4_LEN / 4
int32_t eeg1Buffer[PACKET_SZ_EEG];
int32_t eeg2Buffer[PACKET_SZ_EEG];
int32_t eeg3Buffer[PACKET_SZ_EEG];
int32_t eeg4Buffer[PACKET_SZ_EEG];
uint8_t iEEG = 0;

#define PACKET_SZ_XL SIMPLEPROFILE_CHAR5_LEN / 4
int32_t xlXBuffer[PACKET_SZ_XL];
int32_t xlYBuffer[PACKET_SZ_XL];
int32_t xlZBuffer[PACKET_SZ_XL];
int32_t mgXBuffer[PACKET_SZ_XL];
int32_t mgYBuffer[PACKET_SZ_XL];
int32_t mgZBuffer[PACKET_SZ_XL];
uint8_t iXL = 0;
uint8_t iMG = 0;

/* AXY Vars */

/* NAND Vars */
uint8_t ret;
uint16_t devId;
static uint8_t esloBuffer[PAGE_DATA_SIZE]; // used for writing
static uint8_t readBuf[PAGE_SIZE]; // 2176, always allocate full page size
uint32_t packet;
uAddrType esloAddr, esloExportBlock;

/* ADS129X Vars */
int32_t status;
int32_t ch1;
int32_t ch2;
int32_t ch3;
int32_t ch4;

Watchdog_Params watchdogParams;
Watchdog_Handle watchdogHandle;

uint8_t xl_online = 1;
UART_Handle uart = NULL;

static void advSleep() {
	if (isAsleep) { // wake-up, enable advertise for short period
		Util_restartClock(&clkESLOAdvSleep, ES_ADV_AWAKE_PERIOD);
		GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
		GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
		isAsleep = 0;
	} else {
		Util_restartClock(&clkESLOAdvSleep, ES_ADV_SLEEP_PERIOD);
		GapAdv_disable(advHandleLongRange);
		GapAdv_disable(advHandleLegacy);
		isAsleep = 1;
	}
}

static void esloResetVersion() {
	esloAddr = 0; // comes first, so NAND first entry is version
	esloSetVersion();
	ESLO_encodeNVS(nvsBuffer, &ESLOSignature, &esloVersion, &esloAddr);
	NVS_write(nvsHandle, 0, (void*) nvsBuffer, sizeof(nvsBuffer),
	NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
}

static void esloUpdateNVS() {
	nvsHandle = NVS_open(ESLO_NVS_0, &nvsParams);
	if (nvsHandle != NULL) {
		ESLO_encodeNVS(nvsBuffer, &ESLOSignature, &esloVersion, &esloAddr);
		NVS_write(nvsHandle, 0, (void*) nvsBuffer, sizeof(nvsBuffer),
		NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
	}
	NVS_close(nvsHandle);
}

// set esloAddr and esloVersion
static void esloRecoverSession() {
	uint32_t tempSignature;
	uint32_t tempVersion;
	uint32_t tempAddress;
	bool doVersion = false;

	nvsHandle = NVS_open(ESLO_NVS_0, &nvsParams);
	if (nvsHandle != NULL) {
		NVS_getAttrs(nvsHandle, &regionAttrs);
		NVS_read(nvsHandle, 0, (void*) nvsBuffer, sizeof(nvsBuffer));
		// compare eslo sig
		ESLO_decodeNVS(nvsBuffer, &tempSignature, &tempVersion, &tempAddress);
		if (tempSignature == ESLOSignature) {
			esloVersion = tempVersion;
			esloAddr = tempAddress;
		} else {
			doVersion = true;
		}
	} else {
		doVersion = true;
	}

	if (doVersion) {
		esloResetVersion();
	}

	NVS_close(nvsHandle);
}

static void esloSetVersion() {
	eslo_dt eslo;
	ESLO_GenerateVersion(&esloVersion, CONFIG_TRNG_1);
	eslo.type = Type_Version;
	eslo.data = esloVersion;
	ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);
}

static void exportDataBLE() {
//	uint8_t modBlock = esloExportBlock % 16;
	uint8_t ii;
	uint32_t exportAddr = esloExportBlock * 0x1000;

	if (exportAddr < esloAddr) {
		Power_disablePolicy();
		ret = FlashPageRead(exportAddr, readBuf); // read whole page
		for (ii = 0; ii < 16; ii++) {
			SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR7,
			SIMPLEPROFILE_CHAR7_LEN, readBuf + (ii * 128));
//			SimplePeripheral_enqueueMsg(ES_EXPORT_POST, readBuf + (ii * 128));
		}
	} else {
		Power_enablePolicy();
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR6,
		SIMPLEPROFILE_CHAR6_LEN, &exportAddr);
//		SimplePeripheral_enqueueMsg(ES_EXPORT_DONE, NULL);
	}
}

static void readTherm() {
	adcRes = ADC_convert(adc_therm, &adcValue);
	if (adcRes == ADC_STATUS_SUCCESS) {
		// read, multiply by 2 for voltage divider
		temp_uC = ESLO_convertTherm(
				ADC_convertToMicroVolts(adc_therm, adcValue));
	}
}

// vbatt_uV will never exceed 24-bits
static void readBatt() {
	adcRes = ADC_convert(adc_vBatt, &adcValue);
	if (adcRes == ADC_STATUS_SUCCESS) {
		// read, multiply by 2 for voltage divider
		vbatt_uV = ESLO_convertBatt(
				ADC_convertToMicroVolts(adc_vBatt, adcValue));
	}
}

// sleep should only be called internally
// ...mapEsloSettings() is called when central pushes
static void esloSleep() {
// right now zeros and sleep mode are same
	uint8_t esloSettingsNew[SIMPLEPROFILE_CHAR3_LEN] = { 0 };
// carry over these settings
	esloSettingsNew[Set_TxPower] = esloSettings[Set_TxPower];
	esloSettingsNew[Set_AdvLong] = esloSettings[Set_AdvLong];
// overwrite esloSettings and force sleep mode to take effect
	mapEsloSettings(esloSettingsNew);

// not sure of state right now, so just turn off LED
	uint8_t setGPIO = 0x00;
	SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t), &setGPIO); // only sets parameter, does not cue LED change
	GPIO_write(LED_0, setGPIO);
}

// note that ~Rec = sleep mode and anything in settings that needs to carry over has to be set in esloSleep()
// since it by default overwrites esloSettings with all 0x00 (see Set_TxPower and Set_AdvLong)
static void mapEsloSettings(uint8_t *esloSettingsNew) {
	eslo_dt eslo;

// order: end with the things that have interrupts
//	if (esloSettingsNew[Set_ExportData] > 0x00) {
//		// force turn off AXY and EEG by overwriting new settings
//		esloSettingsNew[Set_SleepWake] = 0x00;
//		esloSettingsNew[Set_EEG1] = 0x00;
//		esloSettingsNew[Set_EEG2] = 0x00;
//		esloSettingsNew[Set_EEG3] = 0x00;
//		esloSettingsNew[Set_EEG4] = 0x00;
//		esloSettingsNew[Set_AxyMode] = 0x00;
//	}
//	esloSettings[Set_ExportData] = esloSettingsNew[Set_ExportData];

// resetVersion only comes from iOS, never maintains value (one and done)
	if (esloSettingsNew[Set_ResetVersion] > 0x00) {
		esloResetVersion();
	}

// this needs some logic: we will never write abstime=0 here
// but cond can occur when the settings are mapped from wakeup
	if (esloSettingsNew[Set_Time1] | esloSettingsNew[Set_Time2]
			| esloSettingsNew[Set_Time3] | esloSettingsNew[Set_Time4] > 0x00) {
		memcpy(&absoluteTime, esloSettingsNew + Set_Time1, 4);
		eslo.type = Type_AbsoluteTime;
		eslo.data = absoluteTime; // data is version in this case
		ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);
		Util_restartClock(&clkESLOPeriodic, ES_PERIODIC_EVT_PERIOD);
	}

	// can't happen when connected, see: GAP_LINK_TERMINATED_EVENT
	if (esloSettings[Set_AdvLong] != *(esloSettingsNew + Set_AdvLong)) {
		esloSettings[Set_AdvLong] = *(esloSettingsNew + Set_AdvLong);
	}

	if (esloSettings[Set_SleepWake] != *(esloSettingsNew + Set_SleepWake)) {
		esloSettings[Set_SleepWake] = *(esloSettingsNew + Set_SleepWake);
	}
	if (esloSettings[Set_TxPower] != *(esloSettingsNew + Set_TxPower)) {
		esloSettings[Set_TxPower] = *(esloSettingsNew + Set_TxPower);
//		GapAdvStatus = GapAdv_disable(advHandleLongRange);
//		GapAdvStatus = GapAdv_disable(advHandleLegacy);
		switch (*(esloSettingsNew + Set_TxPower)) {
		case 0:
			HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_20_DBM);
			break;
		case 1:
			HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_10_DBM);
			break;
		case 2:
			HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
			break;
		case 3:
			HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
			break;
		default:
			break;
		}
//		GapAdvStatus = GapAdv_enable(advHandleLongRange,
//				GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
//		GapAdvStatus = GapAdv_enable(advHandleLegacy,
//				GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
	}
	if (esloSettings[Set_AxyMode] != *(esloSettingsNew + Set_AxyMode)) {
		// set it first, Xl function uses them
		esloSettings[Set_AxyMode] = *(esloSettingsNew + Set_AxyMode);
		updateXlFromSettings(true);
	}
	if (esloSettings[Set_EEG1] != *(esloSettingsNew + Set_EEG1)
			| esloSettings[Set_EEG2] != *(esloSettingsNew + Set_EEG2)
			| esloSettings[Set_EEG3] != *(esloSettingsNew + Set_EEG3)
			| esloSettings[Set_EEG4] != *(esloSettingsNew + Set_EEG4)) {
		// set them first, EEG function uses them
		esloSettings[Set_EEG1] = *(esloSettingsNew + Set_EEG1);
		esloSettings[Set_EEG2] = *(esloSettingsNew + Set_EEG2);
		esloSettings[Set_EEG3] = *(esloSettingsNew + Set_EEG3);
		esloSettings[Set_EEG4] = *(esloSettingsNew + Set_EEG4);
		updateEEGFromSettings(true);
	}

// set and notify iOS, since export data now overrides some settings
	SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN,
			esloSettings);

// updates esloAddr so data will export to end on power cycle if user pushed before powering down
// !! could consider doing this every second if data is recording?
	esloUpdateNVS();

//	if (esloSettingsNew[Set_ExportData] > 0x00) {
//		SimplePeripheral_enqueueMsg(ES_EXPORT_DATA, NULL);
//	}
}

static uint8_t USE_EEG(uint8_t *esloSettings) {
	return (esloSettings[Set_EEG1] | esloSettings[Set_EEG2]
			| esloSettings[Set_EEG3] | esloSettings[Set_EEG4]) & 0x01;
}

static uint8_t USE_AXY(uint8_t *esloSettings) {
	uint8_t ret = 0x00;
	if (esloSettings[Set_AxyMode] > 0) {
		ret = 0x01;
	}
	return ret;
}

static void eegDataHandler(void) {
	eslo_dt eslo_eeg1;
	eslo_dt eslo_eeg2;
	eslo_dt eslo_eeg3;
	eslo_dt eslo_eeg4;

	if (USE_EEG(esloSettings) == ESLO_MODULE_ON) { // double check
		ADS_updateData(&status, &ch1, &ch2, &ch3, &ch4);

		// catch potential issues
		if (status == 0x00000000) {
			return;
		}

		if (esloSettings[Set_EEG1]) {
			eslo_eeg1.type = Type_EEG1;
			eslo_eeg1.data = ch1;
			ESLO_Packet(eslo_eeg1, &packet);
			if (!isPaired) {
				ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_eeg1);
			} else {
				eeg1Buffer[iEEG] = packet;
			}
		}

		if (esloSettings[Set_EEG2]) {
			eslo_eeg2.type = Type_EEG2;
			eslo_eeg2.data = ch2;
			ESLO_Packet(eslo_eeg2, &packet);
			if (!isPaired) {
				ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_eeg2);
			} else {
				eeg2Buffer[iEEG] = packet;
			}
		}

		if (esloSettings[Set_EEG3]) {
			eslo_eeg3.type = Type_EEG3;
			eslo_eeg3.data = ch3;
			ESLO_Packet(eslo_eeg3, &packet);
			if (!isPaired) {
				ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_eeg3);
			} else {
				eeg3Buffer[iEEG] = packet;
			}
		}

		if (esloSettings[Set_EEG4]) {
			eslo_eeg4.type = Type_EEG4;
			eslo_eeg4.data = ch4;
			ESLO_Packet(eslo_eeg4, &packet);
			if (!isPaired) {
				ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_eeg4);
			} else {
				eeg4Buffer[iEEG] = packet;
			}
		}

		iEEG++;
		if (iEEG == PACKET_SZ_EEG) {
			if (isPaired) {
				if (esloSettings[Set_EEG1]) {
					SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,
					SIMPLEPROFILE_CHAR4_LEN, eeg1Buffer);
				}
				if (esloSettings[Set_EEG2]) {
					SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,
					SIMPLEPROFILE_CHAR4_LEN, eeg2Buffer);
				}
				if (esloSettings[Set_EEG3]) {
					SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,
					SIMPLEPROFILE_CHAR4_LEN, eeg3Buffer);
				}
				if (esloSettings[Set_EEG4]) {
					SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,
					SIMPLEPROFILE_CHAR4_LEN, eeg4Buffer);
				}
			}
			iEEG = 0;
		}
		eegCount++;
	}
}

// !! handle ret values?
static void xlDataHandler(void) {
	eslo_dt eslo_xlx;
	eslo_dt eslo_xly;
	eslo_dt eslo_xlz;
	eslo_dt eslo_mgx;
	eslo_dt eslo_mgy;
	eslo_dt eslo_mgz;

//	if (USE_AXY(esloSettings) == ESLO_MODULE_ON) { // double check
//		// XL
//		lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
//		if (reg.status_reg_a.zyxda) {
//			memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
//			lsm303agr_acceleration_raw_get(&dev_ctx_xl,
//					data_raw_acceleration.u8bit);
//
//			eslo_xlx.type = Type_AxyXlx;
//			eslo_xlx.data = (uint32_t) data_raw_acceleration.i16bit[0];
//			ESLO_Packet(eslo_xlx, &packet);
//			xlXBuffer[iXL] = packet;
//			if (!isPaired) {
//				ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_xlx);
//			}
//
//			eslo_xly.type = Type_AxyXly;
//			eslo_xly.data = (uint32_t) data_raw_acceleration.i16bit[1];
//			ESLO_Packet(eslo_xly, &packet);
//			xlYBuffer[iXL] = packet;
//			if (!isPaired) {
//				ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_xly);
//			}
//
//			eslo_xlz.type = Type_AxyXlz;
//			eslo_xlz.data = (uint32_t) data_raw_acceleration.i16bit[2];
//			ESLO_Packet(eslo_xlz, &packet);
//			xlZBuffer[iXL] = packet;
//			if (!isPaired) {
//				ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_xlz);
//			}
//			iXL++;
//		}
//
//		// MG
//		lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
//		if (reg.status_reg_m.zyxda) {
//			memset(data_raw_magnetic.u8bit, 0x00, 3 * sizeof(int16_t));
//			lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw_magnetic.u8bit);
//
//			eslo_mgx.type = Type_AxyMgx;
//			eslo_mgx.data = (uint32_t) data_raw_magnetic.i16bit[0];
//			ESLO_Packet(eslo_mgx, &packet);
//			mgXBuffer[iMG] = packet;
//			if (!isPaired) {
//				ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_mgx);
//			}
//
//			eslo_mgy.type = Type_AxyMgy;
//			eslo_mgy.data = (uint32_t) data_raw_magnetic.i16bit[1];
//			ESLO_Packet(eslo_mgy, &packet);
//			mgYBuffer[iMG] = packet;
//			if (!isPaired) {
//				ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_mgy);
//			}
//
//			//		float compassHeading = atan2f((float)data_raw_magnetic.i16bit[1], (float)data_raw_magnetic.i16bit[0]) * (180 / M_PI);
//
//			eslo_mgz.type = Type_AxyMgz;
//			eslo_mgz.data = (uint32_t) data_raw_magnetic.i16bit[2];
//			ESLO_Packet(eslo_mgz, &packet);
//			mgZBuffer[iMG] = packet;
//			if (!isPaired) {
//				ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_mgz);
//			}
//			iMG++;
//		}
//
//		if (iXL == PACKET_SZ_XL) {
//			if (isPaired) {
//				SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
//				SIMPLEPROFILE_CHAR5_LEN, xlXBuffer);
//				SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
//				SIMPLEPROFILE_CHAR5_LEN, xlYBuffer);
//				SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
//				SIMPLEPROFILE_CHAR5_LEN, xlZBuffer);
//			}
//			iXL = 0;
//		}
//		if (iMG == PACKET_SZ_XL) {
//			if (isPaired) {
//				SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
//				SIMPLEPROFILE_CHAR5_LEN, mgXBuffer);
//				SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
//				SIMPLEPROFILE_CHAR5_LEN, mgYBuffer);
//				SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
//				SIMPLEPROFILE_CHAR5_LEN, mgZBuffer);
//			}
//			iMG = 0;
//		}
//		axyCount++;
//	}
}

void eegDataReady(uint_least8_t index) {
	if (iEEGDiv < EEG_SAMPLING_DIV) {
		iEEGDiv++;
	} else {
		SimplePeripheral_enqueueMsg(ES_EEG_NOTIF, NULL);
		iEEGDiv = 0;
	}
}

void axyXlReady(uint_least8_t index) {
	if (xl_online) {
		SimplePeripheral_enqueueMsg(ES_XL_NOTIF, NULL);
	}
}

static void eegInterrupt(bool enableInterrupt) {
	if (enableInterrupt) {
		GPIO_enableInt(_EEG_DRDY);
	} else {
		GPIO_disableInt(_EEG_DRDY);
	}
}

static uint8_t updateEEGFromSettings(bool actOnInterrupt) {
	bool enableInterrupt;
	uint8_t shdnState = GPIO_read(_SHDN);

	if (USE_EEG(esloSettings) == ESLO_MODULE_ON) {
		if (shdnState == ESLO_LOW) {
			GPIO_write(_SHDN, ESLO_HIGH);
			GPIO_write(_EEG_PWDN, ESLO_HIGH);
			Task_sleep(150000 / Clock_tickPeriod);
			GPIO_setConfig(_EEG_CS,
			GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_LOW | GPIO_CFG_OUT_LOW); // !!consider rm now that 1.8v is supplied
			ADS_init(CONFIG_SPI_EEG, _EEG_CS);
			uint8 adsId = ADS_getDeviceID(); // 0x90 != adsId -> throw error?
			enableInterrupt = true;
			if (actOnInterrupt) {
				eegInterrupt(enableInterrupt);
			}
		}
		// assumes this function is not called unless channel config has changed
		ADS_enableChannels(esloSettings[Set_EEG1], esloSettings[Set_EEG2],
				esloSettings[Set_EEG3], esloSettings[Set_EEG4]);
	}
	if (USE_EEG(esloSettings) == ESLO_MODULE_OFF & shdnState == ESLO_HIGH) {
		enableInterrupt = false;
		eegInterrupt(enableInterrupt); // always turn off before shutting down
		GPIO_setConfig(_EEG_CS, GPIO_CFG_IN_NOPULL); // !!consider rm now that 1.8v is supplied
		ADS_close();
		GPIO_write(_SHDN, ESLO_LOW);
		GPIO_write(_EEG_PWDN, ESLO_LOW);
	}
	return enableInterrupt;
}

static void xlInterrupt(bool enableInterrupt) {
	if (enableInterrupt) {
//		GPIO_enableInt(AXY_DRDY);
	} else {
//		GPIO_disableInt(AXY_DRDY);
	}
}

// !!REDO for ESLO_RB2
static uint8_t updateXlFromSettings(bool actOnInterrupt) {
	bool enableInterrupt;
//
//	if (USE_AXY(esloSettings) == ESLO_MODULE_ON && xl_online && mg_online) {
//		lsm303agr_mag_operating_mode_set(&dev_ctx_mg,
//				LSM303AGR_CONTINUOUS_MODE);
//		lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_10Hz);
//
//		switch (esloSettings[Set_AxyMode]) {
//		case 1:
//			lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_1Hz);
//			Util_rescheduleClock(&clkESLOAxy, 1000);
//			break;
//		case 2:
//			lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_10Hz);
//			Util_rescheduleClock(&clkESLOAxy, 100);
//			break;
//		default:
//			break;
//		}
//		Util_startClock(&clkESLOAxy);
//
//		enableInterrupt = true;
//		if (actOnInterrupt) {
//			xlInterrupt(enableInterrupt);
//		}
//	} else {
//		Util_stopClock(&clkESLOAxy);
//		enableInterrupt = false;
//		xlInterrupt(enableInterrupt); // always turn off before powering down
//		lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_POWER_DOWN);
//		lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_POWER_DOWN);
//	}
	return enableInterrupt;
}

static void ESLO_dumpMemUART() {
	UART_Params uartParams;
	UART_Params_init(&uartParams);
//	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.baudRate = 115200;
	uart = UART_open(CONFIG_UART_0, &uartParams); // UART_close(uart);

	uint8_t i, rxByte;
//	while(1) {
//		UART_read(uart, &rxByte, sizeof(uint8_t));
//
//	}
	for (i = 0; i < 255; i++) {
		UART_write(uart, &i, sizeof(uint8_t));
		GPIO_write(LED_1, !GPIO_read(LED_1));
		Task_sleep(1000);

	}

	UART_close(CONFIG_UART_0);
}

static void ESLO_startup(void) {
	GPIO_init();
	SPI_init();
	ADC_init();
	NVS_init();
	UART_init();
	GPIO_write(LED_0, 0x01);

// init Settings
	esloSettings[Set_EEG1] = 0x00; // only one channel at init
	esloSettings[Set_TxPower] = 0x00; // Set in SysConfig and make it match here
	esloSettings[Set_AxyMode] = 0x00;
	SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN,
			esloSettings);

	/* NVS */
	NVS_Params_init(&nvsParams);
	esloRecoverSession();

	/* NAND */
	NAND_Init(CONFIG_SPI, _NAND_CS);
	ret = FlashReadDeviceIdentification(&devId);

	// break here if debug mode
	if (GPIO_read(DEBUG) == 0x00) {
		ESLO_dumpMemUART();
	}

	/* ADS129X - Defaults in SysConfig */
	bool enableEEGInterrupt = updateEEGFromSettings(false); // do not turn on yet

	/* AXY - init no matter what */
//	AXY_Init(CONFIG_I2C_AXY);
//	dev_ctx_xl.write_reg = platform_i2c_write;
//	dev_ctx_xl.read_reg = platform_i2c_read;
//	dev_ctx_xl.handle = (void*) LSM303AGR_I2C_ADD_XL;
//	dev_ctx_mg.write_reg = platform_i2c_write;
//	dev_ctx_mg.read_reg = platform_i2c_read;
//	dev_ctx_mg.handle = (void*) LSM303AGR_I2C_ADD_MG;
//
//	reg.byte = 0;
//	lsm303agr_xl_device_id_get(&dev_ctx_xl, &reg.byte);
//	if (reg.byte == LSM303AGR_ID_XL) {
//		lsm303agr_temperature_meas_set(&dev_ctx_xl, LSM303AGR_TEMP_ENABLE);
//		lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
//		lsm303agr_xl_full_scale_set(&dev_ctx_xl, LSM303AGR_2g);
//		lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_HR_12bit);
//		//	lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_10Hz); // !! Doesn't matter, updateXlFromSettings() updates them
//	} else {
//		xl_online = 0;
//	}
//
//	reg.byte = 0;
//	lsm303agr_mag_device_id_get(&dev_ctx_mg, &reg.byte);
//	if (reg.byte == LSM303AGR_ID_MG) {
//		/* Restore default configuration for magnetometer */
//		lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);
//		do {
//			lsm303agr_mag_reset_get(&dev_ctx_mg, &reg.byte);
//		} while (reg.byte);
//
//		lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
//		//	lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_10Hz); // !! Doesn't matter, updateXlFromSettings() updates them
//		lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg,
//				LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
//		lsm303agr_mag_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
//		//	lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_CONTINUOUS_MODE); // LSM303AGR_POWER_DOWN, LSM303AGR_CONTINUOUS_MODE
//	} else {
//		mg_online = 0;
//	}
	ADC_Params_init(&adcParams_vBatt);
	adc_vBatt = ADC_open(R_VBATT, &adcParams_vBatt);
	if (adc_vBatt == NULL) {
//		while (1) {
//			// !! what happens here?
//		}
	}
	ADC_Params_init(&adcParams_therm);
	adc_therm = ADC_open(THERM, &adcParams_therm);
	if (adc_therm == NULL) {
//		while (1) {
//			// !! what happens here?
//		}
	}

	Watchdog_init();
	Watchdog_Params_init(&watchdogParams);
	watchdogParams.resetMode = Watchdog_RESET_ON;
//	params.callbackFxn = (Watchdog_Callback) WatchdogCallbackFxn;
	watchdogParams.callbackFxn = NULL;
	watchdogHandle = Watchdog_open(CONFIG_WATCHDOG_0, &watchdogParams);
	if (watchdogHandle == NULL) {
		// Spin forever
//		while (1)
//			;
	}

	updateXlFromSettings(true); // turn on interrupt here
	eegInterrupt(enableEEGInterrupt); // turn on now
	Util_startClock(&clkESLOPeriodic);

	GPIO_write(LED_0, 0x01);
	GPIO_write(LED_1, 0x01);
}

// assumes graceful watchdog
void WatchdogCallbackFxn(Watchdog_Handle handle) {
	esloSleep();
}

/*********************************************************************
 * @fn      simple_peripheral_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void simple_peripheral_spin(void) {
	volatile uint8_t x = 0;

	while (1) {
		x++;
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 */
void SimplePeripheral_createTask(void) {
	Task_Params taskParams;

// Configure task
	Task_Params_init(&taskParams);
	taskParams.stack = spTaskStack;
	taskParams.stackSize = SP_TASK_STACK_SIZE;
	taskParams.priority = SP_TASK_PRIORITY;

	Task_construct(&spTask, SimplePeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void SimplePeripheral_init(void) {
	BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- init ", SP_TASK_PRIORITY);
// ******************************************************************
// N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
// ******************************************************************
// Register the current thread as an ICall dispatcher application
// so that the application can send and receive messages.
	ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
// Set device's Sleep Clock Accuracy
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
	HCI_EXT_SetSCACmd(500);
#endif // (CENTRAL_CFG | PERIPHERAL_CFG)
	RCOSC_enableCalibration();
#endif // USE_RCOSC

// Create an RTOS queue for message from profile to be sent to app.
	appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

// Create one-shot clock for internal periodic events.
	Util_constructClock(&clkNotifyVitals, SimplePeripheral_clockHandler,
	SP_PERIODIC_EVT_PERIOD, 0, false, (UArg) &argPeriodic);

	Util_constructClock(&clkESLOPeriodic, SimplePeripheral_clockHandler,
	ES_PERIODIC_EVT_PERIOD, 0, false, (UArg) &argESLOPeriodic);

	Util_constructClock(&clkESLOAxy, SimplePeripheral_clockHandler,
	ES_AXY_PERIOD, 0, false, (UArg) &argESLOAxy);

	// don't turn on because advertising is enabled on startup below, turn on at conn. terminate
	Util_constructClock(&clkESLOAdvSleep, SimplePeripheral_clockHandler,
	ES_ADV_SLEEP_PERIOD, 0, false, (UArg) &argESLOAdvSleep);

// Set the Device Name characteristic in the GAP GATT Service
// For more information, see the section in the User's Guide:
// http://software-dl.ti.com/lprf/ble5stack-latest/
	GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

// Configure GAP
	{
		uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

		// Pass all parameter update requests to the app for it to decide
		GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
	}

// Setup the GAP Bond Manager. For more information see the GAP Bond Manager
// section in the User's Guide
	setBondManagerParameters();

// Initialize GATT attributes
	GGS_AddService(GATT_ALL_SERVICES);// GAP GATT Service
	GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
	DevInfo_AddService();                      // Device Information Service
	SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

// Setup the SimpleProfile Characteristic Values
// For more information, see the GATT and GATTServApp sections in the User's Guide:
// http://software-dl.ti.com/lprf/ble5stack-latest/
	{
		uint8_t charValue1[SIMPLEPROFILE_CHAR1_LEN] = { 0 };
		uint8_t charValue2[SIMPLEPROFILE_CHAR2_LEN] = { 0 };
//		uint8_t charValue3[SIMPLEPROFILE_CHAR3_LEN] = { 0 }; // set by ESLO init
		uint8_t charValue4[SIMPLEPROFILE_CHAR4_LEN] = { 0 };
		uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 0 };
		uint8_t charValue6[SIMPLEPROFILE_CHAR6_LEN] = { 0 };
		uint8_t charValue7[SIMPLEPROFILE_CHAR7_LEN] = { 0 };

		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1,
		SIMPLEPROFILE_CHAR1_LEN, charValue1);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2,
		SIMPLEPROFILE_CHAR2_LEN, charValue2);
//		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN,
//				charValue3);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,
		SIMPLEPROFILE_CHAR4_LEN, charValue4);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
		SIMPLEPROFILE_CHAR5_LEN, charValue5);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR6,
		SIMPLEPROFILE_CHAR6_LEN, charValue6);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR7,
		SIMPLEPROFILE_CHAR7_LEN, charValue7);
	}

// Register callback with SimpleGATTprofile
	SimpleProfile_RegisterAppCBs(&SimplePeripheral_simpleProfileCBs);

// Start Bond Manager and register callback
	VOID GAPBondMgr_Register(&SimplePeripheral_BondMgrCBs);

// Register with GAP for HCI/Host messages. This is needed to receive HCI
// events. For more information, see the HCI section in the User's Guide:
// http://software-dl.ti.com/lprf/ble5stack-latest/
	GAP_RegisterForMsgs(selfEntity);

// Register for GATT local events and ATT Responses pending for transmission
	GATT_RegisterForMsgs(selfEntity);

// Set default values for Data Length Extension
// Extended Data Length Feature is already enabled by default
	{
		// Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
		// Some brand smartphone is essentially needing 251/2120, so we set them here.
#define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
#define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

		// This API is documented in hci.h
		// See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
		// http://software-dl.ti.com/lprf/ble5stack-latest/
		HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE,
				APP_SUGGESTED_TX_TIME);
	}

// Initialize GATT Client
	GATT_InitClient("");

// Initialize Connection List
	SimplePeripheral_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

	BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- call GAP_DeviceInit", GAP_PROFILE_PERIPHERAL);
//Initialize GAP layer for Peripheral role and register to receive GAP events
	GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode,
			&pRandomAddress);

// Initialize array to store connection handle and RSSI values
	SimplePeripheral_initPHYRSSIArray();

	ESLO_startup();
}

/*********************************************************************
 * @fn      SimplePeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 */
static void SimplePeripheral_taskFxn(UArg a0, UArg a1) {
// Initialize application
	SimplePeripheral_init();

// Application main loop
	for (;;) {
		uint32_t events;

		// Waits for an event to be posted associated with the calling thread.
		// Note that an event associated with a thread is posted when a
		// message is queued to the message receive queue of the thread
		events = Event_pend(syncEvent, Event_Id_NONE, SP_ALL_EVENTS,
		ICALL_TIMEOUT_FOREVER);

		if (events) {
			ICall_EntityID dest;
			ICall_ServiceEnum src;
			ICall_HciExtEvt *pMsg = NULL;

			// Fetch any available messages that might have been sent from the stack
			if (ICall_fetchServiceMsg(&src, &dest,
					(void**) &pMsg) == ICALL_ERRNO_SUCCESS) {
				uint8 safeToDealloc = TRUE;

				if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity)) {
					ICall_Stack_Event *pEvt = (ICall_Stack_Event*) pMsg;

					// Check for BLE stack events first
					if (pEvt->signature != 0xffff) {
						// Process inter-task message
						safeToDealloc = SimplePeripheral_processStackMsg(
								(ICall_Hdr*) pMsg);
					}
				}

				if (pMsg && safeToDealloc) {
					ICall_freeMsg(pMsg);
				}
			}

			// If RTOS queue is not empty, process app message.
			if (events & SP_QUEUE_EVT) {
				while (!Queue_empty(appMsgQueueHandle)) {
					spEvt_t *pMsg = (spEvt_t*) Util_dequeueMsg(
							appMsgQueueHandle);
					if (pMsg) {
						// Process message.
						SimplePeripheral_processAppMsg(pMsg);

						// Free the space from the message.
						ICall_free(pMsg);
					}
				}
			}
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg) {
// Always dealloc pMsg unless set otherwise
	uint8_t safeToDealloc = TRUE;

	BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : Stack msg status=%d, event=0x%x\n", pMsg->status, pMsg->event);

	switch (pMsg->event) {
	case GAP_MSG_EVENT:
		SimplePeripheral_processGapMessage((gapEventHdr_t*) pMsg);
		break;

	case GATT_MSG_EVENT:
		// Process GATT message
		safeToDealloc = SimplePeripheral_processGATTMsg((gattMsgEvent_t*) pMsg);
		break;

	case HCI_GAP_EVENT_EVENT: {
		// Process HCI message
		switch (pMsg->status) {
		case HCI_COMMAND_COMPLETE_EVENT_CODE:
			// Process HCI Command Complete Events here
		{
			SimplePeripheral_processCmdCompleteEvt(
					(hciEvt_CmdComplete_t*) pMsg);
			break;
		}

		case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
			AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, 0);
			break;

			// HCI Commands Events
		case HCI_COMMAND_STATUS_EVENT_CODE: {
			hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t*) pMsg;
			switch (pMyMsg->cmdOpcode) {
			case HCI_LE_SET_PHY: {
				if (pMyMsg->cmdStatus
						== HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE) {
//                Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
//                        "PHY Change failure, peer does not support this");
				} else {
//                Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
//                               "PHY Update Status Event: 0x%x",
//                               pMyMsg->cmdStatus);
				}

				SimplePeripheral_updatePHYStat(HCI_LE_SET_PHY, (uint8_t*) pMsg);
				break;
			}

			default:
				break;
			}
			break;
		}

			// LE Events
		case HCI_LE_EVENT_CODE: {
			hciEvt_BLEPhyUpdateComplete_t *pPUC =
					(hciEvt_BLEPhyUpdateComplete_t*) pMsg;

			// A Phy Update Has Completed or Failed
			if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT) {
				if (pPUC->status != SUCCESS) {
//              Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
//                             "PHY Change failure");
				} else {
					// Only symmetrical PHY is supported.
					// rxPhy should be equal to txPhy.
				}

				SimplePeripheral_updatePHYStat(
				HCI_BLE_PHY_UPDATE_COMPLETE_EVENT, (uint8_t*) pMsg);
			}
			break;
		}

		default:
			break;
		}

		break;
	}

	default:
		// do nothing
		break;
	}

	return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg) {
	if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT) {
		// ATT request-response or indication-confirmation flow control is
		// violated. All subsequent ATT requests or indications will be dropped.
		// The app is informed in case it wants to drop the connection.

		// Display the opcode of the message that caused the violation.
//    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
	} else if (pMsg->method == ATT_MTU_UPDATED_EVENT) {
		// MTU size updated
//    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
	}

// Free message payload. Needed only for ATT Protocol messages
	GATT_bm_free(&pMsg->msg, pMsg->method);

// It's safe to free the incoming message
	return (TRUE);
}

/*********************************************************************
 * @fn      SimplePeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg) {
	bool dealloc = TRUE;

	if (pMsg->event <= APP_EVT_EVENT_MAX) {
		BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=%s\n", 0, appEventStrings[pMsg->event]);
	} else {
		BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=0x%x\n", 0, pMsg->event);
	}

	switch (pMsg->event) {
	case SP_CHAR_CHANGE_EVT:
		SimplePeripheral_processCharValueChangeEvt(*(uint8_t*) (pMsg->pData));
		break;
	case SP_ADV_EVT:
		SimplePeripheral_processAdvEvent((spGapAdvEventData_t*) (pMsg->pData));
		break;
	case SP_PAIR_STATE_EVT:
		SimplePeripheral_processPairState((spPairStateData_t*) (pMsg->pData));
		break;
	case SP_PASSCODE_EVT:
		SimplePeripheral_processPasscode((spPasscodeData_t*) (pMsg->pData));
		break;
	case SP_PERIODIC_EVT:
		SimplePeripheral_notifyVitals();
		break;
	case ES_PERIODIC_EVT:
		ESLO_performPeriodicTask();
		break;
	case ES_AXY_EVT:
		xlDataHandler(); // already in queue, go get data
		break;
	case SP_READ_RPA_EVT:
		SimplePeripheral_updateRPA();
		break;
	case SP_SEND_PARAM_UPDATE_EVT: {
		// Extract connection handle from data
		uint16_t connHandle =
				*(uint16_t*) (((spClockEventData_t*) pMsg->pData)->data);

		SimplePeripheral_processParamUpdate(connHandle);

		// This data is not dynamically allocated
		dealloc = FALSE;
		break;
	}
	case SP_CONN_EVT:
		SimplePeripheral_processConnEvt((Gap_ConnEventRpt_t*) (pMsg->pData));
		break;
	case ES_EEG_NOTIF:
		eegDataHandler();
		break;
	case ES_XL_NOTIF:
		xlDataHandler();
		break;
	case ES_EXPORT_DATA:
		exportDataBLE();
		break;
	case ES_EXPORT_POST:
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR7,
		SIMPLEPROFILE_CHAR7_LEN, (void*) (pMsg->pData));
		break;
	case ES_EXPORT_DONE:
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR6,
		SIMPLEPROFILE_CHAR6_LEN, NULL);
		break;
	case ES_ADV_SLEEP:
		advSleep();
		break;
	default:
		// Do nothing.
		break;
	}

// Free message data if it exists and we are to dealloc
	if ((dealloc == TRUE) && (pMsg->pData != NULL)) {
		ICall_free(pMsg->pData);
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg) {
	switch (pMsg->opcode) {
	case GAP_DEVICE_INIT_DONE_EVENT: {
		bStatus_t status = FAILURE;

		gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t*) pMsg;

		if (pPkt->hdr.status == SUCCESS) {
			// Store the system ID
			uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

			// use 6 bytes of device address for 8 bytes of system ID value
			systemId[0] = pPkt->devAddr[0];
			systemId[1] = pPkt->devAddr[1];
			systemId[2] = pPkt->devAddr[2];

			// set middle bytes to zero
			systemId[4] = 0x00;
			systemId[3] = 0x00;

			// shift three bytes up
			systemId[7] = pPkt->devAddr[5];
			systemId[6] = pPkt->devAddr[4];
			systemId[5] = pPkt->devAddr[3];

			// Set Device Info Service Parameter
			DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
					systemId);

			BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_DEVICE_INIT_DONE_EVENT", 0);
			// Setup and start Advertising
			// For more information, see the GAP section in the User's Guide:
			// http://software-dl.ti.com/lprf/ble5stack-latest/

			BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 0, 0);
			// Create Advertisement set #1 and assign handle
			status = GapAdv_create(&SimplePeripheral_advCallback, &advParams1,
					&advHandleLegacy);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Load advertising data for set #1 that is statically allocated by the app
			status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
					sizeof(advData1), advData1);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Load scan response data for set #1 that is statically allocated by the app
			status = GapAdv_loadByHandle(advHandleLegacy,
					GAP_ADV_DATA_TYPE_SCAN_RSP, sizeof(scanResData1),
					scanResData1);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Set event mask for set #1
			status = GapAdv_setEventMask(advHandleLegacy,
					GAP_ADV_EVT_MASK_START_AFTER_ENABLE
							| GAP_ADV_EVT_MASK_END_AFTER_DISABLE
							| GAP_ADV_EVT_MASK_SET_TERMINATED);

			// Enable legacy advertising for set #1
			status = GapAdv_enable(advHandleLegacy,
					GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 1, 0);
			// Create Advertisement set #2 and assign handle
			status = GapAdv_create(&SimplePeripheral_advCallback, &advParams2,
					&advHandleLongRange);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Load advertising data for set #2 that is statically allocated by the app
			status = GapAdv_loadByHandle(advHandleLongRange,
					GAP_ADV_DATA_TYPE_ADV, sizeof(advData2), advData2);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Set event mask for set #2
			status = GapAdv_setEventMask(advHandleLongRange,
					GAP_ADV_EVT_MASK_START_AFTER_ENABLE
							| GAP_ADV_EVT_MASK_END_AFTER_DISABLE
							| GAP_ADV_EVT_MASK_SET_TERMINATED);

			BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GapAdv_enable", 0);
			// Enable long range advertising for set #2
			status = GapAdv_enable(advHandleLongRange,
					GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Display device address
//        Display_printf(dispHandle, SP_ROW_IDA, 0, "%s Addr: %s",
//                       (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
//                       Util_convertBdAddr2Str(pPkt->devAddr));

			if (addrMode > ADDRMODE_RANDOM) {
				SimplePeripheral_updateRPA();

				// Create one-shot clock for RPA check event.
				Util_constructClock(&clkRpaRead, SimplePeripheral_clockHandler,
				READ_RPA_PERIOD, 0, true, (UArg) &argRpaRead);
			}
		}

		break;
	}

	case GAP_LINK_ESTABLISHED_EVENT: {
		gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t*) pMsg;

		BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_LINK_ESTABLISHED_EVENT", 0);
		// Display the amount of current connections
		uint8_t numActive = linkDB_NumActive("");
//      Display_printf(dispHandle, SP_ROW_STATUS_2, 0, "Num Conns: %d",
//                     (uint16_t)numActive);

		if (pPkt->hdr.status == SUCCESS) {
			Util_stopClock(&clkESLOAdvSleep); // stop advSleep duty cycle

			// Add connection to list and start RSSI
			SimplePeripheral_addConn(pPkt->connectionHandle);

			// Display the address of this connection
//        Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connected to %s",
//                       Util_convertBdAddr2Str(pPkt->devAddr));
			isPaired = true;
			// !! temporary: turn off LED once connected
			uint8_t setGPIO = 0x00;
			SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
					&setGPIO); // only sets parameter, does not cue LED change
			GPIO_write(LED_0, setGPIO);

			// recover old settings
			mapEsloSettings(esloSettingsSleep);

			// Start Periodic Clock.
			Util_startClock(&clkNotifyVitals);
		}
		if ((numActive < MAX_NUM_BLE_CONNS)
				&& (autoConnect == AUTOCONNECT_DISABLE)) {
			// Start advertising since there is room for more connections
			GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
			GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX,
					0);
		} else {
			// Stop advertising since there is no room for more connections
			GapAdv_disable(advHandleLongRange);
			GapAdv_disable(advHandleLegacy);
		}
		break;
	}

	case GAP_LINK_TERMINATED_EVENT: {
		gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t*) pMsg;

		// Display the amount of current connections
		uint8_t numActive = linkDB_NumActive("");
//      Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Device Disconnected!");
//      Display_printf(dispHandle, SP_ROW_STATUS_2, 0, "Num Conns: %d",
//                     (uint16_t)numActive);

		// Remove the connection from the list and disable RSSI if needed
		SimplePeripheral_removeConn(pPkt->connectionHandle);

		// If no active connections
		if (numActive == 0) {
			// Stop periodic clock
			Util_stopClock(&clkNotifyVitals);
			isPaired = false;

			// always save, if device is not sleeping they will have no effect when reloaded
			memcpy(esloSettingsSleep, esloSettings,
			SIMPLEPROFILE_CHAR3_LEN);
			if (esloSettings[Set_SleepWake] == ESLO_MODULE_OFF) {
				esloSleep();
			}

			BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : GAP msg: status=%d, opcode=%s\n", 0, "GAP_LINK_TERMINATED_EVENT");

			isAsleep = 1;
			if (esloSettings[Set_AdvLong] > 0x00) { // long
				Util_startClock(&clkESLOAdvSleep);
			} else { // keep advertising on at full rate
				GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX,
						0);
				GapAdv_enable(advHandleLongRange,
						GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
			}

		}

		break;
	}

	case GAP_UPDATE_LINK_PARAM_REQ_EVENT: {
		gapUpdateLinkParamReqReply_t rsp;

		gapUpdateLinkParamReqEvent_t *pReq =
				(gapUpdateLinkParamReqEvent_t*) pMsg;

		rsp.connectionHandle = pReq->req.connectionHandle;
		rsp.signalIdentifier = pReq->req.signalIdentifier;

		// Only accept connection intervals with slave latency of 0
		// This is just an example of how the application can send a response
		if (pReq->req.connLatency == 0) {
			rsp.intervalMin = pReq->req.intervalMin;
			rsp.intervalMax = pReq->req.intervalMax;
			rsp.connLatency = pReq->req.connLatency;
			rsp.connTimeout = pReq->req.connTimeout;
			rsp.accepted = TRUE;
		} else {
			rsp.accepted = FALSE;
		}

		// Send Reply
		VOID GAP_UpdateLinkParamReqReply(&rsp);

		break;
	}

	case GAP_LINK_PARAM_UPDATE_EVENT: {
		gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t*) pMsg;

		// Get the address from the connection handle
		linkDBInfo_t linkInfo;
		linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

		if (pPkt->status == SUCCESS) {
			// Display the address of the connection update
//        Display_printf(dispHandle, SP_ROW_STATUS_2, 0, "Link Param Updated: %s",
//                       Util_convertBdAddr2Str(linkInfo.addr));
		} else {
			// Display the address of the connection update failure
//        Display_printf(dispHandle, SP_ROW_STATUS_2, 0,
//                       "Link Param Update Failed 0x%x: %s", pPkt->opcode,
//                       Util_convertBdAddr2Str(linkInfo.addr));
		}

		// Check if there are any queued parameter updates
		spConnHandleEntry_t *connHandleEntry = (spConnHandleEntry_t*) List_get(
				&paramUpdateList);
		if (connHandleEntry != NULL) {
			// Attempt to send queued update now
			SimplePeripheral_processParamUpdate(connHandleEntry->connHandle);

			// Free list element
			ICall_free(connHandleEntry);
		}

		break;
	}

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
    case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
    {
      linkDBInfo_t linkInfo;
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

      // Get the address from the connection handle
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      // Display the address of the connection update failure
//      Display_printf(dispHandle, SP_ROW_STATUS_2, 0,
//                     "Peer Device's Update Request Rejected 0x%x: %s", pPkt->opcode,
//                     Util_convertBdAddr2Str(linkInfo.addr));

      break;
    }
#endif

	default:
//      Display_clearLines(dispHandle, SP_ROW_STATUS_1, SP_ROW_STATUS_2);
		break;
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void SimplePeripheral_charValueChangeCB(uint8_t paramId) {
	uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

	if (pValue) {
		*pValue = paramId;

		if (SimplePeripheral_enqueueMsg(SP_CHAR_CHANGE_EVT, pValue) != SUCCESS) {
			ICall_free(pValue);
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId) {
	uint8_t len;
	bStatus_t ret;

	switch (paramId) {
	case SIMPLEPROFILE_CHAR1:
		len = SIMPLEPROFILE_CHAR1_LEN;
		break;
	case SIMPLEPROFILE_CHAR3:
		len = SIMPLEPROFILE_CHAR3_LEN;
		break;
	case SIMPLEPROFILE_CHAR6:
		len = SIMPLEPROFILE_CHAR6_LEN;
		break;
	default:
		break;
	}

	uint8_t *pValue = ICall_malloc(len); // dynamic allocation

	switch (paramId) {
// only characteristics with GATT_PROP_WRITE, all others are written elsewhere
	case SIMPLEPROFILE_CHAR1:
		ret = SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, pValue);
		GPIO_write(LED_0, pValue[0]);
		break;
	case SIMPLEPROFILE_CHAR3:
		ret = SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, pValue);
		mapEsloSettings(pValue);
		break;
	case SIMPLEPROFILE_CHAR6:
		ret = SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR6, pValue);
		memcpy(&esloExportBlock, pValue, sizeof(uint32_t));
		SimplePeripheral_enqueueMsg(ES_EXPORT_DATA, NULL);
		break;
	default:
		// should not reach here!
		break;
	}
	if (ret) {
		ICall_free(pValue);
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_notifyVitals
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_notifyVitals(void) {
	uint8_t *pValue = ICall_malloc(SIMPLEPROFILE_CHAR2_LEN);
	readBatt();
	readTherm();

	if (lowVoltage == 0 || vbatt_uV < lowVoltage) {
		lowVoltage = vbatt_uV;
	}
	ESLO_compileVitals(&vbatt_uV, &lowVoltage, &temp_uC, &esloAddr, pValue);
	SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN,
			pValue);
	if (ret) {
		ICall_free(pValue);
	}
}

static void ESLO_performPeriodicTask() {
	eslo_dt eslo;

	Watchdog_clear(watchdogHandle);

	absoluteTime += (ES_PERIODIC_EVT_PERIOD / 1000);
	eslo.type = Type_AbsoluteTime;
	eslo.data = absoluteTime;
	ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);

	readBatt();
	eslo.type = Type_BatteryVoltage;
	eslo.data = vbatt_uV;
	ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);

	readTherm();
	eslo.type = Type_Therm;
	eslo.data = temp_uC;
	ret = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);

	esloUpdateNVS(); // save esloAddress to recover session

// test multiple times in case of outlier?
	if (vbatt_uV < V_DROPOUT || esloAddr >= FLASH_SIZE) {
		esloSleep(); // good night
		Util_stopClock(&clkESLOPeriodic); // never come back unless user initiates it
		// set parameter to notify condition?
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_updateRPA(void) {
	uint8_t *pRpaNew;

// Read the current RPA.
	pRpaNew = GAP_GetDevAddress(FALSE);

	if (memcmp(pRpaNew, rpa, B_ADDR_LEN)) {
		memcpy(rpa, pRpaNew, B_ADDR_LEN);
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimplePeripheral_clockHandler(UArg arg) {
	spClockEventData_t *pData = (spClockEventData_t*) arg;

	if (pData->event == SP_PERIODIC_EVT) {
		// Start the next period
		Util_startClock(&clkNotifyVitals);
		// Post event to wake up the application
		SimplePeripheral_enqueueMsg(SP_PERIODIC_EVT, NULL);
	} else if (pData->event == ES_PERIODIC_EVT) {
		// Start the next period
		Util_startClock(&clkESLOPeriodic);
		// Post event to wake up the application
		SimplePeripheral_enqueueMsg(ES_PERIODIC_EVT, NULL);
	} else if (pData->event == ES_AXY_EVT) {
		// Start the next period
		Util_startClock(&clkESLOAxy);
		// Post event to wake up the application
		SimplePeripheral_enqueueMsg(ES_AXY_EVT, NULL);
	} else if (pData->event == SP_READ_RPA_EVT) {
		// Start the next period
		Util_startClock(&clkRpaRead);
		// Post event to read the current RPA
		SimplePeripheral_enqueueMsg(SP_READ_RPA_EVT, NULL);
	} else if (pData->event == ES_ADV_SLEEP) {
		// Start the next period
		Util_startClock(&clkESLOAdvSleep);
		// Post event to wake up the application
		SimplePeripheral_enqueueMsg(ES_ADV_SLEEP, NULL);
	} else if (pData->event == SP_SEND_PARAM_UPDATE_EVT) {
		// Send message to app
		SimplePeripheral_enqueueMsg(SP_SEND_PARAM_UPDATE_EVT, pData);
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf,
		uintptr_t arg) {
	spGapAdvEventData_t *pData = ICall_malloc(sizeof(spGapAdvEventData_t));

	if (pData) {
		pData->event = event;
		pData->pBuf = pBuf;

		if (SimplePeripheral_enqueueMsg(SP_ADV_EVT, pData) != SUCCESS) {
			ICall_free(pData);
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData) {
	switch (pEventData->event) {
	case GAP_EVT_ADV_START_AFTER_ENABLE :
		BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GAP_EVT_ADV_START_AFTER_ENABLE", 0);
//      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d Enabled",
//                     *(uint8_t *)(pEventData->pBuf));
		break;

	case GAP_EVT_ADV_END_AFTER_DISABLE :
//      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d Disabled",
//                     *(uint8_t *)(pEventData->pBuf));
		break;

	case GAP_EVT_ADV_START :
		break;

	case GAP_EVT_ADV_END :
		break;

	case GAP_EVT_ADV_SET_TERMINATED : {
#ifndef Display_DISABLE_ALL
//      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);
//      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d disabled after conn %d",
//                     advSetTerm->handle, advSetTerm->connHandle );
#endif
	}
		break;

	case GAP_EVT_SCAN_REQ_RECEIVED :
		break;

	case GAP_EVT_INSUFFICIENT_MEMORY :
		break;

	default:
		break;
	}

// All events have associated memory to free except the insufficient memory
// event
	if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY) {
		ICall_free(pEventData->pBuf);
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
		uint8_t status) {
	spPairStateData_t *pData = ICall_malloc(sizeof(spPairStateData_t));

// Allocate space for the event data.
	if (pData) {
		pData->state = state;
		pData->connHandle = connHandle;
		pData->status = status;

		// Queue the event.
		if (SimplePeripheral_enqueueMsg(SP_PAIR_STATE_EVT, pData) != SUCCESS) {
			ICall_free(pData);
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr,
		uint16_t connHandle, uint8_t uiInputs, uint8_t uiOutputs,
		uint32_t numComparison) {
	spPasscodeData_t *pData = ICall_malloc(sizeof(spPasscodeData_t));

// Allocate space for the passcode event.
	if (pData) {
		pData->connHandle = connHandle;
		memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
		pData->uiInputs = uiInputs;
		pData->uiOutputs = uiOutputs;
		pData->numComparison = numComparison;

		// Enqueue the event.
		if (SimplePeripheral_enqueueMsg(SP_PASSCODE_EVT, pData) != SUCCESS) {
			ICall_free(pData);
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimplePeripheral_processPairState(spPairStateData_t *pPairData) {
	uint8_t state = pPairData->state;
	uint8_t status = pPairData->status;

	switch (state) {
	case GAPBOND_PAIRING_STATE_STARTED:
//      Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Pairing started");
		break;

	case GAPBOND_PAIRING_STATE_COMPLETE:
		if (status == SUCCESS) {
//        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Pairing success");
		} else {
//        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Pairing fail: %d", status);
		}
		break;

	case GAPBOND_PAIRING_STATE_ENCRYPTED:
		if (status == SUCCESS) {
//        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Encryption success");
		} else {
//        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Encryption failed: %d", status);
		}
		break;

	case GAPBOND_PAIRING_STATE_BOND_SAVED:
		if (status == SUCCESS) {
//        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Bond save success");
		} else {
//        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Bond save failed: %d", status);
		}
		break;

	default:
		break;
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData) {
// Display passcode to user
	if (pPasscodeData->uiOutputs != 0) {
//    Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Passcode: %d",
//                   B_APP_DEFAULT_PASSCODE);
	}

// Send passcode response
	GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle, SUCCESS,
			B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      SimplePeripheral_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport) {
// Enqueue the event for processing in the app context.
	if (SimplePeripheral_enqueueMsg(SP_CONN_EVT, pReport) != SUCCESS) {
		ICall_free(pReport);
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport) {
// Get index from handle
	uint8_t connIndex = SimplePeripheral_getConnIndex(pReport->handle);

	if (connIndex >= MAX_NUM_BLE_CONNS) {
//    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
		return;
	}

// If auto phy change is enabled
	if (connList[connIndex].isAutoPHYEnable == TRUE) {
		// Read the RSSI
		HCI_ReadRssiCmd(pReport->handle);
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData) {
	uint8_t success;
	spEvt_t *pMsg = ICall_malloc(sizeof(spEvt_t));

// Create dynamic pointer to message.
	if (pMsg) {
		pMsg->event = event;
		pMsg->pData = pData;

		// Enqueue the message.
		success = Util_enqueueMsg(appMsgQueueHandle, syncEvent,
				(uint8_t*) pMsg);
		return (success) ? SUCCESS : FAILURE;
	}

	return (bleMemAllocError);
}

/*********************************************************************
 * @fn      SimplePeripheral_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_addConn(uint16_t connHandle) {
	uint8_t i;
	uint8_t status = bleNoResources;

// Try to find an available entry
	for (i = 0; i < MAX_NUM_BLE_CONNS; i++) {
		if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID) {
			// Found available entry to put a new connection info in
			connList[i].connHandle = connHandle;

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
			// Allocate data to send through clock handler
			connList[i].pParamUpdateEventData = ICall_malloc(
					sizeof(spClockEventData_t) + sizeof(uint16_t));
			if (connList[i].pParamUpdateEventData) {
				connList[i].pParamUpdateEventData->event =
				SP_SEND_PARAM_UPDATE_EVT;
				*((uint16_t*) connList[i].pParamUpdateEventData->data) =
						connHandle;

				// Create a clock object and start
				connList[i].pUpdateClock = (Clock_Struct*) ICall_malloc(
						sizeof(Clock_Struct));

				if (connList[i].pUpdateClock) {
					Util_constructClock(connList[i].pUpdateClock,
							SimplePeripheral_clockHandler,
							SEND_PARAM_UPDATE_DELAY, 0, true,
							(UArg) (connList[i].pParamUpdateEventData));
				} else {
					ICall_free(connList[i].pParamUpdateEventData);
				}
			} else {
				status = bleMemAllocError;
			}
#endif

			// Set default PHY to 1M
			connList[i].currPhy = HCI_PHY_1_MBPS;

			break;
		}
	}

	return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle) {
	uint8_t i;

	for (i = 0; i < MAX_NUM_BLE_CONNS; i++) {
		if (connList[i].connHandle == connHandle) {
			return i;
		}
	}

	return (MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle) {
	uint8_t i;
// Set to invalid connection index initially
	uint8_t connIndex = MAX_NUM_BLE_CONNS;

	if (connHandle != LINKDB_CONNHANDLE_ALL) {
		// Get connection index from handle
		connIndex = SimplePeripheral_getConnIndex(connHandle);
		if (connIndex >= MAX_NUM_BLE_CONNS) {
			return (bleInvalidRange);
		}
	}

// Clear specific handle or all handles
	for (i = 0; i < MAX_NUM_BLE_CONNS; i++) {
		if ((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL)) {
			connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
			connList[i].currPhy = 0;
			connList[i].phyCngRq = 0;
			connList[i].phyRqFailCnt = 0;
			connList[i].rqPhy = 0;
			memset(connList[i].rssiArr, 0, SP_MAX_RSSI_STORE_DEPTH);
			connList[i].rssiAvg = 0;
			connList[i].rssiCntr = 0;
			connList[i].isAutoPHYEnable = FALSE;
		}
	}

	return (SUCCESS);
}

/*********************************************************************
 * @fn      SimplePeripheral_clearPendingParamUpdate
 *
 * @brief   clean pending param update request in the paramUpdateList list
 *
 * @param   connHandle - connection handle to clean
 *
 * @return  none
 */
void SimplePeripheral_clearPendingParamUpdate(uint16_t connHandle) {
	List_Elem *curr;

	for (curr = List_head(&paramUpdateList); curr != NULL;
			curr = List_next(curr)) {
		if (((spConnHandleEntry_t*) curr)->connHandle == connHandle) {
			List_remove(&paramUpdateList, curr);
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle) {
	uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);

	if (connIndex != MAX_NUM_BLE_CONNS) {
		Clock_Struct *pUpdateClock = connList[connIndex].pUpdateClock;

		if (pUpdateClock != NULL) {
			// Stop and destruct the RTOS clock if it's still alive
			if (Util_isActive(pUpdateClock)) {
				Util_stopClock(pUpdateClock);
			}

			// Destruct the clock object
			Clock_destruct(pUpdateClock);
			// Free clock struct
			ICall_free(pUpdateClock);
			// Free ParamUpdateEventData
			ICall_free(connList[connIndex].pParamUpdateEventData);
		}
		// Clear pending update requests from paramUpdateList
		SimplePeripheral_clearPendingParamUpdate(connHandle);
		// Stop Auto PHY Change
		SimplePeripheral_stopAutoPhyChange(connHandle);
		// Clear Connection List Entry
		SimplePeripheral_clearConnListEntry(connHandle);
	}

	return connIndex;
}

/*********************************************************************
 * @fn      SimplePeripheral_processParamUpdate
 *
 * @brief   Process a parameters update request
 *
 * @return  None
 */
static void SimplePeripheral_processParamUpdate(uint16_t connHandle) {
	gapUpdateLinkParamReq_t req;
	uint8_t connIndex;

	req.connectionHandle = connHandle;
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
	req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
	req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
	req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
	req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
#endif

	connIndex = SimplePeripheral_getConnIndex(connHandle);
	if (connIndex >= MAX_NUM_BLE_CONNS) {
//    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
		return;
	}

// Deconstruct the clock object
	Clock_destruct(connList[connIndex].pUpdateClock);
// Free clock struct, only in case it is not NULL
	if (connList[connIndex].pUpdateClock != NULL) {
		ICall_free(connList[connIndex].pUpdateClock);
		connList[connIndex].pUpdateClock = NULL;
	}
// Free ParamUpdateEventData, only in case it is not NULL
	if (connList[connIndex].pParamUpdateEventData != NULL)
		ICall_free(connList[connIndex].pParamUpdateEventData);

// Send parameter update
	bStatus_t status = GAP_UpdateLinkParamReq(&req);

// If there is an ongoing update, queue this for when the udpate completes
	if (status == bleAlreadyInRequestedMode) {
		spConnHandleEntry_t *connHandleEntry = ICall_malloc(
				sizeof(spConnHandleEntry_t));
		if (connHandleEntry) {
			connHandleEntry->connHandle = connHandle;

			List_put(&paramUpdateList, (List_Elem*) connHandleEntry);
		}
	}
}

/*********************************************************************
 * @fn      SimpleCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg) {
	uint8_t status = pMsg->pReturnParam[0];

//Find which command this command complete is for
	switch (pMsg->cmdOpcode) {
	case HCI_READ_RSSI: {
		int8 rssi = (int8) pMsg->pReturnParam[3];

		// Display RSSI value, if RSSI is higher than threshold, change to faster PHY
		if (status == SUCCESS) {
			uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1],
					pMsg->pReturnParam[2]);

			uint8_t index = SimplePeripheral_getConnIndex(handle);
			if (index >= MAX_NUM_BLE_CONNS) {
//          Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
				return;
			}

			if (rssi != LL_RSSI_NOT_AVAILABLE) {
				connList[index].rssiArr[connList[index].rssiCntr++] = rssi;
				connList[index].rssiCntr %= SP_MAX_RSSI_STORE_DEPTH;

				int16_t sum_rssi = 0;
				for (uint8_t cnt = 0; cnt < SP_MAX_RSSI_STORE_DEPTH; cnt++) {
					sum_rssi += connList[index].rssiArr[cnt];
				}
				connList[index].rssiAvg = (uint32_t) (sum_rssi
						/ SP_MAX_RSSI_STORE_DEPTH);

				uint8_t phyRq = SP_PHY_NONE;
				uint8_t phyRqS = SP_PHY_NONE;
				uint8_t phyOpt = LL_PHY_OPT_NONE;

				if (connList[index].phyCngRq == FALSE) {
					if ((connList[index].rssiAvg >= RSSI_2M_THRSHLD)
							&& (connList[index].currPhy != HCI_PHY_2_MBPS)
							&& (connList[index].currPhy != SP_PHY_NONE)) {
						// try to go to higher data rate
						phyRqS = phyRq = HCI_PHY_2_MBPS;
					} else if ((connList[index].rssiAvg < RSSI_2M_THRSHLD)
							&& (connList[index].rssiAvg >= RSSI_1M_THRSHLD)
							&& (connList[index].currPhy != HCI_PHY_1_MBPS)
							&& (connList[index].currPhy != SP_PHY_NONE)) {
						// try to go to legacy regular data rate
						phyRqS = phyRq = HCI_PHY_1_MBPS;
					} else if ((connList[index].rssiAvg >= RSSI_S2_THRSHLD)
							&& (connList[index].rssiAvg < RSSI_1M_THRSHLD)
							&& (connList[index].currPhy != SP_PHY_NONE)) {
						// try to go to lower data rate S=2(500kb/s)
						phyRqS = HCI_PHY_CODED;
						phyOpt = LL_PHY_OPT_S2;
						phyRq = BLE5_CODED_S2_PHY;
					} else if (connList[index].rssiAvg < RSSI_S2_THRSHLD) {
						// try to go to lowest data rate S=8(125kb/s)
						phyRqS = HCI_PHY_CODED;
						phyOpt = LL_PHY_OPT_S8;
						phyRq = BLE5_CODED_S8_PHY;
					}
					if ((phyRq != SP_PHY_NONE) &&
					// First check if the request for this phy change is already not honored then don't request for change
							(((connList[index].rqPhy == phyRq)
									&& (connList[index].phyRqFailCnt < 2))
									|| (connList[index].rqPhy != phyRq))) {
						//Initiate PHY change based on RSSI
						SimplePeripheral_setPhy(connList[index].connHandle, 0,
								phyRqS, phyRqS, phyOpt);
						connList[index].phyCngRq = TRUE;

						// If it a request for different phy than failed request, reset the count
						if (connList[index].rqPhy != phyRq) {
							// then reset the request phy counter and requested phy
							connList[index].phyRqFailCnt = 0;
						}

						if (phyOpt == LL_PHY_OPT_NONE) {
							connList[index].rqPhy = phyRq;
						} else if (phyOpt == LL_PHY_OPT_S2) {
							connList[index].rqPhy = BLE5_CODED_S2_PHY;
						} else {
							connList[index].rqPhy = BLE5_CODED_S8_PHY;
						}

					} // end of if ((phyRq != SP_PHY_NONE) && ...
				} // end of if (connList[index].phyCngRq == FALSE)
			} // end of if (rssi != LL_RSSI_NOT_AVAILABLE)

//        Display_printf(dispHandle, SP_ROW_RSSI, 0,
//                       "RSSI:%d dBm, AVG RSSI:%d dBm",
//                       (uint32_t)(rssi),
//                       connList[index].rssiAvg);

		} // end of if (status == SUCCESS)
		break;
	}

	case HCI_LE_READ_PHY: {
		if (status == SUCCESS) {
//        Display_printf(dispHandle, SP_ROW_RSSI + 2, 0, "RXPh: %d, TXPh: %d",
//                       pMsg->pReturnParam[3], pMsg->pReturnParam[4]);
		}
		break;
	}

	default:
		break;
	} // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
 * @fn      SimplePeripheral_initPHYRSSIArray
 *
 * @brief   Initializes the array of structure/s to store data related
 *          RSSI based auto PHy change
 *
 * @param   connHandle - the connection handle
 *
 * @param   addr - pointer to device address
 *
 * @return  index of connection handle
 */
static void SimplePeripheral_initPHYRSSIArray(void) {
//Initialize array to store connection handle and RSSI values
	memset(connList, 0, sizeof(connList));
	for (uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++) {
		connList[index].connHandle = SP_INVALID_HANDLE;
	}
}
/*********************************************************************
 // Set default PHY to 1M
 * @fn      SimplePeripheral_startAutoPhyChange
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   devAddr - device address
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static status_t SimplePeripheral_startAutoPhyChange(uint16_t connHandle) {
	status_t status = FAILURE;

// Get connection index from handle
	uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
	SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

// Start Connection Event notice for RSSI calculation
	status = Gap_RegisterConnEventCb(SimplePeripheral_connEvtCB,
			GAP_CB_REGISTER, connHandle);

// Flag in connection info if successful
	if (status == SUCCESS) {
		connList[connIndex].isAutoPHYEnable = TRUE;
	}

	return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_stopAutoPhyChange
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle) {
// Get connection index from handle
	uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
	SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

// Stop connection event notice
	Gap_RegisterConnEventCb(NULL, GAP_CB_UNREGISTER, connHandle);

// Also update the phychange request status for active RSSI tracking connection
	connList[connIndex].phyCngRq = FALSE;
	connList[connIndex].isAutoPHYEnable = FALSE;

	return SUCCESS;
}

/*********************************************************************
 * @fn      SimplePeripheral_setPhy
 *
 * @brief   Call the HCI set phy API and and add the handle to a
 *          list to match it to an incoming command status event
 */
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
		uint8_t txPhy, uint8_t rxPhy, uint16_t phyOpts) {
// Allocate list entry to store handle for command status
	spConnHandleEntry_t *connHandleEntry = ICall_malloc(
			sizeof(spConnHandleEntry_t));

	if (connHandleEntry) {
		connHandleEntry->connHandle = connHandle;

		// Add entry to the phy command status list
		List_put(&setPhyCommStatList, (List_Elem*) connHandleEntry);

		// Send PHY Update
		HCI_LE_SetPhyCmd(connHandle, allPhys, txPhy, rxPhy, phyOpts);
	}

	return SUCCESS;
}

/*********************************************************************
 * @fn      SimplePeripheral_updatePHYStat
 *
 * @brief   Update the auto phy update state machine
 *
 * @param   connHandle - the connection handle
 *
 * @return  None
 */
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg) {
	uint8_t connIndex;

	switch (eventCode) {
	case HCI_LE_SET_PHY: {
		// Get connection handle from list
		spConnHandleEntry_t *connHandleEntry = (spConnHandleEntry_t*) List_get(
				&setPhyCommStatList);

		if (connHandleEntry) {
			// Get index from connection handle
			connIndex = SimplePeripheral_getConnIndex(
					connHandleEntry->connHandle);

			ICall_free(connHandleEntry);

			// Is this connection still valid?
			if (connIndex < MAX_NUM_BLE_CONNS) {
				hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t*) pMsg;

				if (pMyMsg->cmdStatus
						== HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE) {
					// Update the phychange request status for active RSSI tracking connection
					connList[connIndex].phyCngRq = FALSE;
					connList[connIndex].phyRqFailCnt++;
				}
			}
		}
		break;
	}

		// LE Event - a Phy update has completed or failed
	case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT: {
		hciEvt_BLEPhyUpdateComplete_t *pPUC =
				(hciEvt_BLEPhyUpdateComplete_t*) pMsg;

		if (pPUC) {
			// Get index from connection handle
			connIndex = SimplePeripheral_getConnIndex(pPUC->connHandle);

			// Is this connection still valid?
			if (connIndex < MAX_NUM_BLE_CONNS) {
				// Update the phychange request status for active RSSI tracking connection
				connList[connIndex].phyCngRq = FALSE;

				if (pPUC->status == SUCCESS) {
					connList[connIndex].currPhy = pPUC->rxPhy;
				}
				if (pPUC->rxPhy != connList[connIndex].rqPhy) {
					connList[connIndex].phyRqFailCnt++;
				} else {
					// Reset the request phy counter and requested phy
					connList[connIndex].phyRqFailCnt = 0;
					connList[connIndex].rqPhy = 0;
				}
			}
		}

		break;
	}

	default:
		break;
	} // end of switch (eventCode)
}
