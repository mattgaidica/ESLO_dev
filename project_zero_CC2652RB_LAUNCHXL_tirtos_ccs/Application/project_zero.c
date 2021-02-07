/******************************************************************************

 @file  project_zero.c

 @brief This file contains the Project Zero sample application for use
 with the CC2650 Bluetooth Low Energy Protocol Stack.

 !! Highly modified for ESLO
 Notes
 _____
 - http://software-dl.ti.com/lprf/simplelink_academy/modules/ble_01_custom_profile/ble_01_custom_profile.html
 - file:///Users/matt/ti/simplelink_academy_cc13x2_26x2sdk_4_30_01_00/modules/ble5stack/ble_01_basic/ble_01_basic.html

 Group: WCS, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#if !(defined __TI_COMPILER_VERSION__)
#include <intrinsics.h>
#endif

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/utils/List.h>

#include <ti/drivers/ADC.h>
#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>

#include <xdc/runtime/Log.h>
#include <ti/display/AnsiColor.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

#include <icall.h>
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

/* Bluetooth Profiles */
#include <devinfoservice.h>
#include <led_service.h>
#include <data_service.h>

/* Application specific includes */
#include <ti_drivers_config.h>

#include <project_zero.h>
#include "ti_ble_config.h"
#include <util.h>

/*********************************************************************
 * MACROS
 */

// Spin if the expression is not true
#define APP_ASSERT(expr) if(!(expr)) {project_zero_spin();}

#define UTIL_ARRTOHEX_REVERSE     1
#define UTIL_ARRTOHEX_NO_REVERSE  0

/*********************************************************************
 * CONSTANTS
 */
// Task configuration
#define PZ_TASK_PRIORITY                     1

#ifndef PZ_TASK_STACK_SIZE
#define PZ_TASK_STACK_SIZE                   2048
#endif

// Internal Events for RTOS application
#define PZ_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define PZ_APP_MSG_EVT                       Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define PZ_ALL_EVENTS                        (PZ_ICALL_EVT | PZ_APP_MSG_EVT)

// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
#define PZ_SERVICE_WRITE_EVT     0  /* A characteristic value has been written     */
#define PZ_SERVICE_CFG_EVT       1  /* A characteristic configuration has changed  */
#define PZ_UPDATE_CHARVAL_EVT    2  /* Request from ourselves to update a value    */
#define PZ_BUTTON_DEBOUNCED_EVT  3  /* A button has been debounced with new value  */
#define PZ_PAIRSTATE_EVT         4  /* The pairing state is updated                */
#define PZ_PASSCODE_EVT          5  /* A pass-code/PIN is requested during pairing */
#define PZ_ADV_EVT               6  /* A subscribed advertisement activity         */
#define PZ_START_ADV_EVT         7  /* Request advertisement start from task ctx   */
#define PZ_SEND_PARAM_UPD_EVT    8  /* Request parameter update req be sent        */
#define PZ_CONN_EVT              9  /* Connection Event End notice                 */
#define PZ_READ_RPA_EVT         10  /* Read RPA event                              */
#define NOTIFY_ESLO_EVT			11
#define NOTIFY_EEG_EVT			12

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// Connection interval conversion rate to miliseconds
#define CONN_INTERVAL_MS_CONVERSION           1.25

/*********************************************************************
 * TYPEDEFS
 */
// Struct for messages sent to the application task
typedef struct { uint8_t event; void *pData; } pzMsg_t;

// Struct for messages about characteristic data
		typedef struct { uint16_t svcUUID; // UUID of the service
		uint16_t dataLen;//
		uint8_t paramID;// Index of the characteristic
		uint8_t data[];// Flexible array member, extended to malloc - sizeof(.)
		} pzCharacteristicData_t;

// Struct for message about sending/requesting passcode from peer.
		typedef struct { uint16_t connHandle; uint8_t uiInputs; uint8_t uiOutputs; uint32_t numComparison; } pzPasscodeReq_t;

// Struct for message about a pending parameter update request.
		typedef struct { uint16_t connHandle; } pzSendParamReq_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
		typedef struct { uint8_t state; uint16_t connHandle; uint8_t status; } pzPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
		typedef struct { uint8_t deviceAddr[B_ADDR_LEN]; uint16_t connHandle; uint8_t uiInputs; uint8_t uiOutputs; uint32_t numComparison; } pzPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
		typedef struct { uint32_t event; void *pBuf; } pzGapAdvEventData_t;

// List element for parameter update and PHY command status lists
		typedef struct { List_Elem elem; uint16_t *connHandle; } pzConnHandleEntry_t;

// Connected device information
		typedef struct { uint16_t connHandle;               // Connection Handle
		Clock_Struct* pUpdateClock;// pointer to clock struct
		bool phyCngRq;// Set to true if PHY change request is in progress
		uint8_t currPhy;// The active PHY for a connection
		uint8_t rqPhy;// The requested PHY for a connection
		uint8_t phyRqFailCnt;// PHY change request fail count
		} pzConnRec_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
		typedef struct { uint8_t event; uint8_t data[]; } pzClockEventData_t;

		/*********************************************************************
		 * GLOBAL VARIABLES
		 */
// Task configuration
		Task_Struct pzTask;

		uint8_t appTaskStack[PZ_TASK_STACK_SIZE];

		/*********************************************************************
		 * LOCAL VARIABLES
		 */

// Entity ID globally used to check for source and/or destination of messages
		static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
		static ICall_SyncHandle syncEvent;

// Queue object used for app messages
		static Queue_Struct appMsgQueue; static Queue_Handle appMsgQueueHandle;

// Advertising handles
		static uint8_t advHandleLegacy;

// Per-handle connection info
		static pzConnRec_t connList[MAX_NUM_BLE_CONNS];

// List to store connection handles for set phy command status's
		static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
		static List_List paramUpdateList;

// Clock instance for RPA read events.
		static Clock_Struct clkRpaRead;

// Address mode
		static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
		static uint8 rpaAddr[B_ADDR_LEN] = { 0 };

		/*********************************************************************
		 * LOCAL FUNCTIONS
		 */

		/* Task functions */
		static void ProjectZero_init(void); static void ProjectZero_taskFxn(UArg a0, UArg a1);

		/* Event message processing functions */
		static void ProjectZero_processStackEvent(uint32_t stack_event); static void ProjectZero_processApplicationMessage(pzMsg_t *pMsg); static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg); static void ProjectZero_processGapMessage(gapEventHdr_t *pMsg); static void ProjectZero_processHCIMsg(ICall_HciExtEvt *pMsg); static void ProjectZero_processPairState(pzPairStateData_t *pPairState); static void ProjectZero_processPasscode(pzPasscodeReq_t *pReq); static void ProjectZero_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg); static void ProjectZero_processAdvEvent(pzGapAdvEventData_t *pEventData);

		/* Profile value change handlers */
		static void ProjectZero_updateCharVal(pzCharacteristicData_t *pCharData); static void ProjectZero_LedService_ValueChangeHandler( pzCharacteristicData_t *pCharData); static void ProjectZero_DataService_ValueChangeHandler( pzCharacteristicData_t *pCharData); static void ProjectZero_DataService_CfgChangeHandler( pzCharacteristicData_t *pCharData);

		/* Stack or profile callback function */
		static void ProjectZero_advCallback(uint32_t event, void *pBuf, uintptr_t arg); static void ProjectZero_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle, uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison); static void ProjectZero_pairStateCb(uint16_t connHandle, uint8_t state, uint8_t status);

		static void ProjectZero_LedService_ValueChangeCB(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue); static void ProjectZero_DataService_ValueChangeCB(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue); static void ProjectZero_DataService_CfgChangeCB(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);

		/* Connection handling functions */
		static uint8_t ProjectZero_getConnIndex(uint16_t connHandle); static uint8_t ProjectZero_clearConnListEntry(uint16_t connHandle); static uint8_t ProjectZero_addConn(uint16_t connHandle); static uint8_t ProjectZero_removeConn(uint16_t connHandle); static void ProjectZero_updatePHYStat(uint16_t eventCode, uint8_t *pMsg); static void ProjectZero_handleUpdateLinkParamReq( gapUpdateLinkParamReqEvent_t *pReq); static void ProjectZero_sendParamUpdate(uint16_t connHandle); static void ProjectZero_handleUpdateLinkEvent(gapLinkUpdateEvent_t *pEvt);
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
		static void ProjectZero_paramUpdClockHandler(UArg arg);
#endif
		static void ProjectZero_clockHandler(UArg arg); static void ProjectZero_processConnEvt(Gap_ConnEventRpt_t *pReport); // static void ProjectZero_connEvtCB(Gap_ConnEventRpt_t *pReport);

		/* Utility functions */
		static status_t ProjectZero_enqueueMsg(uint8_t event, void *pData); static char* util_arrtohex(uint8_t const *src, uint8_t src_len, uint8_t *dst, uint8_t dst_len, uint8_t reverse); static char* util_getLocalNameStr(const uint8_t *advData, uint8_t len);

		/*********************************************************************
		 * EXTERN FUNCTIONS
		 */
		extern void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);

		/*********************************************************************
		 * PROFILE CALLBACKS
		 */
// GAP Bond Manager Callbacks
		static gapBondCBs_t ProjectZero_BondMgrCBs = { ProjectZero_passcodeCb, // Passcode callback
		ProjectZero_pairStateCb// Pairing/Bonding state Callback
		};

		/*
		 * Callbacks in the user application for events originating from BLE services.
		 */
// LED Service callback handler.
// The type LED_ServiceCBs_t is defined in led_service.h
		static LedServiceCBs_t ProjectZero_LED_ServiceCBs = { .pfnChangeCb = ProjectZero_LedService_ValueChangeCB, // Characteristic value change callback handler
		.pfnCfgChangeCb = NULL,// No notification-/indication enabled chars in LED Service
		};

// Data Service callback handler.
// The type Data_ServiceCBs_t is defined in data_service.h
		static DataServiceCBs_t ProjectZero_Data_ServiceCBs = { .pfnChangeCb = ProjectZero_DataService_ValueChangeCB, // Characteristic value change callback handler
		.pfnCfgChangeCb = ProjectZero_DataService_CfgChangeCB
, // Noti/ind configuration callback handler
		};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
static Clock_Struct clkBTENotify;

void ProjectZero_clockHandler(UArg arg);

ADC_Handle adc;
ADC_Params adcParams;
int_fast16_t adcRes;
uint16_t adcValue0;
uint32_t adcValue0MicroVolt;

void ProjectZero_notifyHandler(UArg arg) {
	adcRes = ADC_convert(adc, &adcValue0);
	if (adcRes == ADC_STATUS_SUCCESS) {
		// read, multiply by 2 for voltage divider
		adcValue0MicroVolt = ADC_convertRawToMicroVolts(adc, adcValue0) * 2;
	}
	uint8_t advBuffer[4];
	memcpy(advBuffer, &adcValue0MicroVolt, sizeof(adcValue0MicroVolt));
//	ProjectZero_enqueueMsg(NOTIFY_ESLO_EVT, advBuffer);

	uint8_t eegBuffer[DS_EEG_LEN] = {0};
	int i;
	int j = 0;
	for (i = 0; i < DS_EEG_LEN - 1; i+=4) {
		eegBuffer[i] = j;
		j++;
	}
	ProjectZero_enqueueMsg(NOTIFY_EEG_EVT, eegBuffer); // EEG = 1, AXY = 2
}

/*********************************************************************
 * @fn     project_zero_spin
 *
 * @brief   Spin forever
 */
static void project_zero_spin(void) {
	volatile uint8_t x = 0;
	;

	while (1) {
		x++;
	}
}

/*********************************************************************
 * @fn      ProjectZero_createTask
 *
 * @brief   Task creation function for the Project Zero.
 */
void ProjectZero_createTask(void) {
	Task_Params taskParams;

// Configure task
	Task_Params_init(&taskParams);
	taskParams.stack = appTaskStack;
	taskParams.stackSize = PZ_TASK_STACK_SIZE;
	taskParams.priority = PZ_TASK_PRIORITY;

	Task_construct(&pzTask, ProjectZero_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      ProjectZero_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void ProjectZero_init(void) {
// ******************************************************************
// NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
// ******************************************************************
// Register the current thread as an ICall dispatcher application
// so that the application can send and receive messages.
	ICall_registerApp(&selfEntity, &syncEvent);

// Initialize queue for application messages.
// Note: Used to transfer control to application thread from e.g. interrupts.
	Queue_construct(&appMsgQueue, NULL);
	appMsgQueueHandle = Queue_handle(&appMsgQueue);

// ******************************************************************
// Hardware initialization
// ******************************************************************
	GPIO_init();
// Set the Device Name characteristic in the GAP GATT Service
// For more information, see the section in the User's Guide:
// http://software-dl.ti.com/lprf/ble5stack-latest/
	GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

// Configure GAP for param update
	{
		uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

		// Pass all parameter update requests to the app for it to decide
		GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
	}

// Setup the GAP Bond Manager. For more information see the GAP Bond Manager
// section in the User's Guide
	setBondManagerParameters();

// ******************************************************************
// BLE Service initialization
// ******************************************************************
	GGS_AddService(GATT_ALL_SERVICES);// GAP GATT Service
	GATTServApp_AddService(GATT_ALL_SERVICES);	// GATT Service
	DevInfo_AddService();	// Device Information Service

// Add services to GATT server and give ID of this task for Indication acks.
	LedService_AddService(selfEntity);
	DataService_AddService(selfEntity);

// Register callbacks with the generated services that
// can generate events (writes received) to the application
	LedService_RegisterAppCBs(&ProjectZero_LED_ServiceCBs);
	DataService_RegisterAppCBs(&ProjectZero_Data_ServiceCBs);

// Placeholder variable for characteristic intialization
	uint8_t initVal[20] = { 0 };
	uint8_t initEEG[DS_EEG_LEN] = { 0 };
// Initalization of characteristics in LED_Service that can provide data.
	LedService_SetParameter(LS_LED0_ID, LS_LED0_LEN, initVal);

// Initalization of characteristics in Data_Service that can provide data.
	DataService_SetParameter(DS_EEG1_ID, sizeof(initEEG), initEEG);
	DataService_SetParameter(DS_STREAM_ID, DS_STREAM_LEN, initVal);

// Start Bond Manager and register callback
	VOID GAPBondMgr_Register(&ProjectZero_BondMgrCBs);

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

// Initialize GATT Client, used by GAPBondMgr to look for RPAO characteristic for network privacy
	GATT_InitClient();

// Initialize Connection List
	ProjectZero_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

//Initialize GAP layer for Peripheral role and register to receive GAP events
	GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode,
			&pRandomAddress);

	ADC_init();
	ADC_Params_init(&adcParams);
	adc = ADC_open(R_VBATT, &adcParams);
	if (adc == NULL) {
		while (1) {
		}
	}

	Util_constructClock(&clkBTENotify, ProjectZero_notifyHandler, 1000, 50,
	false, 0x00);
}

/*********************************************************************
 * @fn      ProjectZero_taskFxn
 *
 * @brief   Application task entry point for the Project Zero.
 *
 * @param   a0, a1 - not used.
 */
static void ProjectZero_taskFxn(UArg a0, UArg a1) {
// Initialize application
	ProjectZero_init();

// Application main loop
	for (;;) {
		uint32_t events;

		// Waits for an event to be posted associated with the calling thread.
		// Note that an event associated with a thread is posted when a
		// message is queued to the message receive queue of the thread
		events = Event_pend(syncEvent, Event_Id_NONE, PZ_ALL_EVENTS,
		ICALL_TIMEOUT_FOREVER);

		if (events) {
			ICall_EntityID dest;
			ICall_ServiceEnum src;
			ICall_HciExtEvt *pMsg = NULL;

			// Fetch any available messages that might have been sent from the stack
			if (ICall_fetchServiceMsg(&src, &dest,
					(void**) &pMsg) == ICALL_ERRNO_SUCCESS) {
				uint8_t safeToDealloc = TRUE;

				if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity)) {
					ICall_Stack_Event *pEvt = (ICall_Stack_Event*) pMsg;

					// Check for BLE stack events first
					if (pEvt->signature == 0xffff) {
						// Process stack events
						ProjectZero_processStackEvent(pEvt->event_flag);
					} else {
						switch (pMsg->hdr.event) {
						case GAP_MSG_EVENT:
							// Process GAP message
							ProjectZero_processGapMessage(
									(gapEventHdr_t*) pMsg);
							break;

						case GATT_MSG_EVENT:
							// Process GATT message
							safeToDealloc = ProjectZero_processGATTMsg(
									(gattMsgEvent_t*) pMsg);
							break;

						case HCI_GAP_EVENT_EVENT:
							ProjectZero_processHCIMsg(pMsg);
							break;

							// NOT USED
//						case L2CAP_SIGNAL_EVENT:
//							// Process L2CAP free buffer notification
//							ProjectZero_processL2CAPMsg(
//									(l2capSignalEvent_t*) pMsg);
//							break;

						default:
							// do nothing
							break;
						}
					}
				}

				if (pMsg && safeToDealloc) {
					ICall_freeMsg(pMsg);
				}
			}

			// Process messages sent from another task or another context.
			while (!Queue_empty(appMsgQueueHandle)) {
				pzMsg_t *pMsg = (pzMsg_t*) Util_dequeueMsg(appMsgQueueHandle);
				if (pMsg) {
					// Process application-layer message probably sent from ourselves.
					ProjectZero_processApplicationMessage(pMsg);
					// Free the received message.
					ICall_free(pMsg);
				}
			}
		}
	}
}

/*********************************************************************
 * @fn      ProjectZero_processStackEvent
 *
 * @brief   Process stack event. The event flags received are user-selected
 *          via previous calls to stack APIs.
 *
 * @param   stack_event - mask of events received
 *
 * @return  none
 */
static void ProjectZero_processStackEvent(uint32_t stack_event) {
// Intentionally blank
}

/*********************************************************************
 * @fn      ProjectZero_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg) {
	if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT) {
		// ATT request-response or indication-confirmation flow control is
		// violated. All subsequent ATT requests or indications will be dropped.
		// The app is informed in case it wants to drop the connection.

		// Display the opcode of the message that caused the violation.
		Log_error1("FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
	} else if (pMsg->method == ATT_MTU_UPDATED_EVENT) {
		// MTU size updated
		Log_info1("MTU Size: %d", pMsg->msg.mtuEvt.MTU);
	}

// Free message payload. Needed only for ATT Protocol messages
	GATT_bm_free(&pMsg->msg, pMsg->method);

// It's safe to free the incoming message
	return (TRUE);
}

/*********************************************************************
 * @fn      ProjectZero_processApplicationMessage
 *
 * @brief   Handle application messages
 *
 *          These are messages not from the BLE stack, but from the
 *          application itself.
 *
 *          For example, in a Software Interrupt (Swi) it is not possible to
 *          call any BLE APIs, so instead the Swi function must send a message
 *          to the application Task for processing in Task context.
 *
 * @param   pMsg  Pointer to the message of type pzMsg_t.
 */
static void ProjectZero_processApplicationMessage(pzMsg_t *pMsg) {
// Cast to pzCharacteristicData_t* here since it's a common message pdu type.
	pzCharacteristicData_t *pCharData = (pzCharacteristicData_t*) pMsg->pData;

	switch (pMsg->event) {
	case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
		AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, 0);
		break;

	case PZ_SERVICE_WRITE_EVT: /* Message about received value write */
		/* Call different handler per service */
		switch (pCharData->svcUUID) {
		case LED_SERVICE_SERV_UUID:
			ProjectZero_LedService_ValueChangeHandler(pCharData);
			break;
		case DATA_SERVICE_SERV_UUID:
			ProjectZero_DataService_ValueChangeHandler(pCharData);
			break;
		}
		break;

	case PZ_SERVICE_CFG_EVT: /* Message about received CCCD write */
		/* Call different handler per service */
		switch (pCharData->svcUUID) {
		case DATA_SERVICE_SERV_UUID:
			ProjectZero_DataService_CfgChangeHandler(pCharData);
			break;
		}
		break;

	case PZ_UPDATE_CHARVAL_EVT: /* Message from ourselves to send  */
		ProjectZero_updateCharVal(pCharData);
		break;

	case PZ_ADV_EVT:
		ProjectZero_processAdvEvent((pzGapAdvEventData_t*) (pMsg->pData));
		break;

	case PZ_SEND_PARAM_UPD_EVT: {
		// Send connection parameter update
		pzSendParamReq_t *req = (pzSendParamReq_t*) pMsg->pData;
		ProjectZero_sendParamUpdate(req->connHandle);
	}
		break;

	case PZ_START_ADV_EVT:
		if (linkDB_NumActive() < MAX_NUM_BLE_CONNS) {
			// Enable advertising if there is room for more connections
			GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
		}
		break;

	case PZ_PAIRSTATE_EVT: /* Message about the pairing state */
		ProjectZero_processPairState((pzPairStateData_t*) (pMsg->pData));
		break;

	case PZ_PASSCODE_EVT: /* Message about pairing PIN request */
	{
		pzPasscodeReq_t *pReq = (pzPasscodeReq_t*) pMsg->pData;
		ProjectZero_processPasscode(pReq);
	}
		break;

	case PZ_CONN_EVT:
		ProjectZero_processConnEvt((Gap_ConnEventRpt_t*) (pMsg->pData));
		break;

	case PZ_READ_RPA_EVT: {
		uint8_t *pRpaNew;
		// Need static so string persists until printed in idle thread.
		static uint8_t rpaAddrStr[3 * B_ADDR_LEN + 1];

		// Read the current RPA.
		pRpaNew = GAP_GetDevAddress(FALSE);
		if (pRpaNew != NULL) {
			if (memcmp(pRpaNew, rpaAddr, B_ADDR_LEN) != 0) {
				util_arrtohex(pRpaNew, B_ADDR_LEN, rpaAddrStr,
						sizeof(rpaAddrStr),
						UTIL_ARRTOHEX_REVERSE);
				//print RP address
				Log_info1("RP ADDR: "
						ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET),
						(uintptr_t)rpaAddrStr);

				memcpy(rpaAddr, pRpaNew, B_ADDR_LEN);
			}
		}
		break;
	}
	case NOTIFY_ESLO_EVT: {
		DataService_SetParameter(DS_STREAM_ID, 4, (uint8_t*) (pMsg->pData));
		break;
	}
	case NOTIFY_EEG_EVT: {
		DataService_SetParameter(DS_EEG1_ID, DS_EEG_LEN, (uint8_t*) (pMsg->pData));
		break;
	}
	default:
		break;
	}

	if (pMsg->pData != NULL) {
		ICall_free(pMsg->pData);
	}
}

/*********************************************************************
 * @fn      ProjectZero_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void ProjectZero_processGapMessage(gapEventHdr_t *pMsg) {
	switch (pMsg->opcode) {
	case GAP_DEVICE_INIT_DONE_EVENT: {
		uint8_t *pRpaNew;
		bStatus_t status = FAILURE;
		// Need static so string persists until printed in idle thread.
		static uint8_t rpaAddrStr[3 * B_ADDR_LEN + 1];

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
			// Display device address
			// Need static so string persists until printed in idle thread.
			static uint8_t addrStr[3 * B_ADDR_LEN + 1];
			util_arrtohex(pPkt->devAddr, B_ADDR_LEN, addrStr, sizeof addrStr,
			UTIL_ARRTOHEX_REVERSE);
			Log_info1("GAP is started. Our address: "
					ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET),
					(uintptr_t)addrStr);

			// Setup and start Advertising
			// For more information, see the GAP section in the User's Guide:
			// http://software-dl.ti.com/lprf/ble5stack-latest/

			// Create Advertisement set #1 and assign handle
			status = GapAdv_create(&ProjectZero_advCallback, &advParams1,
					&advHandleLegacy);

			APP_ASSERT(status == SUCCESS);

			Log_info1("Name in advData1 array: "
					ANSI_COLOR(FG_YELLOW) "%s" ANSI_COLOR(ATTR_RESET),
					(uintptr_t)util_getLocalNameStr(advData1,
							sizeof(advData1)));

			// Load advertising data for set #1 that is statically allocated by the app
			status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
					sizeof(advData1), advData1);
			APP_ASSERT(status == SUCCESS);

			// Load scan response data for set #1 that is statically allocated by the app
			status = GapAdv_loadByHandle(advHandleLegacy,
					GAP_ADV_DATA_TYPE_SCAN_RSP, sizeof(scanResData1),
					scanResData1);
			APP_ASSERT(status == SUCCESS);

			// Set event mask for set #1
			status = GapAdv_setEventMask(advHandleLegacy,
					GAP_ADV_EVT_MASK_START_AFTER_ENABLE
							| GAP_ADV_EVT_MASK_END_AFTER_DISABLE
							| GAP_ADV_EVT_MASK_SET_TERMINATED);

			// Enable legacy advertising for set #1
			status = GapAdv_enable(advHandleLegacy,
					GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
			APP_ASSERT(status == SUCCESS);

			if (addrMode > ADDRMODE_RANDOM) {
				// Read the current RPA.
				pRpaNew = GAP_GetDevAddress(FALSE);
				if (pRpaNew != NULL) {
					// Update the current RPA.
					memcpy(rpaAddr, pRpaNew, B_ADDR_LEN);
					//print RP address
					util_arrtohex(pRpaNew, B_ADDR_LEN, rpaAddrStr,
							sizeof(rpaAddrStr),
							UTIL_ARRTOHEX_REVERSE);
					Log_info1("RP ADDR: "
							ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET),
							(uintptr_t)rpaAddrStr);

					// Create one-shot clock for RPA check event.
					Util_constructClock(&clkRpaRead, ProjectZero_clockHandler,
					READ_RPA_PERIOD, 0, true, PZ_READ_RPA_EVT);
				}
			}
		}
		break;
	}

	case GAP_LINK_ESTABLISHED_EVENT: {
		gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t*) pMsg;

		// Display the amount of current connections
		Log_info2("Link establish event, status 0x%02x. Num Conns: %d",
				pPkt->hdr.status,
				linkDB_NumActive());

		if (pPkt->hdr.status == SUCCESS) {
			// Add connection to list
			ProjectZero_addConn(pPkt->connectionHandle);

			// Display the address of this connection
			static uint8_t addrStr[3 * B_ADDR_LEN + 1];
			util_arrtohex(pPkt->devAddr, B_ADDR_LEN, addrStr, sizeof addrStr,
			UTIL_ARRTOHEX_REVERSE);
			Log_info1("Connected. Peer address: "
					ANSI_COLOR(FG_GREEN)"%s"ANSI_COLOR(ATTR_RESET),
					(uintptr_t)addrStr);
		}

		if (linkDB_NumActive() < MAX_NUM_BLE_CONNS) {
			// Start advertising since there is room for more connections
			GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
		}
	}
		break;

	case GAP_LINK_TERMINATED_EVENT: {
		gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t*) pMsg;

		// Display the amount of current connections
		Log_info0("Device Disconnected!");
		Log_info1("Num Conns: %d", linkDB_NumActive());

		// Remove the connection from the list and disable RSSI if needed
		ProjectZero_removeConn(pPkt->connectionHandle);
	}
		break;

	case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
		ProjectZero_handleUpdateLinkParamReq(
				(gapUpdateLinkParamReqEvent_t*) pMsg);
		break;

	case GAP_LINK_PARAM_UPDATE_EVENT:
		ProjectZero_handleUpdateLinkEvent((gapLinkUpdateEvent_t*) pMsg);
		break;

	case GAP_PAIRING_REQ_EVENT:
		// Disable advertising so that the peer device can be added to
		// the resolving list
		GapAdv_disable(advHandleLegacy);
		break;

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
		case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
		{
			linkDBInfo_t linkInfo;
			gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

			// Get the address from the connection handle
			linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

			// Display the address of the connection update failure
			Log_info2("Peer Device's Update Request Rejected 0x%x: %s", pPkt->opcode,
					Util_convertBdAddr2Str(linkInfo.addr));

			break;
		}
#endif

	default:
		break;
	}
}

void ProjectZero_processHCIMsg(ICall_HciExtEvt *pEvt) {
	ICall_Hdr *pMsg = (ICall_Hdr*) pEvt;

// Process HCI message
	switch (pMsg->status) {
	case HCI_COMMAND_COMPLETE_EVENT_CODE:
		// Process HCI Command Complete Events here
		ProjectZero_processCmdCompleteEvt((hciEvt_CmdComplete_t*) pMsg);
		break;

	case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
		AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, 0);
		break;

		// HCI Commands Events
	case HCI_COMMAND_STATUS_EVENT_CODE: {
		hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t*) pMsg;
		switch (pMyMsg->cmdOpcode) {
		case HCI_LE_SET_PHY: {
			if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE) {
				Log_info0("PHY Change failure, peer does not support this");
			} else {
				Log_info1("PHY Update Status Event: 0x%x",
						pMyMsg->cmdStatus);
			}

			ProjectZero_updatePHYStat(HCI_LE_SET_PHY, (uint8_t*) pMsg);
		}
			break;

		default:
			break;
		}
	}
		break;

		// LE Events
	case HCI_LE_EVENT_CODE: {
		hciEvt_BLEPhyUpdateComplete_t *pPUC =
				(hciEvt_BLEPhyUpdateComplete_t*) pMsg;

		// A Phy Update Has Completed or Failed
		if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT) {
			if (pPUC->status != SUCCESS) {
				Log_info0("PHY Change failure");
			} else {
				// Only symmetrical PHY is supported.
				// rxPhy should be equal to txPhy.
				Log_info1("PHY Updated to %s",
						(uintptr_t)((pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1M" :
								(pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2M" :
								(pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "CODED" : "Unexpected PHY Value"));
			}

			ProjectZero_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT,
					(uint8_t*) pMsg);
		}
	}
		break;

	default:
		break;
	}
}

/*********************************************************************
 * @fn      ProjectZero_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static void ProjectZero_processAdvEvent(pzGapAdvEventData_t *pEventData) {
	switch (pEventData->event) {
	/* Sent on the first advertisement after a GapAdv_enable */
	case GAP_EVT_ADV_START_AFTER_ENABLE :
		Log_info1("Adv Set %d Enabled", *(uint8_t *)(pEventData->pBuf));
		break;

		/* Sent after advertising stops due to a GapAdv_disable */
	case GAP_EVT_ADV_END_AFTER_DISABLE :
		Log_info1("Adv Set %d Disabled", *(uint8_t *)(pEventData->pBuf));
		break;

		/* Sent at the beginning of each advertisement. (Note that this event
		 * is not enabled by default, see GapAdv_setEventMask). */
	case GAP_EVT_ADV_START :
		break;

		/* Sent after each advertisement. (Note that this event is not enabled
		 * by default, see GapAdv_setEventMask). */
	case GAP_EVT_ADV_END :
		break;

		/* Sent when an advertisement set is terminated due to a
		 * connection establishment */
	case GAP_EVT_ADV_SET_TERMINATED : {
		GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t*) (pEventData->pBuf);

		Log_info2("Adv Set %d disabled after conn %d",
				advSetTerm->handle, advSetTerm->connHandle);
	}
		break;

		/* Sent when a scan request is received. (Note that this event
		 * is not enabled by default, see GapAdv_setEventMask). */
	case GAP_EVT_SCAN_REQ_RECEIVED :
		break;

		/* Sent when an operation could not complete because of a lack of memory.
		 This message is not allocated on the heap and must not be freed */
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
 * @fn      ProjectZero_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @param   pPairData - pointer to pair state data container
 */
static void ProjectZero_processPairState(pzPairStateData_t *pPairData) {
	uint8_t state = pPairData->state;
	uint8_t status = pPairData->status;

	switch (state) {
	case GAPBOND_PAIRING_STATE_STARTED:
		Log_info0("Pairing started");
		break;

	case GAPBOND_PAIRING_STATE_COMPLETE:
		if (status == SUCCESS) {
			Log_info0("Pairing success");
		} else {
			Log_info1("Pairing fail: %d", status);
		}
		break;

	case GAPBOND_PAIRING_STATE_ENCRYPTED:
		if (status == SUCCESS) {
			Log_info0("Encryption success");
		} else {
			Log_info1("Encryption failed: %d", status);
		}
		break;

	case GAPBOND_PAIRING_STATE_BOND_SAVED:
		if (status == SUCCESS) {
			Log_info0("Bond save success");
		} else {
			Log_info1("Bond save failed: %d", status);
		}
		break;

	default:
		break;
	}
}

/*********************************************************************
 * @fn      ProjectZero_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @param   pReq - pointer to passcode req
 */
static void ProjectZero_processPasscode(pzPasscodeReq_t *pReq) {
	Log_info2("BondMgr Requested passcode. We are %s passcode %06d",
			(uintptr_t)(pReq->uiInputs ? "Sending" : "Displaying"),
			B_APP_DEFAULT_PASSCODE);

// Send passcode response.
	GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
}
/*********************************************************************
 * @fn      ProjectZero_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void ProjectZero_processConnEvt(Gap_ConnEventRpt_t *pReport) {
	/* If we are waiting for an OAD Reboot, process connection events to ensure
	 * that we are not waiting to send data before restarting
	 */

// Process connection events normally
	Log_info1("Connection event done for connHandle: %d", pReport->handle);
}
/*********************************************************************
 * @fn      ProjectZero_clockHandler
 *
 * @brief   clock handler function
 *
 * @param   arg - argument from the clock initiator
 *
 * @return  none
 */
void ProjectZero_clockHandler(UArg arg) {
	uint8_t evtId = (uint8_t) (arg & 0xFF);

	switch (evtId) {
	case PZ_READ_RPA_EVT: {
		// Restart timer
		Util_startClock(&clkRpaRead);
		// Let the application handle the event
		ProjectZero_enqueueMsg(PZ_READ_RPA_EVT, NULL);
		break;
	}

	default:
		break;
	}
}

// !! NOT USED
/*********************************************************************
 * @fn      ProjectZero_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
//static void ProjectZero_connEvtCB(Gap_ConnEventRpt_t *pReport) {
//// Enqueue the event for processing in the app context.
//	if (ProjectZero_enqueueMsg(PZ_CONN_EVT, pReport) != SUCCESS) {
//		ICall_free(pReport);
//	}
//}
/*********************************************************************
 * @fn      ProjectZero_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 */
static void ProjectZero_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg) {
	uint8_t status = pMsg->pReturnParam[0];

//Find which command this command complete is for
	switch (pMsg->cmdOpcode) {
	case HCI_READ_RSSI: {
		int8 rssi = (int8) pMsg->pReturnParam[3];

		// Display RSSI value, if RSSI is higher than threshold, change to faster PHY
		if (status == SUCCESS) {
			uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1],
					pMsg->pReturnParam[2]);

			Log_info2("RSSI:%d, connHandle %d",
					(uint32_t)(rssi),
					(uint32_t)handle);
		} // end of if (status == SUCCESS)
		break;
	}

	case HCI_LE_READ_PHY: {
		if (status == SUCCESS) {
			Log_info2("RXPh: %d, TXPh: %d",
					pMsg->pReturnParam[3], pMsg->pReturnParam[4]);
		}
		break;
	}

	default:
		break;
	} // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
 * @fn      ProjectZero_handleUpdateLinkParamReq
 *
 * @brief   Receive and respond to a parameter update request sent by
 *          a peer device
 *
 * @param   pReq - pointer to stack request message
 */
static void ProjectZero_handleUpdateLinkParamReq(
		gapUpdateLinkParamReqEvent_t *pReq) {
	gapUpdateLinkParamReqReply_t rsp;

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
}

/*********************************************************************
 * @fn      ProjectZero_handleUpdateLinkEvent
 *
 * @brief   Receive and parse a parameter update that has occurred.
 *
 * @param   pEvt - pointer to stack event message
 */
static void ProjectZero_handleUpdateLinkEvent(gapLinkUpdateEvent_t *pEvt) {
// Get the address from the connection handle
	linkDBInfo_t linkInfo;
	linkDB_GetInfo(pEvt->connectionHandle, &linkInfo);

	static uint8_t addrStr[3 * B_ADDR_LEN + 1];
	util_arrtohex(linkInfo.addr, B_ADDR_LEN, addrStr, sizeof addrStr,
	UTIL_ARRTOHEX_REVERSE);

	if (pEvt->status == SUCCESS) {
		uint8_t ConnIntervalFracture = 25 * (pEvt->connInterval % 4);
		// Display the address of the connection update
		Log_info5(
				"Updated params for %s, interval: %d.%d ms, latency: %d, timeout: %d ms",
				(uintptr_t)addrStr,
				(uintptr_t)(pEvt->connInterval*CONN_INTERVAL_MS_CONVERSION),
				ConnIntervalFracture,
				pEvt->connLatency,
				pEvt->connTimeout*CONN_TIMEOUT_MS_CONVERSION);
	} else {
		// Display the address of the connection update failure
		Log_info2("Update Failed 0x%02x: %s", pEvt->opcode, (uintptr_t)addrStr);
	}

// Check if there are any queued parameter updates
	pzConnHandleEntry_t *connHandleEntry = (pzConnHandleEntry_t*) List_get(
			&paramUpdateList);
	if (connHandleEntry != NULL) {
		// Attempt to send queued update now
		ProjectZero_sendParamUpdate(*(connHandleEntry->connHandle));

		// Free list element
		ICall_free(connHandleEntry->connHandle);
		ICall_free(connHandleEntry);
	}
}

/*********************************************************************
 * @fn      ProjectZero_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @param   connHandle - connection handle
 *
 * @return  bleMemAllocError if a param update event could not be sent. Else SUCCESS.
 */
static uint8_t ProjectZero_addConn(uint16_t connHandle) {
	uint8_t i;
	uint8_t status = bleNoResources;

// Try to find an available entry
	for (i = 0; i < MAX_NUM_BLE_CONNS; i++) {
		if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID) {
			// Found available entry to put a new connection info in
			connList[i].connHandle = connHandle;

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
			// Create a clock object and start
			connList[i].pUpdateClock = (Clock_Struct*) ICall_malloc(
					sizeof(Clock_Struct));

			if (connList[i].pUpdateClock) {
				Util_constructClock(connList[i].pUpdateClock,
						ProjectZero_paramUpdClockHandler,
						SEND_PARAM_UPDATE_DELAY, 0, true,
						(uintptr_t) connHandle);
			}
#endif

			// Set default PHY to 1M
			connList[i].currPhy = HCI_PHY_1_MBPS; // TODO: Is this true, neccessarily?

			break;
		}
	}

	return (status);
}

/*********************************************************************
 * @fn      ProjectZero_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @param   connHandle - connection handle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t ProjectZero_getConnIndex(uint16_t connHandle) {
	uint8_t i;

	for (i = 0; i < MAX_NUM_BLE_CONNS; i++) {
		if (connList[i].connHandle == connHandle) {
			return (i);
		}
	}

	return (MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      ProjectZero_clearConnListEntry
 *
 * @brief   Clear the connection information structure held locally.
 *
 * @param   connHandle - connection handle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t ProjectZero_clearConnListEntry(uint16_t connHandle) {
	uint8_t i;
// Set to invalid connection index initially
	uint8_t connIndex = MAX_NUM_BLE_CONNS;

	if (connHandle != LINKDB_CONNHANDLE_ALL) {
		// Get connection index from handle
		connIndex = ProjectZero_getConnIndex(connHandle);
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
		}
	}

	return (SUCCESS);
}

/*********************************************************************
 * @fn      ProjectZero_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @param   connHandle - connection handle
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t ProjectZero_removeConn(uint16_t connHandle) {
	uint8_t connIndex = ProjectZero_getConnIndex(connHandle);

	if (connIndex < MAX_NUM_BLE_CONNS) {
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
		}
		// Clear Connection List Entry
		ProjectZero_clearConnListEntry(connHandle);
	}

	return connIndex;
}

/*********************************************************************
 * @fn      ProjectZero_sendParamUpdate
 *
 * @brief   Remove a device from the connected device list
 *
 * @param   connHandle - connection handle
 */
static void ProjectZero_sendParamUpdate(uint16_t connHandle) {
	gapUpdateLinkParamReq_t req;
	uint8_t connIndex;

	req.connectionHandle = connHandle;
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
	req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
	req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
	req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
	req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
#endif

	connIndex = ProjectZero_getConnIndex(connHandle);
	APP_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

// Deconstruct the clock object
	Clock_destruct(connList[connIndex].pUpdateClock);
// Free clock struct
	ICall_free(connList[connIndex].pUpdateClock);
	connList[connIndex].pUpdateClock = NULL;

// Send parameter update
	bStatus_t status = GAP_UpdateLinkParamReq(&req);

// If there is an ongoing update, queue this for when the update completes
	if (status == bleAlreadyInRequestedMode) {
		pzConnHandleEntry_t *connHandleEntry = ICall_malloc(
				sizeof(pzConnHandleEntry_t));
		if (connHandleEntry) {
			connHandleEntry->connHandle = ICall_malloc(sizeof(uint16_t));

			if (connHandleEntry->connHandle) {
				*(connHandleEntry->connHandle) = connHandle;

				List_put(&paramUpdateList, (List_Elem*) &connHandleEntry);
			}
		}
	}
}

/*********************************************************************
 * @fn      ProjectZero_updatePHYStat
 *
 * @brief   Update the auto phy update state machine
 *
 * @param   eventCode - HCI LE Event code
 *          pMsg - message to process
 */
static void ProjectZero_updatePHYStat(uint16_t eventCode, uint8_t *pMsg) {
	uint8_t connIndex;
	pzConnHandleEntry_t *connHandleEntry;

	switch (eventCode) {
	case HCI_LE_SET_PHY: {
		// Get connection handle from list
		connHandleEntry = (pzConnHandleEntry_t*) List_get(&setPhyCommStatList);

		if (connHandleEntry) {
			// Get index from connection handle
			connIndex = ProjectZero_getConnIndex(
					*(connHandleEntry->connHandle));
			APP_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

			ICall_free(connHandleEntry->connHandle);
			ICall_free(connHandleEntry);

			hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t*) pMsg;

			if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE) {
				// Update the phy change request status for active RSSI tracking connection
				connList[connIndex].phyCngRq = FALSE;
				connList[connIndex].phyRqFailCnt++;
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
			uint8_t index = ProjectZero_getConnIndex(pPUC->connHandle);
			APP_ASSERT(index < MAX_NUM_BLE_CONNS);

			// Update the phychange request status for active RSSI tracking connection
			connList[index].phyCngRq = FALSE;

			if (pPUC->status == SUCCESS) {
				connList[index].currPhy = pPUC->rxPhy;
			}
			if (pPUC->rxPhy != connList[index].rqPhy) {
				connList[index].phyRqFailCnt++;
			} else {
				// Reset the request phy counter and requested phy
				connList[index].phyRqFailCnt = 0;
				connList[index].rqPhy = 0;
			}
		}

		break;
	}

	default:
		break;
	} // end of switch (eventCode)
}

/*
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void ProjectZero_LedService_ValueChangeHandler(
		pzCharacteristicData_t *pCharData) {
	static uint8_t pretty_data_holder[16]; // 5 bytes as hex string "AA:BB:CC:DD:EE"
	util_arrtohex(pCharData->data, pCharData->dataLen, pretty_data_holder,
			sizeof(pretty_data_holder),
			UTIL_ARRTOHEX_NO_REVERSE);

	switch (pCharData->paramID) {
	case LS_LED0_ID:
		Log_info3("Value Change msg: %s %s: %s",
				(uintptr_t)"LED Service",
				(uintptr_t)"LED0",
				(uintptr_t)pretty_data_holder);

		// Do something useful with pCharData->data here
		// -------------------------
		// Set the output value equal to the received value. 0 is off, not 0 is on
		GPIO_write(LED_0, pCharData->data[0]);
		Log_info2("Turning %s %s",
				(uintptr_t)ANSI_COLOR(FG_GREEN)"LED0"ANSI_COLOR(ATTR_RESET),
				(uintptr_t)(pCharData->data[0] ? "on" : "off"));
		break;

	default:
		return;
	}
}

/*
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void ProjectZero_DataService_ValueChangeHandler(
		pzCharacteristicData_t *pCharData) {
// Value to hold the received string for printing via Log, as Log printouts
// happen in the Idle task, and so need to refer to a global/static variable.
	static uint8_t received_eeg[DS_EEG_LEN] = { 0 };

	switch (pCharData->paramID) {
	case DS_EEG1_ID:
		// Do something useful with pCharData->data here
		// -------------------------
		// Copy received data to holder array, ensuring NULL termination.
		memset(received_eeg, 0, DS_EEG_LEN);
		memcpy(received_eeg, pCharData->data,
				MIN(pCharData->dataLen, DS_EEG_LEN - 1));
		// Needed to copy before log statement, as the holder array remains after
		// the pCharData message has been freed and reused for something else.
		Log_info3("Value Change msg: %s %s: %s",
				(uintptr_t)"Data Service",
				(uintptr_t)"EEG",
				(uintptr_t)received_eeg);
		break;

	case DS_STREAM_ID:
		Log_info3("Value Change msg: Data Service Stream: %02x:%02x:%02x...",
				pCharData->data[0],
				pCharData->data[1],
				pCharData->data[2]);
		// -------------------------
		// Do something useful with pCharData->data here
//		DataService_SetParameter(DS_STREAM_ID, pCharData->dataLen,
//				&pCharData->data);
		break;

	default:
		return;
	}
}

/*
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void ProjectZero_DataService_CfgChangeHandler(
		pzCharacteristicData_t *pCharData) {
// Cast received data to uint16, as that's the format for CCCD writes.
	uint16_t configValue = *(uint16_t*) pCharData->data;
	char *configValString;

// Determine what to tell the user
	switch (configValue) {
	case GATT_CFG_NO_OPERATION:
		configValString = "Noti/Ind disabled";
		if (Util_isActive(&clkBTENotify) == true) {
			Util_stopClock(&clkBTENotify);
		}
		break;
	case GATT_CLIENT_CFG_NOTIFY:
		configValString = "Notifications enabled";
		if (Util_isActive(&clkBTENotify) == false) {
			Util_startClock(&clkBTENotify);
		}
		break;
	case GATT_CLIENT_CFG_INDICATE:
		configValString = "Indications enabled";
		break;
	default:
		configValString = "Unsupported operation";
	}

	switch (pCharData->paramID) {
	case DS_STREAM_ID:
		Log_info3("CCCD Change msg: %s %s: %s",
				(uintptr_t)"Data Service",
				(uintptr_t)"Stream",
				(uintptr_t)configValString);
		// -------------------------
		// Do something useful with configValue here. It tells you whether someone
		// wants to know the state of this characteristic.
		// ...
		break;

	case DS_EEG1_ID:
		Log_info3("CCCD Change msg: %s %s: %s",
				(uintptr_t)"Data Service",
				(uintptr_t)"EEG1",
				(uintptr_t)configValString);
		break;
	}
}

/*
 * @brief  Convenience function for updating characteristic data via pzCharacteristicData_t
 *         structured message.
 *
 * @note   Must run in Task context in case BLE Stack APIs are invoked.
 *
 * @param  *pCharData  Pointer to struct with value to update.
 */
static void ProjectZero_updateCharVal(pzCharacteristicData_t *pCharData) {
	switch (pCharData->svcUUID) {
	case LED_SERVICE_SERV_UUID:
		LedService_SetParameter(pCharData->paramID, pCharData->dataLen,
				pCharData->data);
		break;
	}
}

/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of direct system callbacks.
 *
 *  Typically enqueue the information or request as a message for the
 *  application Task for handling.
 *
 ****************************************************************************
 *****************************************************************************/

/*
 *  Callbacks from the Stack Task context (GAP or Service changes)
 *****************************************************************************/

/*********************************************************************
 * @fn      ProjectZero_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 *          pBuf - data potentially accompanying event
 *          arg - not used
 */
static void ProjectZero_advCallback(uint32_t event, void *pBuf, uintptr_t arg) {
	pzGapAdvEventData_t *eventData = ICall_malloc(sizeof(pzGapAdvEventData_t));

	if (eventData != NULL) {
		eventData->event = event;
		eventData->pBuf = pBuf;

		if (ProjectZero_enqueueMsg(PZ_ADV_EVT, eventData) != SUCCESS) {
			ICall_free(eventData);
		}
	}
}

/*********************************************************************
 * @fn      ProjectZero_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle
 *          state - pair state
 *          status - pair status
 */
static void ProjectZero_pairStateCb(uint16_t connHandle, uint8_t state,
		uint8_t status) {
	pzPairStateData_t *pairState = (pzPairStateData_t*) ICall_malloc(
			sizeof(pzPairStateData_t));

	if (pairState != NULL) {
		pairState->state = state;
		pairState->connHandle = connHandle;
		pairState->status = status;

		if (ProjectZero_enqueueMsg(PZ_PAIRSTATE_EVT, pairState) != SUCCESS) {
			ICall_free(pairState);
		}
	}
}

/*********************************************************************
 * @fn      ProjectZero_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @param   pDeviceAddr - not used
 *          connHandle - connection handle
 *          uiInpuits - if TRUE, the local device should accept a passcode input
 *          uiOutputs - if TRUE, the local device should display the passcode
 *          numComparison - the code that should be displayed for numeric
 *          comparison pairing. If this is zero, then passcode pairing is occurring.
 */
static void ProjectZero_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
		uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison) {
	pzPasscodeReq_t *req = (pzPasscodeReq_t*) ICall_malloc(
			sizeof(pzPasscodeReq_t));
	if (req != NULL) {
		req->connHandle = connHandle;
		req->uiInputs = uiInputs;
		req->uiOutputs = uiOutputs;
		req->numComparison = numComparison;

		if (ProjectZero_enqueueMsg(PZ_PASSCODE_EVT, req) != SUCCESS) {
			ICall_free(req);
		}
	};
}

/*********************************************************************
 * @fn      ProjectZero_LedService_ValueChangeCB
 *
 * @brief   Callback for characteristic change when a peer writes to us
 *
 * @param   connHandle - connection handle
 *          paramID - the parameter ID maps to the characteristic written to
 *          len - length of the data written
 *          pValue - pointer to the data written
 */
static void ProjectZero_LedService_ValueChangeCB(uint16_t connHandle,
		uint8_t paramID, uint16_t len, uint8_t *pValue) {
// See the service header file to compare paramID with characteristic.
	Log_info1("(CB) LED Svc Characteristic value change: paramID(%d). "
			"Sending msg to app.", paramID);

	pzCharacteristicData_t *pValChange = ICall_malloc(
			sizeof(pzCharacteristicData_t) + len);

	if (pValChange != NULL) {
		pValChange->svcUUID = LED_SERVICE_SERV_UUID;
		pValChange->paramID = paramID;
		memcpy(pValChange->data, pValue, len);
		pValChange->dataLen = len;

		if (ProjectZero_enqueueMsg(PZ_SERVICE_WRITE_EVT, pValChange) != SUCCESS) {
			ICall_free(pValChange);
		}
	}
}

/*********************************************************************
 * @fn      ProjectZero_DataService_ValueChangeCB
 *
 * @brief   Callback for characteristic change when a peer writes to us
 *
 * @param   connHandle - connection handle
 *          paramID - the parameter ID maps to the characteristic written to
 *          len - length of the data written
 *          pValue - pointer to the data written
 */
static void ProjectZero_DataService_ValueChangeCB(uint16_t connHandle,
		uint8_t paramID, uint16_t len, uint8_t *pValue) {
// See the service header file to compare paramID with characteristic.
	Log_info1("(CB) Data Svc Characteristic value change: paramID(%d). "
			"Sending msg to app.", paramID);

	pzCharacteristicData_t *pValChange = ICall_malloc(
			sizeof(pzCharacteristicData_t) + len);

	if (pValChange != NULL) {
		pValChange->svcUUID = DATA_SERVICE_SERV_UUID;
		pValChange->paramID = paramID;
		memcpy(pValChange->data, pValue, len);
		pValChange->dataLen = len;

		if (ProjectZero_enqueueMsg(PZ_SERVICE_WRITE_EVT, pValChange) != SUCCESS) {
			ICall_free(pValChange);
		}
	}
}

/*********************************************************************
 * @fn      ProjectZero_DataService_CfgChangeCB
 *
 * @brief   Callback for when a peer enables or disables the CCCD attribute,
 *          indicating they are interested in notifications or indications.
 *
 * @param   connHandle - connection handle
 *          paramID - the parameter ID maps to the characteristic written to
 *          len - length of the data written
 *          pValue - pointer to the data written
 */
static void ProjectZero_DataService_CfgChangeCB(uint16_t connHandle,
		uint8_t paramID, uint16_t len, uint8_t *pValue) {
	Log_info1("(CB) Data Svc Char config change paramID(%d). "
			"Sending msg to app.", paramID);

	pzCharacteristicData_t *pValChange = ICall_malloc(
			sizeof(pzCharacteristicData_t) + len);

	if (pValChange != NULL) {
		pValChange->svcUUID = DATA_SERVICE_SERV_UUID;
		pValChange->paramID = paramID;
		memcpy(pValChange->data, pValue, len);
		pValChange->dataLen = len;

		if (ProjectZero_enqueueMsg(PZ_SERVICE_CFG_EVT, pValChange) != SUCCESS) {
			ICall_free(pValChange);
		}
	}
}

/*
 *  Callbacks from Swi-context
 *****************************************************************************/

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
/*********************************************************************
 * @fn      ProjectZero_paramUpdClockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - app message pointer
 */
static void ProjectZero_paramUpdClockHandler(UArg arg) {
	pzSendParamReq_t *req = (pzSendParamReq_t*) ICall_malloc(
			sizeof(pzSendParamReq_t));
	if (req) {
		req->connHandle = (uint16_t) arg;
		if (ProjectZero_enqueueMsg(PZ_SEND_PARAM_UPD_EVT, req) != SUCCESS) {
			ICall_free(req);
		}
	}
}
#endif

/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/

/*********************************************************************
 * @fn     ProjectZero_enqueueMsg
 *
 * @brief  Utility function that sends the event and data to the application.
 *         Handled in the task loop.
 *
 * @param  event    Event type
 * @param  pData    Pointer to message data
 */
static status_t ProjectZero_enqueueMsg(uint8_t event, void *pData) {
	uint8_t success;
	pzMsg_t *pMsg = ICall_malloc(sizeof(pzMsg_t));

	if (pMsg) {
		pMsg->event = event;
		pMsg->pData = pData;

		success = Util_enqueueMsg(appMsgQueueHandle, syncEvent,
				(uint8_t*) pMsg);
		return (success) ? SUCCESS : FAILURE;
	}

	return (bleMemAllocError);
}

/*********************************************************************
 * @fn     util_arrtohex
 *
 * @brief   Convert {0x01, 0x02} to "01:02"
 *
 * @param   src - source byte-array
 * @param   src_len - length of array
 * @param   dst - destination string-array
 * @param   dst_len - length of array
 *
 * @return  array as string
 */
char* util_arrtohex(uint8_t const *src, uint8_t src_len, uint8_t *dst,
		uint8_t dst_len, uint8_t reverse) {
	char hex[] = "0123456789ABCDEF";
	uint8_t *pStr = dst;
	uint8_t avail = dst_len - 1;
	int8_t inc = 1;
	if (reverse) {
		src = src + src_len - 1;
		inc = -1;
	}

	memset(dst, 0, avail);

	while (src_len && avail > 3) {
		if (avail < dst_len - 1) {
			*pStr++ = ':';
			avail -= 1;
		}

		*pStr++ = hex[*src >> 4];
		*pStr++ = hex[*src & 0x0F];
		src += inc;
		avail -= 2;
		src_len--;
	}

	if (src_len && avail) {
		*pStr++ = ':'; // Indicate not all data fit on line.
	}
	return ((char*) dst);
}

/*********************************************************************
 * @fn     util_getLocalNameStr
 *
 * @brief   Extract the LOCALNAME from Scan/AdvData
 *
 * @param   data - Pointer to the advertisement or scan response data
 * @param   len  - Length of advertisment or scan repsonse data
 *
 * @return  Pointer to null-terminated string with the adv local name.
 */
static char* util_getLocalNameStr(const uint8_t *data, uint8_t len) {
	uint8_t nuggetLen = 0;
	uint8_t nuggetType = 0;
	uint8_t advIdx = 0;

	static char localNameStr[32] = { 0 };
	memset(localNameStr, 0, sizeof(localNameStr));

	for (advIdx = 0; advIdx < len;) {
		nuggetLen = data[advIdx++];
		nuggetType = data[advIdx];
		if ((nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE
				|| nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT)) {
			uint8_t len_temp =
					nuggetLen < (sizeof(localNameStr) - 1) ?
							(nuggetLen - 1) : (sizeof(localNameStr) - 2);
			// Only copy the first 31 characters, if name bigger than 31.
			memcpy(localNameStr, &data[advIdx + 1], len_temp);
			break;
		} else {
			advIdx += nuggetLen;
		}
	}

	return (localNameStr);
}
