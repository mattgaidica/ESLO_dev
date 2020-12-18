/*********************************************************************
 * INCLUDES
 */
#include <unistd.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include "util.h"
#include <icall.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>
#include <devinfoservice.h>

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include <ti_drivers_config.h>
#include "ti_ble_config.h"
#include "simple_broadcaster.h"

#include <ADS129X.h>
#include <Definitions.h> // ADS129X

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define EEG_N_SAMPLE 50
#define EEG_FLAG_BYTE 5
#define EEG_DATA_BYTE 7

// Task configuration
#define SB_TASK_PRIORITY                     1

#ifndef SB_TASK_STACK_SIZE
#define SB_TASK_STACK_SIZE                   1024
#endif

#define SB_ADV_EVT                           1

// Internal Events for RTOS application
#define SB_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SB_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define SB_ALL_EVENTS                        (SB_ICALL_EVT | \
                                               SB_QUEUE_EVT)

// Spin if the expression is not true
#define SIMPLEBROADCASTER_ASSERT(expr) if (!(expr)) HAL_ASSERT_SPINLOCK;
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct {
	uint8_t event;                // event type
	void *pData;               // pointer to message
} sbEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
void ESLO_startup(void);

/*********************************************************************
 * LOCAL VARIABLES
 */
// ADS129X
int32_t status;
int32_t ch1;
int32_t ch2;
int32_t ch3;
int32_t ch4;
int8_t iEEG = 0;
int8_t eegAdvCount = 0;

// Advertisement data
uint8_t advData1[EEG_DATA_BYTE + (EEG_N_SAMPLE * 3)] = { 0x02,
GAP_ADTYPE_FLAGS,
GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED | GAP_ADTYPE_FLAGS_GENERAL, (EEG_N_SAMPLE
		* 3) + 3,
GAP_ADTYPE_MANUFACTURER_SPECIFIC, 0x00, 0xff };

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Task configuration
Task_Struct sbTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(sbTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t sbTaskStack[SB_TASK_STACK_SIZE];

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct {
	uint32_t event;
	void *pBuf;
} sbGapAdvEventData_t;

// Advertising handles
static uint8 advHandleLegacy;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBroadcaster_init(void);
static void SimpleBroadcaster_taskFxn(UArg a0, UArg a1);
static void SimpleBroadcaster_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBroadcaster_processGapMessage(gapEventHdr_t *pMsg);
static void SimpleBroadcaster_processAppMsg(sbEvt_t *pMsg);
static void SimpleBroadcaster_advCallback(uint32_t event, void *pBuf,
		uintptr_t arg);
static bool SimpleBroadcaster_processAdvEvent(sbGapAdvEventData_t *pEventData);
static status_t SimpleBroadcaster_enqueueMsg(uint8_t event, void *pData);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
/*
 static gapRolesCBs_t simpleBroadcaster_BroadcasterCBs =
 {
 SimpleBroadcaster_stateChangeCB   // Profile State Change Callbacks
 };
 */
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBroadcaster_createTask
 *
 * @brief   Task creation function for the Simple Broadcaster.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBroadcaster_createTask(void) {
	Task_Params taskParams;

	// Configure task
	Task_Params_init(&taskParams);
	taskParams.stack = sbTaskStack;
	taskParams.stackSize = SB_TASK_STACK_SIZE;
	taskParams.priority = SB_TASK_PRIORITY;

	Task_construct(&sbTask, SimpleBroadcaster_taskFxn, &taskParams, NULL);
}

void eegDataReady(uint_least8_t index) {
	int i;
	ADS_updateData(&status, &ch1, &ch2, &ch3, &ch4);
	// offset from header bytes
	advData1[(iEEG * 3) + EEG_DATA_BYTE] = ch1 >> 16;
	advData1[(iEEG * 3) + EEG_DATA_BYTE + 1] = ch1 >> 8;
	advData1[(iEEG * 3) + EEG_DATA_BYTE + 2] = ch1;
	advData1[EEG_FLAG_BYTE + 1] = iEEG;
	iEEG++;
	if (iEEG == EEG_N_SAMPLE) {
		advData1[EEG_FLAG_BYTE] = eegAdvCount;
		GPIO_toggle(LED_0);
		for (i = 0; i < 200; i++) {
		}
		GPIO_toggle(LED_0);
		if (eegAdvCount == 0xFF) {
			eegAdvCount = 0x00;
		} else {
			eegAdvCount++;
		}
		iEEG = 0;
	}
}

void ESLO_startup(void) {
	GPIO_init();
	SPI_init();

	GPIO_write(LED_0, CONFIG_GPIO_LED_ON); // init
	GPIO_write(_SHDN, GPIO_CFG_OUT_HIGH); // turn on ADS129X
	Task_sleep(150000 / Clock_tickPeriod); // ?? 150ms
	ADS_init(CONFIG_SPI_EEG, _EEG_CS);

//	GPIO_setConfig(_EEG_DRDY, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
//	GPIO_setCallback(_EEG_DRDY, eegDataReady);
	GPIO_enableInt(_EEG_DRDY);
}

/*********************************************************************
 * @fn      SimpleBroadcaster_init
 *
 * @brief   Initialization function for the Simple Broadcaster App
 *          Task. This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBroadcaster_init(void) {
	// ******************************************************************
	// N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
	// ******************************************************************
	// Register the current thread as an ICall dispatcher application
	// so that the application can send and receive messages.
	ICall_registerApp(&selfEntity, &syncEvent);

	uint8 bdAddress[B_ADDR_LEN] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 };
	HCI_EXT_SetBDADDRCmd(bdAddress);

#ifdef USE_RCOSC
	// Set device's Sleep Clock Accuracy
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
	HCI_EXT_SetSCACmd(500);
#endif // (CENTRAL_CFG | PERIPHERAL_CFG)
	RCOSC_enableCalibration();
#endif // USE_RCOSC

	// Create an RTOS queue for message from profile to be sent to app.
	appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

	// Register with GAP for HCI/Host messages. This is needed to receive HCI
	// events. For more information, see the HCI section in the User's Guide:
	// http://software-dl.ti.com/lprf/ble5stack-latest/
	GAP_RegisterForMsgs(selfEntity);

	//Initialize GAP layer for Peripheral role and register to receive GAP events
	GAP_DeviceInit(GAP_PROFILE_BROADCASTER, selfEntity, addrMode,
			&pRandomAddress);
}

/*********************************************************************
 * @fn      SimpleBroadcaster_processEvent
 *
 * @brief   Application task entry point for the Simple Broadcaster.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBroadcaster_taskFxn(UArg a0, UArg a1) {
	// Initialize application
	SimpleBroadcaster_init();
	ESLO_startup();

	// Application main loop
	for (;;) {
		// Get the ticks since startup
		uint32_t tickStart = Clock_getTicks();

		uint32_t events;

		events = Event_pend(syncEvent, Event_Id_NONE, SB_ALL_EVENTS,
		ICALL_TIMEOUT_FOREVER);

		if (events) {
			ICall_EntityID dest;
			ICall_ServiceEnum src;
			ICall_HciExtEvt *pMsg = NULL;

			if (ICall_fetchServiceMsg(&src, &dest,
					(void**) &pMsg) == ICALL_ERRNO_SUCCESS) {
				if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity)) {
					// Process inter-task message
					SimpleBroadcaster_processStackMsg((ICall_Hdr*) pMsg);
				}

				if (pMsg) {
					ICall_freeMsg(pMsg);
				}
			}

			// If RTOS queue is not empty, process app message.
			if (events & SB_QUEUE_EVT) {
				while (!Queue_empty(appMsgQueueHandle)) {
					sbEvt_t *pMsg = (sbEvt_t*) Util_dequeueMsg(
							appMsgQueueHandle);
					if (pMsg) {
						// Process message.
						SimpleBroadcaster_processAppMsg(pMsg);

						// Free the space from the message.
						ICall_free(pMsg);
					}
				}
			}
		}
	}
}

/*********************************************************************
 * @fn      SimpleBroadcaster_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBroadcaster_processStackMsg(ICall_Hdr *pMsg) {
	switch (pMsg->event) {
	case GAP_MSG_EVENT:
		SimpleBroadcaster_processGapMessage((gapEventHdr_t*) pMsg);
		break;

	default:
		// do nothing
		break;
	}
}

/*********************************************************************
 * @fn      SimpleBroadcaster_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void SimpleBroadcaster_processGapMessage(gapEventHdr_t *pMsg) {
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

			// Create Advertisement set
			status = GapAdv_create(&SimpleBroadcaster_advCallback, &advParams1,
					&advHandleLegacy);
			SIMPLEBROADCASTER_ASSERT(status == SUCCESS);

			// Load advertising data
			status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
					sizeof(advData1), advData1);
			SIMPLEBROADCASTER_ASSERT(status == SUCCESS);

			// Set event mask
			status = GapAdv_setEventMask(advHandleLegacy,
					GAP_ADV_EVT_MASK_START_AFTER_ENABLE
							| GAP_ADV_EVT_MASK_END_AFTER_DISABLE
							| GAP_ADV_EVT_MASK_END
							| GAP_ADV_EVT_MASK_SET_TERMINATED);
			SIMPLEBROADCASTER_ASSERT(status == SUCCESS);

			// Enable legacy advertising
			status = GapAdv_enable(advHandleLegacy,
					GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
			SIMPLEBROADCASTER_ASSERT(status == SUCCESS);
		}

		break;
	}

	default:
		break;
	}
}

/*********************************************************************
 * @fn      SimpleBroadcaster_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static status_t SimpleBroadcaster_enqueueMsg(uint8_t event, void *pData) {
	uint8_t success;
	sbEvt_t *pMsg = ICall_malloc(sizeof(sbEvt_t));

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
 * @fn      SimpleBroadcaster_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void SimpleBroadcaster_advCallback(uint32_t event, void *pBuf,
		uintptr_t arg) {
	sbGapAdvEventData_t *pData = ICall_malloc(sizeof(sbGapAdvEventData_t));

	if (pData) {
		pData->event = event;
		pData->pBuf = pBuf;

		if (SimpleBroadcaster_enqueueMsg(SB_ADV_EVT, pData) != SUCCESS) {
			ICall_free(pData);
		}
	}
}

/*********************************************************************
 * @fn      SimpleBroadcaster_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBroadcaster_processAppMsg(sbEvt_t *pMsg) {
	bool safeToDealloc = TRUE;

	switch (pMsg->event) {
	case SB_ADV_EVT:
		safeToDealloc = SimpleBroadcaster_processAdvEvent(
				(sbGapAdvEventData_t*) (pMsg->pData));
		break;

	default:
		// Do nothing.
		break;
	}

	if (safeToDealloc) {
		// Free message data
		if (pMsg->pData) {
			ICall_free(pMsg->pData);
		}
	}
}

/*********************************************************************
 * @fn      SimpleBroadcaster_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static bool SimpleBroadcaster_processAdvEvent(sbGapAdvEventData_t *pEventData) {
	bool safeToDealloc = TRUE;
	bStatus_t status = FAILURE;
	uint32_t tickStart = Clock_getTicks();

	switch (pEventData->event) {
	case GAP_EVT_ADV_START_AFTER_ENABLE :
		// advertising
		break;

	case GAP_EVT_ADV_END :
		// Disable advertising and prepare the buffer.
		status = GapAdv_prepareLoadByBuffer(advData1, FALSE);
		SIMPLEBROADCASTER_ASSERT(status == SUCCESS)
		;

//		advData1[14] = (uint8_t)tickStart;
//		advData1[15] = (uint8_t)tickStart << 8;

		status = GapAdv_loadByBuffer(sizeof(advData1), advData1);
		SIMPLEBROADCASTER_ASSERT(status == SUCCESS)
		;

		break;

	case GAP_EVT_INSUFFICIENT_MEMORY :
		safeToDealloc = FALSE;
		break;

	default:
		// Do nothing.
		break;
	}
	ICall_free(pEventData->pBuf); // matt
	return (safeToDealloc);
}

/*********************************************************************
 *********************************************************************/
