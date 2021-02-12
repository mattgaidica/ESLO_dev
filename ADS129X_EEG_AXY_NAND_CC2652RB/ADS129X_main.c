#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

/* Driver Header files */
#include <ti/drivers/ADC.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>

#include <xdc/std.h>
#include <xdc/runtime/Types.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Log.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* POSIX Header files */
#include <pthread.h>

/* AXY */
#include <lsm303agr_reg.h>
#include <lsm303agr_CCXXXX.h>

/* NAND */
#include <SPI_NAND.h>
#include <ESLO.h>
#include <Serialize.h>

/* ADS129X */
#include <ADS129X.h>
#include <Definitions.h>

/* ----- END INCLUDES ----- */

/* Module Settings */
ESLO_ModuleStatus USE_EEG = ESLO_MODULE_OFF;
ESLO_ModuleStatus USE_XL = ESLO_MODULE_OFF;
ESLO_ModuleStatus USE_MG = ESLO_MODULE_OFF;
ESLO_ModuleStatus USE_TIMER = ESLO_MODULE_OFF;

/* AXY Vars */
stmdev_ctx_t dev_ctx_xl;
stmdev_ctx_t dev_ctx_mg;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_magnetic;
static uint8_t whoamI, rst;
uint32_t XL_TIMEOUT = 500000;

/* NAND Vars */
uint8_t ret;
uint16_t devId;
uint8_t esloBuffer[PAGE_DATA_SIZE];
//uint8_t readBuf[PAGE_SIZE]; // 2176, always allocate full page size
uint32_t esloPacket;
uAddrType esloAddr;

/* ADS129X Vars */
int32_t status;
int32_t ch1;
int32_t ch2;
int32_t ch3;
int32_t ch4;

/* ----- Application ----- */
uint32_t secondCount = 0;
uint32_t esloVersion = 0x00000000;
uint32_t axyCount;
uint32_t eegCount;
uint32_t magCount;
eslo_dt eslo = { .mode = Mode_Debug };
ReturnType ret; // NAND

/* Tasks and Semaphores */
void eegTaskFcn(UArg a0, UArg a1);
void xlTaskFcn(UArg a0, UArg a1);
void mgTaskFcn(UArg a0, UArg a1);
void adcTaskFcn(UArg a0, UArg a1);

void* mainThread(void *arg0);
void timerCallback(Timer_Handle myHandle, int_fast16_t status);

/* Stack size in bytes */
#define THREADSTACKSIZE    1024

Task_Handle eegTask;
Task_Params eegTaskParams;
Semaphore_Handle eegSem;
Semaphore_Params eegSemParams;

Task_Handle xlTask;
Task_Params xlTaskParams;
Semaphore_Handle xlSem;
Semaphore_Params xlSemParams;

Task_Handle mgTask;
Task_Params mgTaskParams;
Semaphore_Handle mgSem;
Semaphore_Params mgSemParams;

Task_Handle adcTask;
Task_Params adcTaskParams;
Semaphore_Handle adcSem;
Semaphore_Params adcSemParams;

Timer_Handle timer0;
Timer_Params timerParams;

ADC_Handle adc;
ADC_Params adcParams;
int_fast16_t adcRes;
uint16_t adcValue0;
uint32_t adcValue0MicroVolt;

void loopCallback();

void loopCallback() {
	if (GPIO_read(DEBUG) == 0) {
		GPIO_write(LED_0, ESLO_LOW);
	} else {
		GPIO_toggle(LED_0);
	}
	Semaphore_post(adcSem);
	secondCount++;
}


void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
	if (GPIO_read(DEBUG) == 0) {
		GPIO_write(LED_0, ESLO_LOW);
	} else {
		GPIO_toggle(LED_0);
	}
	Semaphore_post(adcSem);
	secondCount++;
}

void adcTaskFcn(UArg a0, UArg a1) {
	while (1) {
		Semaphore_pend(adcSem, BIOS_WAIT_FOREVER);
		/* Blocking mode conversion */
		adcRes = ADC_convert(adc, &adcValue0);
		if (adcRes == ADC_STATUS_SUCCESS) {
			// read, multiply by 2 for voltage divider
			adcValue0MicroVolt = ADC_convertRawToMicroVolts(adc, adcValue0) * 2;
			eslo.type = Type_BatteryVoltage;
			eslo.data = adcValue0MicroVolt;
			ret = ESLO_Write(&esloAddr, esloBuffer, eslo);
		}
		eslo.type = Type_AbsoluteTime;
		eslo.data = secondCount;
		ret = ESLO_Write(&esloAddr, esloBuffer, eslo);
	}
}

void eegTaskFcn(UArg a0, UArg a1) {
	while (1) {
		Semaphore_pend(eegSem, BIOS_WAIT_FOREVER); // Wait for semaphore from HWI callback function
		ADS_updateData(&status, &ch1, &ch2, &ch3, &ch4);
		if (GPIO_read(DEBUG)) {
			eslo.type = Type_EEG1;
			eslo.data = ch1;
			ret = ESLO_Write(&esloAddr, esloBuffer, eslo);

			eslo.type = Type_EEG2;
			eslo.data = ch2;
			ret = ESLO_Write(&esloAddr, esloBuffer, eslo);

			eslo.type = Type_EEG3;
			eslo.data = eegCount; //ch3;
			ret = ESLO_Write(&esloAddr, esloBuffer, eslo);

			eslo.type = Type_EEG4;
			eslo.data = ch4;
			ret = ESLO_Write(&esloAddr, esloBuffer, eslo);

			eegCount++;
		}
	}
}

void xlTaskFcn(UArg a0, UArg a1) {
	uint8_t iFifo;
	while (1) {
		// timeout 5s for debugging (force interrupt reset)
		Semaphore_pend(xlSem, BIOS_WAIT_FOREVER); //XL_TIMEOUT);

		lsm303agr_fifo_src_reg_a_t fifo_reg;
		lsm303agr_xl_fifo_status_get(&dev_ctx_xl, &fifo_reg);

		for (iFifo = 0; iFifo <= fifo_reg.fss; iFifo++) {
			memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
			lsm303agr_acceleration_raw_get(&dev_ctx_xl,
					data_raw_acceleration.u8bit);

			if (GPIO_read(DEBUG)) {
				eslo.type = Type_AxyXlx;
				eslo.data = (uint32_t) data_raw_acceleration.i16bit[0];
				ret = ESLO_Write(&esloAddr, esloBuffer, eslo);

				eslo.type = Type_AxyXly;
				eslo.data = (uint32_t) data_raw_acceleration.i16bit[1];
				ret = ESLO_Write(&esloAddr, esloBuffer, eslo);

				eslo.type = Type_AxyXlz;
				eslo.data = (uint32_t) data_raw_acceleration.i16bit[2];
				ret = ESLO_Write(&esloAddr, esloBuffer, eslo);

				axyCount++;
			}
			// !!handle ret

//			acceleration_mg[0] = lsm303agr_from_fs_2g_hr_to_mg(
//					data_raw_acceleration.i16bit[0]);
//			acceleration_mg[1] = lsm303agr_from_fs_2g_hr_to_mg(
//					data_raw_acceleration.i16bit[1]);
//			acceleration_mg[2] = lsm303agr_from_fs_2g_hr_to_mg(
//					data_raw_acceleration.i16bit[2]
		}

		lsm303agr_xl_fifo_mode_set(&dev_ctx_xl, LSM303AGR_BYPASS_MODE);
		lsm303agr_xl_fifo_mode_set(&dev_ctx_xl, LSM303AGR_FIFO_MODE);
	}
}

void mgTaskFcn(UArg a0, UArg a1) {
	while (1) {
		Semaphore_pend(mgSem, BIOS_WAIT_FOREVER);

//		lsm303agr_reg_t reg;
//		lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);

		/* Read magnetic field data */
		memset(data_raw_magnetic.u8bit, 0x00, 3 * sizeof(int16_t));
		lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw_magnetic.u8bit);

		magCount++;
	}
}

void eegDataReady(uint_least8_t index) {
	Semaphore_post(eegSem);
}

void axyXlReady(uint_least8_t index) {
	Semaphore_post(xlSem);
}

void axyMagReady(uint_least8_t index) {
	Semaphore_post(mgSem);
}

void ESLO_startup(void) {
	/* NAND */
	NAND_Init(CONFIG_SPI, _NAND_CS);
	ret = FlashReadDeviceIdentification(&devId);

	ESLO_SetVersion(&esloVersion, CONFIG_TRNG_0);
	eslo.type = Type_Version;
	eslo.version = esloVersion; // set version once
	eslo.data = esloVersion; // data is version in this case
	ret = ESLO_Write(&esloAddr, esloBuffer, eslo);

	/* ADS129X - Defaults in SysConfig */
	if (USE_EEG == ESLO_MODULE_ON) {
		GPIO_write(_SHDN, ESLO_HIGH);
		GPIO_setConfig(_EEG_CS,
		GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_LOW | GPIO_CFG_OUT_LOW);
		GPIO_setConfig(EEG_START,
		GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_LOW | GPIO_CFG_OUT_HIGH);
		Task_sleep(150000 / Clock_tickPeriod);
		// check for ADS ID
		ADS_init(CONFIG_SPI_EEG, _EEG_CS);
		uint8_t adsId = ADS_getDeviceID(); // 0x90
	} else {
		GPIO_write(_SHDN, ESLO_LOW); // redundant to Sysconfig
	}

	/* AXY */
	AXY_Init(CONFIG_I2C_AXY);
	dev_ctx_xl.write_reg = platform_i2c_write;
	dev_ctx_xl.read_reg = platform_i2c_read;
	dev_ctx_xl.handle = (void*) LSM303AGR_I2C_ADD_XL;
	dev_ctx_mg.write_reg = platform_i2c_write;
	dev_ctx_mg.read_reg = platform_i2c_read;
	dev_ctx_mg.handle = (void*) LSM303AGR_I2C_ADD_MG;

	lsm303agr_temperature_meas_set(&dev_ctx_xl, LSM303AGR_TEMP_DISABLE);

	if (USE_XL == ESLO_MODULE_ON) {
		whoamI = 0;
		lsm303agr_xl_device_id_get(&dev_ctx_xl, &whoamI);
		while (whoamI != LSM303AGR_ID_XL) {
			while (1) {
			}
		}
		lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
		lsm303agr_xl_full_scale_set(&dev_ctx_xl, LSM303AGR_2g);
		lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_HR_12bit);
		lsm303agr_xl_fifo_set(&dev_ctx_xl, PROPERTY_ENABLE);
		lsm303agr_ctrl_reg3_a_t int1Val;
		int1Val.i1_overrun = 1;
		int1Val.i1_wtm = 0;
		int1Val.i1_drdy2 = 0;
		int1Val.i1_drdy1 = 0;
		int1Val.i1_aoi2 = 0;
		int1Val.i1_aoi1 = 0;
		int1Val.i1_click = 0;
		lsm303agr_xl_pin_int1_config_set(&dev_ctx_xl, &int1Val);
	}

	if (USE_MG == ESLO_MODULE_ON) {
		whoamI = 0;
		lsm303agr_mag_device_id_get(&dev_ctx_mg, &whoamI);

		if (whoamI != LSM303AGR_ID_MG) {
			while (1) {
			}
		}

		/* Restore default configuration for magnetometer */
		lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);
		do {
			lsm303agr_mag_reset_get(&dev_ctx_mg, &rst);
		} while (rst);

		lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
		lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_10Hz);
		lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg,
				LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
		lsm303agr_mag_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
		lsm303agr_mag_drdy_on_pin_set(&dev_ctx_mg, PROPERTY_ENABLE);
	}

	if (USE_TIMER == ESLO_MODULE_ON) {
		Timer_Params_init(&timerParams);
		timerParams.period = 1;
		timerParams.periodUnits = Timer_PERIOD_HZ;
		timerParams.timerMode = Timer_CONTINUOUS_CALLBACK;
		timerParams.timerCallback = timerCallback;
		timer0 = Timer_open(CONFIG_TIMER_0, &timerParams);
		if (timer0 == NULL) { // failed init
			while (1) {
			}
		}
		if (Timer_start(timer0) == Timer_STATUS_ERROR) {
			while (1) { // failed start
			}
		}
	}

	ADC_Params_init(&adcParams);
	adc = ADC_open(R_VBATT, &adcParams);
	if (adc == NULL) {
		while (1){
		}
	}

	// ENABLE INTERRUPTS
	if (USE_EEG == ESLO_MODULE_ON) {
		GPIO_enableInt(_EEG_DRDY);
	}

	if (USE_XL == ESLO_MODULE_ON) {
		lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_1Hz);
		lsm303agr_xl_fifo_mode_set(&dev_ctx_xl, LSM303AGR_BYPASS_MODE); // clear int
		GPIO_enableInt(AXY_INT1);
		lsm303agr_xl_fifo_mode_set(&dev_ctx_xl, LSM303AGR_FIFO_MODE); // enable
	} else {
		lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_POWER_DOWN);
	}

	if (USE_MG == ESLO_MODULE_ON) {
		GPIO_enableInt(AXY_MAG);
		lsm303agr_mag_operating_mode_set(&dev_ctx_mg,
				LSM303AGR_CONTINUOUS_MODE);
	} else {
		lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_POWER_DOWN);
	}
}

/*
 *  ======== mainThread ========
 */
void* mainThread(void *arg0) {
	GPIO_init();

	// !!cant use XL timeout with this
	uint8_t loopCount = 0;
	while (loopCount < 2) {
		if (GPIO_read(DEBUG) == 0) {
			loopCount = 0;
		} else {
			loopCount++;
		}
		GPIO_write(LED_0, ELSO_HIGH);
		Task_sleep(10000);
		GPIO_write(LED_0, ESLO_LOW);
		Task_sleep(200000); // 2s
	}
	GPIO_write(LED_0, ESLO_LOW);

	SPI_init();
	I2C_init();
	Timer_init();
	ADC_init();

	ESLO_startup();

	while (1) {
		Task_sleep(300000);
		loopCallback();
	}
}

/*
 *  ======== main ========
 */
int main(void) {
	pthread_t thread;
	pthread_attr_t attrs;
	struct sched_param priParam;
	int retc;

	Board_init();

	/* Initialize the attributes structure with default values */
	pthread_attr_init(&attrs);

	/* Set priority, detach state, and stack size attributes */
	priParam.sched_priority = 1;
	retc = pthread_attr_setschedparam(&attrs, &priParam);
	retc |= pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
	retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
	if (retc != 0) {
		/* failed to set attributes */
		while (1) {
		}
	}

	retc = pthread_create(&thread, &attrs, mainThread, NULL);
	if (retc != 0) {
		/* pthread_create() failed */
		while (1) {
		}
	}

	// Create Task(s)
	Task_Params_init(&eegTaskParams); // Init Task Params with pri=2, stackSize = 512
	eegTaskParams.priority = 2;
	eegTaskParams.stackSize = 1024;
	eegTask = Task_create(eegTaskFcn, &eegTaskParams, Error_IGNORE); // Create task1 with task1Fxn (Error Block ignored, we explicitly test 'task1' handle)
	if (eegTask == NULL) {                  // Verify that Task1 was created
		System_abort("Task eeg create failed");      // If not abort program
	}

	Task_Params_init(&xlTaskParams); // Init Task Params with pri=2, stackSize = 512
	xlTaskParams.priority = 2;
	xlTaskParams.stackSize = 1024;
	xlTask = Task_create(xlTaskFcn, &xlTaskParams, Error_IGNORE); // Create task1 with task1Fxn (Error Block ignored, we explicitly test 'task1' handle)
	if (xlTask == NULL) {                   // Verify that Task1 was created
		System_abort("Task xl create failed");       // If not abort program
	}

	Task_Params_init(&mgTaskParams); // Init Task Params with pri=2, stackSize = 512
	mgTaskParams.priority = 2;
	mgTaskParams.stackSize = 1024;
	mgTask = Task_create(mgTaskFcn, &mgTaskParams, Error_IGNORE); // Create task1 with task1Fxn (Error Block ignored, we explicitly test 'task1' handle)
	if (mgTask == NULL) {                   // Verify that Task1 was created
		System_abort("Task mg create failed");       // If not abort program
	}

	Task_Params_init(&adcTaskParams); // Init Task Params with pri=2, stackSize = 512
	adcTaskParams.priority = 2;
	adcTaskParams.stackSize = 1024;
	adcTask = Task_create(adcTaskFcn, &adcTaskParams, Error_IGNORE); // Create task1 with task1Fxn (Error Block ignored, we explicitly test 'task1' handle)
	if (adcTask == NULL) {                   // Verify that Task1 was created
		System_abort("Task adc create failed");       // If not abort program
	}

	// Create Semaphores
	Semaphore_Params_init(&eegSemParams);
	eegSemParams.mode = Semaphore_Mode_BINARY;
	eegSem = Semaphore_create(0, &eegSemParams, Error_IGNORE); // Create Sem, count=0, params default, Error_Block ignored
	if (eegSem == NULL) {                    // Verify that item was created
		System_abort("Semaphore eeg create failed"); // If not abort program
	}

	Semaphore_Params_init(&xlSemParams);
	xlSemParams.mode = Semaphore_Mode_BINARY;
	xlSem = Semaphore_create(0, &xlSemParams, Error_IGNORE); // Create Sem, count=0, params default, Error_Block ignored
	if (xlSem == NULL) {                     // Verify that item was created
		System_abort("Semaphore xl create failed");  // If not abort program
	}

	Semaphore_Params_init(&mgSemParams);
	mgSemParams.mode = Semaphore_Mode_BINARY;
	mgSem = Semaphore_create(0, &mgSemParams, Error_IGNORE); // Create Sem, count=0, params default, Error_Block ignored
	if (mgSem == NULL) {                     // Verify that item was created
		System_abort("Semaphore mg create failed");  // If not abort program
	}

	Semaphore_Params_init(&adcSemParams);
	adcSemParams.mode = Semaphore_Mode_BINARY;
	adcSem = Semaphore_create(0, &adcSemParams, Error_IGNORE); // Create Sem, count=0, params default, Error_Block ignored
	if (adcSem == NULL) {                     // Verify that item was created
		System_abort("Semaphore adc create failed");  // If not abort program
	}

	BIOS_start();

	return (0);
}
