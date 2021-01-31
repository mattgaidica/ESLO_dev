/*
 *  ======== emptyLibs.c ========
 */

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

/* ----- Application ----- */
uint32_t secondCount = 0;
uint32_t esloVersion = 0x00000000;
uint32_t axyCount;
uint32_t eegCount;
uint32_t magCount;
eslo_dt eslo = { .mode = Mode_Debug };
ReturnType ret; // NAND

void timerCallback(Timer_Handle myHandle, int_fast16_t status);

void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
	if (GPIO_read(DEBUG) == 0) {
		GPIO_write(LED_0, GPIO_CFG_OUT_HIGH);
	} else {
		GPIO_toggle(LED_0);
	}
	secondCount++;
}

Timer_Handle timer0;
Timer_Params timerParams;

ADC_Handle adc;
ADC_Params adcParams;
int_fast16_t adcRes;

/* Stack size in bytes */
#define THREADSTACKSIZE    1024

void eegDataReady(uint_least8_t index) {
}

void axyXlReady(uint_least8_t index) {
}

void axyMagReady(uint_least8_t index) {
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
		GPIO_write(_SHDN, GPIO_CFG_OUT_HIGH);
		GPIO_setConfig(_EEG_CS,
		GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_LOW | GPIO_CFG_OUT_LOW);
		GPIO_setConfig(EEG_START,
		GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_LOW | GPIO_CFG_OUT_HIGH);
		Task_sleep(150000 / Clock_tickPeriod);
		// check for ADS ID
		ADS_init(CONFIG_SPI_EEG, _EEG_CS);
		uint8 adsId = ADS_getDeviceID(); // 0x90
	} else {
		GPIO_write(_SHDN, GPIO_CFG_OUT_LOW); // redundant to Sysconfig
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
		while (1) {
		}
	}

	// ENABLE INTERRUPTS
	if (USE_EEG == ESLO_MODULE_ON) {
		GPIO_enableInt(_EEG_DRDY);
	}

	if (USE_XL == ESLO_MODULE_ON) {
		lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_10Hz);
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
	/* 1 second delay */
//    uint32_t time = 1;
	/* Call driver init functions */
	GPIO_init();
	// Watchdog_init();

	GPIO_write(LED_0, CONFIG_GPIO_LED_ON);

	SPI_init();
	I2C_init();
	Timer_init();
	ADC_init();

	ESLO_startup();

	while (1) {
		Task_sleep(300000);
		GPIO_write(LED_0, CONFIG_GPIO_LED_OFF);
	}
}
