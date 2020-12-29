#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* AXY */
#include <lsm303agr_reg.h>
#include <lsm303agr_CCXXXX.h>

static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_magnetic;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float magnetic_mG[3];
static float temperature_degC;
static uint8_t whoamI, rst;

/* ADS129X */
#include <ADS129X.h>
#include <Definitions.h>

int32_t status;
int32_t ch1;
int32_t ch2;
int32_t ch3;
int32_t ch4;

int32_t ch1Buf[256];
int32_t ch2Buf[256];
int32_t ch3Buf[256];
int32_t ch4Buf[256];

/* Initialize mems driver interface */
stmdev_ctx_t dev_ctx_xl;
stmdev_ctx_t dev_ctx_mg;

void axyMagReady(uint_least8_t index) {
//	GPIO_toggle(LED_0);
	lsm303agr_reg_t reg;
	lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);

	if (reg.status_reg_m.zyxda) {
		/* Read magnetic field data */
		memset(data_raw_magnetic.u8bit, 0x00, 3 * sizeof(int16_t));
		lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw_magnetic.u8bit);
		magnetic_mG[0] = lsm303agr_from_lsb_to_mgauss(
				data_raw_magnetic.i16bit[0]);
		magnetic_mG[1] = lsm303agr_from_lsb_to_mgauss(
				data_raw_magnetic.i16bit[1]);
		magnetic_mG[2] = lsm303agr_from_lsb_to_mgauss(
				data_raw_magnetic.i16bit[2]);
	}

}

void axyXlReady(uint_least8_t index) {
	GPIO_disableInt(AXY_INT1);
//	memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
//	lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw_acceleration.u8bit);
//	acceleration_mg[0] = lsm303agr_from_fs_2g_hr_to_mg(
//			data_raw_acceleration.i16bit[0]);
//	acceleration_mg[1] = lsm303agr_from_fs_2g_hr_to_mg(
//			data_raw_acceleration.i16bit[1]);
//	acceleration_mg[2] = lsm303agr_from_fs_2g_hr_to_mg(
//			data_raw_acceleration.i16bit[2]);
	lsm303agr_xl_fifo_mode_set(&dev_ctx_xl, LSM303AGR_BYPASS_MODE);
	lsm303agr_xl_fifo_mode_set(&dev_ctx_xl, LSM303AGR_FIFO_MODE);
	GPIO_toggle(LED_0);
	GPIO_enableInt(AXY_INT1);
}

void eegDataReady(uint_least8_t index) {
	ADS_updateData(&status, &ch1, &ch2, &ch3, &ch4);
}

void ESLO_startup(void) {
	GPIO_init();
	SPI_init();
	I2C_init();
	GPIO_write(LED_0, CONFIG_GPIO_LED_ON);

	/* ADS129X */
	GPIO_write(_SHDN, GPIO_CFG_OUT_HIGH);
	Task_sleep(150000 / Clock_tickPeriod);
	ADS_init(CONFIG_SPI_EEG, _EEG_CS);

	/* AXY */
	AXY_Init(CONFIG_I2C_AXY);

	dev_ctx_xl.write_reg = platform_i2c_write;
	dev_ctx_xl.read_reg = platform_i2c_read;
	dev_ctx_xl.handle = (void*) LSM303AGR_I2C_ADD_XL;
	dev_ctx_mg.write_reg = platform_i2c_write;
	dev_ctx_mg.read_reg = platform_i2c_read;
	dev_ctx_mg.handle = (void*) LSM303AGR_I2C_ADD_MG;

	whoamI = 0;
	lsm303agr_xl_device_id_get(&dev_ctx_xl, &whoamI);

	if (whoamI != LSM303AGR_ID_XL) {
		while (1) {
			GPIO_toggle(LED_0);
			Task_sleep(200000);
			whoamI = 0;
			lsm303agr_xl_device_id_get(&dev_ctx_xl, &whoamI);
		}
	}

	whoamI = 0;
	lsm303agr_mag_device_id_get(&dev_ctx_mg, &whoamI);

	if (whoamI != LSM303AGR_ID_MG) {
		while (1) {
			GPIO_toggle(LED_0);
			Task_sleep(200000);
		}
	}

	/* Restore default configuration for magnetometer */
	lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);

	do {
		lsm303agr_mag_reset_get(&dev_ctx_mg, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
	lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_10Hz);
	lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_10Hz);
	/* Set accelerometer full scale */
	lsm303agr_xl_full_scale_set(&dev_ctx_xl, LSM303AGR_2g);
	/* Set / Reset magnetic sensor mode */
	lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg,
			LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
	/* Enable temperature compensation on mag sensor */
	lsm303agr_mag_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
	/* Enable temperature sensor */
	lsm303agr_temperature_meas_set(&dev_ctx_xl, LSM303AGR_TEMP_ENABLE);

	/* Set device in continuous mode */
	lsm303agr_xl_fifo_set(&dev_ctx_xl, PROPERTY_ENABLE);
	lsm303agr_xl_fifo_mode_set(&dev_ctx_xl, LSM303AGR_FIFO_MODE);

	lsm303agr_ctrl_reg3_a_t int1Val;
	int1Val.i1_overrun = 1;
	lsm303agr_xl_pin_int1_config_set(&dev_ctx_xl, &int1Val);
	lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_HR_12bit);

	/* Set magnetometer in continuous mode */
	lsm303agr_mag_drdy_on_pin_set(&dev_ctx_mg, PROPERTY_ENABLE);
	lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_CONTINUOUS_MODE);

//	GPIO_enableInt(_EEG_DRDY);
//	GPIO_enableInt(AXY_INT1);
//	GPIO_enableInt(_AXY_MAG);
}

/*
 *  ======== mainThread ========
 */
void* mainThread(void *arg0) {
	ESLO_startup();

	while (1) {
		GPIO_write(LED_0, GPIO_read(AXY_INT1));

//		lsm303agr_reg_t reg;
//		lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);

//		if (reg.status_reg_a.zyxda) {
		if (GPIO_read(AXY_INT1)) {
			/* Read accelerometer data */
			memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
			lsm303agr_acceleration_raw_get(&dev_ctx_xl,
					data_raw_acceleration.u8bit);
			acceleration_mg[0] = lsm303agr_from_fs_2g_hr_to_mg(
					data_raw_acceleration.i16bit[0]);
			acceleration_mg[1] = lsm303agr_from_fs_2g_hr_to_mg(
					data_raw_acceleration.i16bit[1]);
			acceleration_mg[2] = lsm303agr_from_fs_2g_hr_to_mg(
					data_raw_acceleration.i16bit[2]);
		}
	}
}
