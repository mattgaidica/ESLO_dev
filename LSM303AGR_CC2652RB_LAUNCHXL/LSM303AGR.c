/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

// XDC
#include <xdc/runtime/System.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// user defined
//#include <lsm303agr_defs.h>
#include <lsm303agr_reg.h>
#include <lsm303agr_CCXXXX.h>

#define TX_BUF_DIM          100

// AXY
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_magnetic;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float magnetic_mG[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[TX_BUF_DIM];

//static void tx_com(UART_Handle uart, uint8_t *tx_buffer, uint16_t len) {
//	UART_write(uart, tx_buffer, len);
//}

/*
 *  ======== mainThread ========
 */
void* mainThread(void *arg0) {
//	UART_Handle uart;
//	UART_Params uartParams;

	/* Call driver init functions */
	GPIO_init();
	I2C_init();
//	UART_init();

	AXY_Init(CONFIG_I2C_AXY);

	GPIO_write(LED_0, CONFIG_GPIO_LED_ON); // init
	GPIO_write(_SHDN, GPIO_CFG_OUT_LOW); // ADS129X off

//	UART_Params_init(&uartParams);
//	uartParams.writeDataMode = UART_DATA_BINARY;
//	uartParams.readDataMode = UART_DATA_BINARY;
//	uartParams.readReturnMode = UART_RETURN_FULL;
//	uartParams.baudRate = 115200;
//
//	uart = UART_open(CONFIG_UART_0, &uartParams);

//	if (uart == NULL) {
//		/* UART_open() failed */
//		while (1)
//			;
//	}

	/* Initialize mems driver interface */
	stmdev_ctx_t dev_ctx_xl;
	dev_ctx_xl.write_reg = platform_i2c_write;
	dev_ctx_xl.read_reg = platform_i2c_read;
	dev_ctx_xl.handle = (void*) LSM303AGR_I2C_ADD_XL;
	stmdev_ctx_t dev_ctx_mg;
	dev_ctx_mg.write_reg = platform_i2c_write;
	dev_ctx_mg.read_reg = platform_i2c_read;
	dev_ctx_mg.handle = (void*) LSM303AGR_I2C_ADD_MG;

	whoamI = 0;
	lsm303agr_xl_device_id_get(&dev_ctx_xl, &whoamI);

	if (whoamI != LSM303AGR_ID_XL)
		while (1)
			; /*manage here device not found */

	whoamI = 0;
	lsm303agr_mag_device_id_get(&dev_ctx_mg, &whoamI);

	if (whoamI != LSM303AGR_ID_MG)
		while (1)
			; /*manage here device not found */

	/* Restore default configuration for magnetometer */
	lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);

	do {
		lsm303agr_mag_reset_get(&dev_ctx_mg, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
	lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_1Hz);
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
	lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_HR_12bit);
	/* Set magnetometer in continuous mode */
	lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_CONTINUOUS_MODE);

	while (1) {
		/* Read output only if new value is available */
		lsm303agr_reg_t reg;
		lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);

		if (reg.status_reg_a.zyxda) {
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
			System_sprintf((char*) tx_buffer,
					"Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
					acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
//			tx_com(uart, tx_buffer, strlen((char const*) tx_buffer));
		}

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
			System_sprintf((char*) tx_buffer,
					"Magnetic field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
					magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
//			tx_com(uart, tx_buffer, strlen((char const*) tx_buffer));
		}

		lsm303agr_temp_data_ready_get(&dev_ctx_xl, &reg.byte);

		if (reg.byte) {
			/* Read temperature data */
			memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
			lsm303agr_temperature_raw_get(&dev_ctx_xl,
					data_raw_temperature.u8bit);
			temperature_degC = lsm303agr_from_lsb_hr_to_celsius(
					data_raw_temperature.i16bit);
			System_sprintf((char*) tx_buffer, "Temperature [degC]:%6.2f\r\n",
					temperature_degC);
//			tx_com(uart, tx_buffer, strlen((char const*) tx_buffer));
		}
		GPIO_toggle(LED_0);
	}
}
