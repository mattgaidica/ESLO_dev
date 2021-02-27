// init

whoamI = 0;
lsm303agr_mag_device_id_get(&dev_ctx_mg, &whoamI);
if (whoamI != LSM303AGR_ID_MG)
while (1)
; /*manage here device not found */

/* Restore default configuration for magnetometer */
lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);
do {
	lsm303agr_mag_reset_get(&dev_ctx_mg, &rst);
}while (rst);

lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_10Hz);
lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg,
		LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
lsm303agr_mag_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
lsm303agr_temperature_meas_set(&dev_ctx_xl, LSM303AGR_TEMP_ENABLE);
lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_CONTINUOUS_MODE); // LSM303AGR_POWER_DOWN
lsm303agr_mag_drdy_on_pin_set(&dev_ctx_mg, PROPERTY_ENABLE);

mgInterrupt(true);// turn on interrupt

static void xlDataHandler(void) {
	uint8_t iFifo;
	lsm303agr_fifo_src_reg_a_t fifo_reg;
	lsm303agr_xl_fifo_status_get(&dev_ctx_xl, &fifo_reg);

	if (USE_AXY(esloSettings) == ESLO_MODULE_ON) { // double check
		eegInterrupt(false); // !! use I2C callback instead? single-shot LSM303AGR on CC2652 timer?
		for (iFifo = 0; iFifo <= fifo_reg.fss; iFifo++) { // 32 samples
			memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
			lsm303agr_acceleration_raw_get(&dev_ctx_xl,
					data_raw_acceleration.u8bit);

			eslo.type = Type_AxyXlx;
			eslo.data = (uint32_t) data_raw_acceleration.i16bit[0];
			ESLO_Packet(eslo, &packet);
			xlXBuffer[iFifo] = packet;
			if (!isPaired) {
				ret = ESLO_Write(&esloAddr, esloBuffer, eslo);
			}

			eslo.type = Type_AxyXly;
			eslo.data = (uint32_t) data_raw_acceleration.i16bit[1];
			ESLO_Packet(eslo, &packet);
			xlYBuffer[iFifo] = packet;
			if (!isPaired) {
				ret = ESLO_Write(&esloAddr, esloBuffer, eslo);
			}

			eslo.type = Type_AxyXlz;
			eslo.data = (uint32_t) data_raw_acceleration.i16bit[2];
			ESLO_Packet(eslo, &packet);
			xlZBuffer[iFifo] = packet;
			if (!isPaired) {
				ret = ESLO_Write(&esloAddr, esloBuffer, eslo);
			}

			axyCount++;
			// !!handle ret
		}

		if (isPaired) {
			SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
					SIMPLEPROFILE_CHAR5_LEN, xlXBuffer);
			SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
					SIMPLEPROFILE_CHAR5_LEN, xlYBuffer);
			SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
					SIMPLEPROFILE_CHAR5_LEN, xlZBuffer);
		}

		lsm303agr_xl_fifo_mode_set(&dev_ctx_xl, LSM303AGR_BYPASS_MODE);
		lsm303agr_xl_fifo_mode_set(&dev_ctx_xl, LSM303AGR_FIFO_MODE);
	}
}

static uint8_t updateXlFromSettings(bool actOnInterrupt) {
	bool enableInterrupt;

	if (USE_AXY(esloSettings) == ESLO_MODULE_ON) {
		switch (esloSettings[Set_AxyMode]) {
		case 1:
			lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_1Hz);
			break;
		case 2:
			lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_10Hz);
			break;
		default:
			break;
		}
		lsm303agr_xl_fifo_mode_set(&dev_ctx_xl, LSM303AGR_BYPASS_MODE); // clear int
		lsm303agr_xl_fifo_mode_set(&dev_ctx_xl, LSM303AGR_FIFO_MODE); // enable
		enableInterrupt = true;
		if (actOnInterrupt) {
			xlInterrupt(enableInterrupt);
		}
	} else {
		enableInterrupt = false;
		xlInterrupt(enableInterrupt); // always turn off before powering down
		lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_POWER_DOWN);
	}
	return enableInterrupt;
}

static void mgInterrupt(bool enableInterrupt) {
	if (enableInterrupt) {
		GPIO_enableInt (AXY_MAG);
	} else {
		GPIO_disableInt (AXY_MAG);
	}
}
