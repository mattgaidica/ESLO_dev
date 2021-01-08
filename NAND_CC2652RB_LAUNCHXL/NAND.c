#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* Driver Header files */
#include <SPI_NAND.h>
#include <ESLO.h>
#include <Serialize.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

/* Driver configuration */
#include <ti_drivers_config.h>

// NAND user defined
uint8_t ret;
uint16_t devId;
uAddrType esloAddr = 0x00000000;
uint8_t readBuf[PAGE_SIZE]; // 2176, always allocate full page size
uint32_t i;

// !! erase memory.dat before running
void* mainThread(void *arg0) {
	uint8_t saveBuf[1][PAGE_DATA_SIZE];
	GPIO_init();
	SPI_init();

	GPIO_write(_SHDN, GPIO_CFG_OUT_LOW); // ADS129X off
	GPIO_write(LED_0, CONFIG_GPIO_LED_OFF);

	NAND_Init(CONFIG_SPI, _NAND_CS, _FRAM_CS);
	ret = FlashReadDeviceIdentification(&devId); // 0x2C25

	uint32_t iBlock = 0;
	uint8_t iPage;
	uint32_t esloVersion;
	uint32_t esloCurVersion;
	uint8_t doLoop = 1;

	// could find last block first, then for loop
	while (doLoop == 1) {
		for (iPage = 0; iPage < 16; iPage++) {
			ret = FlashPageRead(esloAddr, readBuf); // read whole page
			memcpy(saveBuf[iPage], readBuf, PAGE_DATA_SIZE); // transfer to array
			esloAddr += 0x00001000; // +1 page
		}
		// write entire block
		GPIO_toggle(LED_0);

		if (iBlock == 0) {
			memcpy(&esloVersion, saveBuf[0], 4); // first page
		} else {
			memcpy(&esloCurVersion, saveBuf[0], 4);
			if (esloVersion != esloCurVersion) {
				doLoop = 0;
			}
		}
		iBlock++;
	}

	GPIO_write(LED_0, CONFIG_GPIO_LED_OFF);
	return (0);
}
