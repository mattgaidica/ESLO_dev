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
	GPIO_init();
	SPI_init();

	GPIO_write(_SHDN, GPIO_CFG_OUT_LOW); // ADS129X off
	GPIO_write(LED_0, CONFIG_GPIO_LED_OFF);

	NAND_Init(CONFIG_SPI, _NAND_CS);
	ret = FlashReadDeviceIdentification(&devId); // 0x2C25

	uint32_t iBlock = 0;
	uint8_t iPage;
	uint32_t esloVersion;
	uint32_t esloCurVersion;
	uint8_t doLoop = 1;

	// could find last block first, then for loop
	while (doLoop == 1) {
		iPage = ADDRESS_2_PAGE(esloAddr);
		iBlock = ADDRESS_2_BLOCK(esloAddr);
		ret = FlashPageRead(esloAddr, readBuf); // read whole page

		if (iPage == 0) {
			if (iBlock == 0) {
				memcpy(&esloVersion, readBuf, 4); // first instance
			} else {
				memcpy(&esloCurVersion, readBuf, 4); // subsequent instances
				if (readBuf[3] == 0xFF) {
					doLoop = 0;
				} else {
					if (readBuf[3] == 0x0F && esloVersion != esloCurVersion) {
						doLoop = 0;
					}
				}
			}
		}
		GPIO_toggle(LED_0); // GEL breakpoint
		esloAddr += 0x00001000; // +1 page
	}

	GPIO_write(LED_0, CONFIG_GPIO_LED_OFF);
	return (0);
}
