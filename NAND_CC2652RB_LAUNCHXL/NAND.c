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

	NAND_Init(CONFIG_SPI, _NAND_CS, _FRAM_CS);
	ret = FlashReadDeviceIdentification(&devId); // 0x2C25

	// will run 131,072 times
	//	while (esloAddr < FLASH_SIZE) {
	GPIO_write(LED_0, CONFIG_GPIO_LED_ON);
	for (i = 0; i < 50; i++) {
		ret = FlashPageRead(esloAddr, readBuf);
		esloAddr += 0x00001000;
		GPIO_toggle(LED_0);
	}

	GPIO_write(LED_0, CONFIG_GPIO_LED_OFF);
	return (0);
}
