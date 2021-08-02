/*
 *  ======== nvsexternal.c ========
 *  open iTerm, find device >> ls /dev/tty.*
 *  >> cd ~/Downloads
 *  clear contents of screenlog.0
 *  debug program to mainThread()
 *  >> screen -L /dev/tty.usbmodemL1100LNK1 115200
 *  1: /dev/tty.usbmodemL1100NA51
 *  2: /dev/tty.usbmodemL1100NM51
 *  3: /dev/tty.usbmodemL1100MPN1
 *  4: /dev/tty.usbmodemL1100LNK1
 *  play program until end
 *  close iTerm with ctrl+a,k
 */

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/* Driver Header files */
#include <ti/display/Display.h>
#include <ti/drivers/NVS.h>

/* Driver configuration */
#include "ti_drivers_config.h"

//static uint8_t ESLO_PREFIX[2] = { 0xEE, 0xEE };
#define BASE_ADDR_LOC		0xFF000
#define B_ADDR_LEN			6
#define SC_ADDR_STR_SIZE	15
uint8_t payload[16] = { 0x00 };
uint32_t nvsOffset, curOffset, curTime;
int8_t RSSI;

char* Util_convertBdAddr2Str(uint8_t *pAddr);

//memcpy(payload, &pAdvRpt->addr, 6 * sizeof(uint8_t));
//memcpy(payload + 6, &curTime, sizeof(uint32_t));
//memcpy(payload + 10, &pAdvRpt->rssi, sizeof(int8_t));
//if (nvsOffset > regionAttrs.sectorSize) {
//	nvsOffset = 0; // wrap data or first time running
//}
//// handle erase
//if (nvsOffset % regionAttrs.sectorSize == 0) {
//	NVS_erase(nvsHandle, nvsOffset, regionAttrs.sectorSize);
//}
//// write payload
//NVS_write(nvsHandle, nvsOffset, (void*) payload, sizeof(payload),
//		NVS_WRITE_POST_VERIFY);
//// update location for power lapse
//NVS_write(nvsHandle, BASE_ADDR_LOC, (void*) &nvsOffset, sizeof(nvsOffset),
//		NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
//nvsOffset += sizeof(payload);

/*
 *  ======== mainThread ========
 */
void* mainThread(void *arg0) {
	NVS_Handle nvsHandle;
	NVS_Attrs regionAttrs;
	NVS_Params nvsParams;

	Display_Handle displayHandle;

	Display_init();
	NVS_init();

	displayHandle = Display_open(Display_Type_UART, NULL);
	if (displayHandle == NULL) {
		/* Display_open() failed */
		while (1)
			;
	}

	NVS_Params_init(&nvsParams);
	nvsHandle = NVS_open(CONFIG_NVSEXTERNAL, &nvsParams);

	if (nvsHandle == NULL) {
		Display_printf(displayHandle, 0, 0, "NVS_open() failed.");
		return (NULL);
	}

	Display_printf(displayHandle, 0, 0, "\n");

	/*
	 * This will populate a NVS_Attrs structure with properties specific
	 * to a NVS_Handle such as region base address, region size,
	 * and sector size.
	 */
	NVS_getAttrs(nvsHandle, &regionAttrs);
	NVS_read(nvsHandle, BASE_ADDR_LOC, (void*) &nvsOffset, sizeof(nvsOffset));

	for (curOffset = 0; curOffset < nvsOffset; curOffset += 16) {
		NVS_read(nvsHandle, curOffset, (void*) payload, sizeof(payload));
		memcpy(&curTime, payload + 6, sizeof(curTime));
		memcpy(&RSSI, payload + 10, sizeof(RSSI));
//		if (payload[5] != 0xEE) {
//			break;
//		}
		Display_printf(displayHandle, 0, 0, "%s,%u,%i",
				Util_convertBdAddr2Str(payload), curTime, RSSI);
	}

	/* The signature was not found in the NVS region. */
	Display_printf(displayHandle, 0, 0, "\n");

	NVS_close(nvsHandle);

	return (NULL);
}

/*********************************************************************
 * @fn      Util_convertBdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @param   pAddr - BD address
 *
 * @return  BD address as a string
 */
char* Util_convertBdAddr2Str(uint8_t *pAddr) {
	uint8_t charCnt;
	char hex[] = "0123456789ABCDEF";
	static char str[(2 * B_ADDR_LEN) + 3];
	char *pStr = str;

	*pStr++ = '0';
	*pStr++ = 'x';

	// Start from end of addr
	pAddr += B_ADDR_LEN;

	for (charCnt = B_ADDR_LEN; charCnt > 0; charCnt--) {
		*pStr++ = hex[*--pAddr >> 4];
		*pStr++ = hex[*pAddr & 0x0F];
	}
	*pStr = 0;

	return str;
}
