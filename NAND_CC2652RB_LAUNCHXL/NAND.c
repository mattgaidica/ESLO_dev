/* For sleep() */
#include <unistd.h>
//#include <stdint.h>
//#include <stddef.h>
#include <stdlib.h> // rand()
#include <string.h>
#include <math.h> // atan2(x,y)

// CMSIS Math
#include "arm_math.h"
#include "arm_const_structs.h"
#define FFT_SAMPLES 1024*2
#define FFT_SAMPLES_HALF (FFT_SAMPLES / 2)
#define BLOCK_SIZE 32;

#include "FFTsignal.h"
static float32_t complexFFT[FFT_SAMPLES];
static float32_t realFFT[FFT_SAMPLES_HALF];
static float32_t imagFFT[FFT_SAMPLES_HALF];
static float32_t angleFFT[FFT_SAMPLES_HALF];
static float32_t powerFFT[FFT_SAMPLES_HALF];
static float32_t filtSignal[FFT_SAMPLES];

// FILTER START
#define IIR_ORDER 4
#define IIR_NUMSTAGES (IIR_ORDER/2)
static float32_t m_biquad_state[IIR_ORDER];
static float32_t m_biquad_coeffs[5 * IIR_NUMSTAGES] = { 1.0000000000f,
		-1.9169186492f, 1.0000000000f, 1.9314191302f, -0.9338812219f,
		1.0000000000f, -1.9829505952f, 1.0000000000f, 1.9737574359f,
		-0.9797267211f };
arm_biquad_cascade_df2T_instance_f32 const filtInst = {
IIR_ORDER / 2, m_biquad_state, m_biquad_coeffs };

// FILTER END

uint32_t fftSize = FFT_SAMPLES;
uint32_t ifftFlag = 0;
arm_rfft_fast_instance_f32 S;
uint32_t maxIndex = 0;

/* Driver Header files */
#include <SPI_NAND.h>
#include <ESLO.h>
#include <Serialize.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

#include <ti/sysbios/knl/Clock.h>

/* Driver configuration */
#include <ti_drivers_config.h>

// NAND user defined
uint8_t ret;
uint16_t devId;

uint32_t packet = 0xFF123456;
eslo_dt eslo = { .mode = Mode_Debug, .type = Type_EEG1 };

uAddrType iPage = 0;
#define UDADDR(Address) ((uAddrType) (Address << 13))
#define UDBLOCKADDR(Address) ((uAddrType) (Address << 13))
uAddrType temp;

uint8_t writeBuf[PAGE_DATA_SIZE]; // 2048, do not read spare?
uint8_t readBuf[PAGE_SIZE]; // 2176, always allocate full page size
uint32_t i, iErr;

void* mainThread(void *arg0) {
	/* Call driver init functions */
	uint32_t time;

	GPIO_init();
	SPI_init();

	GPIO_write(_SHDN, GPIO_CFG_OUT_LOW); // ADS129X off
	GPIO_write(LED_0, CONFIG_GPIO_LED_ON);

	NAND_Init(CONFIG_SPI, _NAND_CS, _FRAM_CS);
	ret = FlashReadDeviceIdentification(&devId);

	/* FFT START */
	arm_status status;
	float32_t maxValue, DCoffset;
	/* FFT END */

	while (1) {
		GPIO_toggle(LED_0);

//		time = Clock_getTicks();
		// FILTER
		arm_biquad_cascade_df2T_f32(&filtInst, inputSignal, filtSignal,
		FFT_SAMPLES_HALF);

		status = ARM_MATH_SUCCESS;
		status = arm_rfft_fast_init_f32(&S, fftSize);
		/* Process the data through the CFFT/CIFFT module */
		arm_rfft_fast_f32(&S, filtSignal, complexFFT, ifftFlag);

		// first entry is all real DC offset
		DCoffset = complexFFT[0];

		//de-interleave
		for (i = 0; i < FFT_SAMPLES_HALF; i++) {
			realFFT[i] = complexFFT[i * 2];
			imagFFT[i] = complexFFT[(i * 2) + 1];
		}

		for (i = 0; i < FFT_SAMPLES_HALF; i++) {
			angleFFT[i] = atan2f(imagFFT[i], realFFT[i]);
		}

		arm_cmplx_mag_squared_f32(complexFFT, powerFFT, FFT_SAMPLES_HALF);

		// find max bin (dominant frequency) without DC data
		arm_max_f32(&powerFFT[1], FFT_SAMPLES_HALF - 1, &maxValue, &maxIndex);
		// correct index
		maxIndex += 1;

		/*
		 for (i = 0; i < sizeof(writeBuf); i++) {
		 writeBuf[i] = rand();
		 }

		 // block erase either needs to be 'graceful'
		 // or all blocks needs to be erased on startup?
		 // minimum delete-data size = 64 pages!
		 ret = FlashBlockErase(UDADDR(iPage));
		 for (iPage = 0; iPage < 10; iPage++) {
		 ret = FlashPageProgram(UDADDR(iPage), writeBuf, sizeof(writeBuf));
		 }

		 iErr = 0;
		 for (iPage = 0; iPage < 10; iPage++) {
		 ret = FlashPageRead(UDADDR(iPage), readBuf);
		 for (i = 0; i < sizeof(writeBuf); i++) {
		 if (memcmp(&writeBuf[i],&readBuf[i],1) != 0) {
		 iErr++;
		 }
		 }
		 }
		 */

		sleep(1);
	}
}
