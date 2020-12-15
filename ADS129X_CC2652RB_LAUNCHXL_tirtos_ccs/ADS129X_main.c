/* For usleep() */
#include <Definitions.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include <ADS129X.h>

// user defined
int32_t status;
int32_t ch1;
int32_t ch2;
int32_t ch3;
int32_t ch4;

int32_t ch1Buf[256];
int32_t ch2Buf[256];
int32_t ch3Buf[256];
int32_t ch4Buf[256];
int i, j;

/*
 *  ======== mainThread ========
 */
void* mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    SPI_init();

//    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
//    RF_cmdPropTx.pPkt = packet;
//    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;

    GPIO_write(LED_0, CONFIG_GPIO_LED_ON); // init
    GPIO_write(_SHDN, GPIO_CFG_OUT_HIGH); // turn on ADS129X

    usleep(100000); // wait for EEG
    ADS_init(CONFIG_SPI_EEG, _EEG_CS);

    while (1)
    {
//        usleep(200000);
        GPIO_toggle(LED_0);

        for (i = 0; i < 256; i++)
        {
//            for (j = 0; j < 3; j++)
//            {
            while (GPIO_read(_EEG_DRDY))
            {
            } // block
            ADS_updateData(&status, &ch1, &ch2, &ch3, &ch4);
//            }
            ch1Buf[i] = ch1;
            ch2Buf[i] = ch2;
            ch3Buf[i] = ch3;
            ch4Buf[i] = ch4;
        }
    }
}
