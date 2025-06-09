#include <FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>
//#include <task.h>
/* UART */
#include "iot_uart.h"
/* I2C */
#include "iot_i2c.h"
/* Target includes */
#include "target/core-v-mcu/include/core-v-mcu-system.h"

char * SOFTWARE_VERSION_STR = "I2C temp demo v0.1 - NoInt \n";
/* Prepare hardware to run the demo. */
static void prvSetupHardware( void );
static IotUARTHandle_t uart0;
static IotI2CHandle_t i2cTemp;



void prvSetupHardware(void)
{
    /* Init board hardware. */
    int32_t iotStatus;

    uart0 = iot_uart_open(0);
    if (uart0 == NULL) {
       for(;;);
    }
    iot_uart_write_sync(uart0, (uint8_t *)"System Initialized\r\n", sizeof("System Initialized\r\n") - 1U);

    i2cTemp = iot_i2c_open(1);
    if (i2cTemp == NULL) {
       iot_uart_write_sync(uart0, (uint8_t *)"I2C Open Failed!\r\n", sizeof("I2C Open Failed!\r\n") - 1U);
       for(;;);
    }

    uint16_t slaveAddress = 0x45;


    IotI2CConfig_t xI2CConfig = { .ulBusFreq = 500000, .ulMasterTimeout = 100 };

    iotStatus = iot_i2c_ioctl(i2cTemp, eI2CSetMasterConfig, &xI2CConfig);
    if (iotStatus != IOT_I2C_SUCCESS) {
        iot_uart_write_sync(uart0, (uint8_t *)"I2C Config Failed!\r\n", sizeof("I2C Config Failed!\r\n") - 1U);
        for(;;);
    }

    iotStatus = iot_i2c_ioctl(i2cTemp, eI2CSetSlaveAddr, &slaveAddress);
    if (iotStatus != IOT_I2C_SUCCESS) {
        iot_uart_write_sync(uart0, (uint8_t *)"I2C Slave Address Failed!\r\n", sizeof("I2C Slave Address Failed!\r\n") - 1U);
        for(;;);
    }
}


int main( void )
{

    int32_t iotStatus;
    uint8_t readBuffer[6] = {0};
    uint8_t measureCommand[2] = {0x2C, 0x06};
    char tempBuff[64];


    prvSetupHardware();


    iotStatus = iot_uart_write_sync(uart0, (uint8_t *)"Temperature & Humidity Demo\r\n",
                                    sizeof("Temperature & Humidity Demo\r\n") - 1U);
    if (iotStatus != IOT_UART_SUCCESS) {
        for(;;);
    }

    for (;;) {

        iotStatus = iot_i2c_write_sync(i2cTemp, measureCommand, sizeof(measureCommand));
        if (iotStatus != IOT_I2C_SUCCESS) {
            iot_uart_write_sync(uart0, (uint8_t *)"I2C Write Failed!\r\n", sizeof("I2C Write Failed!\r\n") - 1U);
            for(;;);
        }


        for (volatile uint32_t i = 0; i < 1000000; i++) { }

        iotStatus = iot_i2c_read_sync(i2cTemp, readBuffer, sizeof(readBuffer));
        if (iotStatus != IOT_I2C_SUCCESS) {
            iot_uart_write_sync(uart0, (uint8_t *)"I2C Read Failed!\r\n", sizeof("I2C Read Failed!\r\n") - 1U);
            for(;;);
        }


        int16_t rawTemp = ((uint16_t)readBuffer[0] << 8) | readBuffer[1];
        int32_t tempC = -45 + 175 * rawTemp / 65535;

        uint16_t rawHumidity = ((uint16_t)readBuffer[3] << 8) | readBuffer[4];
        int32_t humidity = 100 * rawHumidity / 65535;

        uint8_t lengthTemp = snprintf(tempBuff, sizeof(tempBuff),
                                              "Temperature: %ld C, Humidity:%ld%%\r\n", tempC, humidity);

//        uint8_t lengthTemp = snprintf(tempBuff, sizeof(tempBuff),
//                                                      "Temperature: %ld C\r\n", tempC);

        iotStatus = iot_uart_write_sync(uart0, (uint8_t *)tempBuff, lengthTemp);
        if (iotStatus != IOT_UART_SUCCESS) {
            for(;;);
        }
    }


    return 0;

}
/*-----------------------------------------------------------*/































