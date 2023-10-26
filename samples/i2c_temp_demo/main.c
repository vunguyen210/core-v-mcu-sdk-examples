/*
 * Copyright 2023 AWS
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* FreeRTOS kernel includes. */
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <task.h>

/* UART */
#include "iot_uart.h"

/* I2C */
#include "iot_i2c.h"

/* Target includes */
#include "target/core-v-mcu/include/core-v-mcu-system.h"

/******************************************************************************
 * This project provides a demo program: a simple i2c style project,
 *
 * This file also implements the code that is not demo specific, including the
 * hardware setup and standard FreeRTOS hook functions.
 *
 * ENSURE TO READ THE DOCUMENTATION PAGE FOR THIS PORT AND DEMO APPLICATION ON
 * THE http://www.FreeRTOS.org WEB SITE FOR FULL INFORMATION ON USING THIS DEMO
 * APPLICATION, AND ITS ASSOCIATE FreeRTOS ARCHITECTURE PORT!
 */

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
 within this file.  See https://www.freertos.org/a00016.html */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char * pcTaskName );
void vApplicationTickHook( void );

/* Prepare hardware to run the demo. */
static void prvSetupHardware( void );

/* Demo task */
static void TempTask( void * pvParameters );
/*-----------------------------------------------------------*/
char * SOFTWARE_VERSION_STR = "I2C temp demo v0.1 - NoInt \n";

static IotUARTHandle_t uart0;
static IotI2CHandle_t i2cTemp;
static SemaphoreHandle_t shTxSemaphore;

int main( void )
{
    BaseType_t xResult;
    TaskHandle_t pxTaskHandle = { 0 };

    prvSetupHardware();

    xResult = xTaskCreate( &TempTask,
                           "UART",
                           3000U, // Task will allocate 3000 heap bytes for
                                  // stack
                           NULL,
                           2U, // Created task at second-lowest priority
                           &pxTaskHandle );

    configASSERT( xResult == pdPASS );

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following
     line will never be reached.  If the following line does execute, then
     there was insufficient FreeRTOS heap memory available for the Idle and/or
     timer tasks to be created.  See the memory management section on the
     FreeRTOS web site for more details on the FreeRTOS heap
     http://www.freertos.org/a00111.html. */

    for( ;; )
        ;
}
/*-----------------------------------------------------------*/
static void uartCallback( IotUARTOperationStatus_t xStatus,
                          void * pvUserContext )
{
    BaseType_t switchContexts = pdFALSE;
    uint16_t uLastWrite;
    int32_t iotStatus;
    if( ( xStatus == eUartWriteCompleted ) ||
        ( xStatus == eUartLastWriteFailed ) )
    {
        iotStatus = iot_uart_ioctl( uart0, eGetTxNoOfbytes, &uLastWrite );

        configASSERT( iotStatus == IOT_UART_SUCCESS );

        xSemaphoreGiveFromISR( shTxSemaphore, &switchContexts );
    }

    portYIELD_FROM_ISR( switchContexts );
}

static void i2cCallback( IotI2COperationStatus_t xStatus, void * pvUserContext )
{
    BaseType_t switchContexts = pdFALSE;
    if( ( xStatus == eI2CCompleted ) || ( xStatus == eI2CNackFromSlave ) )
    {
        xSemaphoreGiveFromISR( shTxSemaphore, &switchContexts );
    }
}

static void openUart( void )
{
    uart0 = iot_uart_open( 0 );
    configASSERT( uart0 != NULL );
    iot_uart_set_callback( uart0, uartCallback, NULL );
}

static void openI2C( void )
{
    i2cTemp = iot_i2c_open( 1 );
    configASSERT( i2cTemp != NULL );
    iot_i2c_set_callback( i2cTemp, i2cCallback, NULL );
}

static void prvSetupHardware( void )
{
    /* Init board hardware. */
    system_init();

    shTxSemaphore = xSemaphoreCreateBinary();

    openUart();
    openI2C();

    uint16_t slaveAddress = 0x4B;
    int32_t iotStatus;

    IotI2CConfig_t xI2CConfig = { .ulBusFreq = 500000, .ulMasterTimeout = 100 };

    iotStatus = iot_i2c_ioctl( i2cTemp, eI2CSetMasterConfig, &xI2CConfig );
    configASSERT( iotStatus == IOT_I2C_SUCCESS );

    iotStatus = iot_i2c_ioctl( i2cTemp, eI2CSetSlaveAddr, &slaveAddress );
    configASSERT( iotStatus == IOT_I2C_SUCCESS );

    xSemaphoreGive( shTxSemaphore );
}

/*-----------------------------------------------------------*/
static void TempTask( void * pvParameters )
{
    ( void ) pvParameters;
    TickType_t xNextWakeTime;

    int32_t iotStatus;
    uint8_t readBuffer[ 16 ] = { 0 };
    uint8_t i2cDeviceRegister = 0;
    uint8_t lengthTemp = 0;

    char * tempBuff[ 32 ];

    iotStatus = iot_i2c_ioctl( i2cTemp, eI2CSendNoStopFlag, NULL );
    configASSERT( iotStatus == IOT_I2C_SUCCESS );

    if( xSemaphoreTake( shTxSemaphore, portMAX_DELAY ) == pdTRUE )
    {
        iot_uart_write_async( uart0,
                              ( uint8_t * ) "Temperature Demo UART Test\r\n",
                              sizeof( "Temperature Demo UART Test\r\n" ) - 1U );
    }

    // Initialize xNextWakeTime - this only needs to be done once.
    xNextWakeTime = xTaskGetTickCount();

    for( ;; )
    {
        /* Place this task in the blocked state
         *  until it is time to run again. */

        vTaskDelayUntil( &xNextWakeTime, pdMS_TO_TICKS( 1000U ) );

        /* write the device register address. */
        iotStatus = iot_i2c_write_sync( i2cTemp,
                                        &i2cDeviceRegister,
                                        sizeof( i2cDeviceRegister ) );
        configASSERT( IOT_I2C_SUCCESS == iotStatus );

        /* read from i2c device for 2 bytes */
        iotStatus = iot_i2c_read_sync( i2cTemp, ( uint8_t * ) &readBuffer, 2 );
        configASSERT( IOT_I2C_SUCCESS == iotStatus );

        // Parse the temperature info from the readBuffer
        int whole = ( ( readBuffer[ 0 ] << 8 ) | readBuffer[ 1 ] ) / 128;
        int frac = ( ( readBuffer[ 0 ] << 8 ) | readBuffer[ 1 ] ) % 128;

        lengthTemp = ( uint8_t ) snprintf( ( char * ) tempBuff,
                                           32,
                                           "Current Temperature: %d.%02d C\r\n",
                                           whole,
                                           frac );

        if( xSemaphoreTake( shTxSemaphore, portMAX_DELAY ) == pdTRUE )
        {
            iot_uart_write_async( uart0, ( uint8_t * ) tempBuff, lengthTemp );
        }
    }
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
     configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
     function that will get called if a call to pvPortMalloc() fails.
     pvPortMalloc() is called internally by the kernel whenever a task, queue,
     timer or semaphore is created.  It is also called by various parts of the
     demo application.  If heap_1.c or heap_2.c are used, then the size of the
     heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
     FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
     to query the size of free heap space that remains (although it does not
     provide information on how the remaining heap might be fragmented). */
    configASSERT( 0 );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
     to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
     task.  It is essential that code added to this hook function never attempts
     to block in any way (for example, call xQueueReceive() with a block time
     specified, or call vTaskDelay()).  If the application makes use of the
     vTaskDelete() API function (as this demo application does) then it is also
     important that vApplicationIdleHook() is permitted to return to its calling
     function, because it is the responsibility of the idle task to clean up
     memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char * pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
     configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     function is called if a stack overflow is detected. */
    configASSERT( 0 );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
    /* The tick interrupt can optionally call an application defined hook
     * (or callback) function - the tick hook. The tick hook provides a
     * convenient place to implement timer functionality.
     *
     * The tick hook will only get called if configUSE_TICK_HOOK is set to 1
     * within FreeRTOSConfig.h.vApplicationTickHook() executes from within an
     * ISR so must be very short, not use much stack, and not call any API
     *  functions that don't end in "FromISR" or "FROM_ISR".
     */
}
/*-----------------------------------------------------------*/
