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
#include <task.h>

#include <stddef.h>

/* UART */
#include "iot_uart.h"

#include "iot_gpio.h"

/* Target includes */
#include "target/core-v-mcu/include/core-v-mcu-system.h"

// Variables to configure ExpressLink with
#define WIFI_SSID                   ""
#define MQTT_ENDPOINT               ""
#define WIFI_PASSPHRASE             ""

#define EOL                         "\r\n"

#define WIFI_SSID_COMMAND           ( "AT+CONF SSID=" WIFI_SSID EOL )
#define WIFI_SSID_COMMAND_LEN       ( sizeof( WIFI_SSID_COMMAND ) - 1UL )

#define WIFI_PASSPHRASE_COMMAND     ( "AT+CONF Passphrase=" WIFI_PASSPHRASE EOL )
#define WIFI_PASSPHRASE_COMMAND_LEN ( sizeof( WIFI_PASSPHRASE_COMMAND ) - 1UL )

#define MQTT_ENDPOINT_COMMAND       ( "AT+CONF Endpoint=" MQTT_ENDPOINT EOL )
#define MQTT_ENDPOINT_COMMAND_LEN   ( sizeof( MQTT_ENDPOINT_COMMAND ) - 1UL )

#define INTERCHAR_TIMEOUT_TICKS     ( pdMS_TO_TICKS( 50U ) )

/******************************************************************************
 * This project provides a demo program: a simple uart style project,
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
static void UARTTask( void * pvParameters );
/*-----------------------------------------------------------*/
char * SOFTWARE_VERSION_STR = "ExpressLink demo v0.1 - NoInt \n";

static IotUARTHandle_t uart0;
static IotUARTHandle_t uart1;

static IotGpioHandle_t reset;
static IotGpioHandle_t event;
static IotGpioHandle_t wake;

static SemaphoreHandle_t uart0TxSemaphore;

static SemaphoreHandle_t uart1RxSemaphore;

int main( void )
{
    BaseType_t xResult;
    TaskHandle_t pxTaskHandle = { 0 };

    prvSetupHardware();

    xResult = xTaskCreate( &UARTTask,
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
static void uart0CallbackISR( IotUARTOperationStatus_t xStatus,
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

        xSemaphoreGiveFromISR( uart0TxSemaphore, &switchContexts );
    }

    portYIELD_FROM_ISR( switchContexts );
}

static void uart1ReceiveISR( IotUARTOperationStatus_t xStatus,
                             void * pvUserContext )
{
    BaseType_t switchContexts = pdFALSE;

    xSemaphoreGiveFromISR( uart1RxSemaphore, &switchContexts );

    portYIELD_FROM_ISR( switchContexts );
}

static void openUart( void )
{
    uart0 = iot_uart_open( 0 );
    configASSERT( uart0 != NULL );
    uart1 = iot_uart_open( 1 );
    configASSERT( uart1 != NULL );
    iot_uart_set_callback( uart0, uart0CallbackISR, NULL );
    iot_uart_set_callback( uart1, uart1ReceiveISR, NULL );
}

static void prvSetupHardware( void )
{
    /* Init board hardware. */
    system_init();

    uart0TxSemaphore = xSemaphoreCreateBinary();
    uart1RxSemaphore = xSemaphoreCreateBinary();
    openUart();

    // IO 26, 27, 28 for XL GPIO pins
    wake = iot_gpio_open( 26 - 7 );
    reset = iot_gpio_open( 27 - 7 );
    event = iot_gpio_open( 28 - 7 );

    IotGpioDirection_t input = eGpioDirectionInput;
    IotGpioDirection_t output = eGpioDirectionOutput;
    IotGpioOutputMode_t push = eGpioOpenDrain;
    uint8_t enableGpio = 2;

    iot_gpio_ioctl( reset, eSetGpioFunction, &enableGpio );
    iot_gpio_ioctl( event, eSetGpioFunction, &enableGpio );
    iot_gpio_ioctl( wake, eSetGpioFunction, &enableGpio );

    iot_gpio_ioctl( reset, eSetGpioDirection, &output );
    iot_gpio_ioctl( event, eSetGpioDirection, &input );
    iot_gpio_ioctl( wake, eSetGpioDirection, &output );

    iot_gpio_ioctl( reset, eSetGpioOutputMode, &push );
    iot_gpio_ioctl( wake, eSetGpioOutputMode, &push );

    iot_gpio_write_sync( reset, 0 );
    iot_gpio_write_sync( wake, 0 );
}

static size_t ExpressLinkCommand( char * command,
                                  size_t commandLen,
                                  uint8_t * output,
                                  size_t outputLen,
                                  TickType_t timeout )
{
    uint16_t received = 0;
    uint16_t lastReceived = 0;

    iot_uart_read_async( uart1, output, outputLen );

    iot_uart_write_sync( uart1, ( uint8_t * ) command, commandLen );

    TickType_t start = xTaskGetTickCount();

    while( 1 )
    {
        if( xTaskGetTickCount() > start + timeout )
        {
            if( iot_uart_cancel( uart1 ) == IOT_UART_NOTHING_TO_CANCEL )
            {
                xSemaphoreTake( uart1RxSemaphore, 0 );
            }
            break;
        }

        if( xSemaphoreTake( uart1RxSemaphore, INTERCHAR_TIMEOUT_TICKS ) ==
            pdTRUE )
        {
            break;
        }

        iot_uart_ioctl( uart1, eGetRxNoOfbytes, &received );

        if( ( received > 0 ) && ( received == lastReceived ) )
        {
            if( iot_uart_cancel( uart1 ) == IOT_UART_NOTHING_TO_CANCEL )
            {
                xSemaphoreTake( uart1RxSemaphore, 0 );
            }
            break;
        }

        lastReceived = received;
    }

    iot_uart_ioctl( uart1, eGetRxNoOfbytes, &received );

    return ( size_t ) received;
}

/*-----------------------------------------------------------*/
static void UARTTask( void * pvParameters )
{
    ( void ) pvParameters;
    TickType_t xNextWakeTime;

    iot_uart_write_async( uart0,
                          ( uint8_t * ) "UART Task\r\n",
                          sizeof( "UART Task\r\n" ) - 1U );

    vTaskDelay( pdMS_TO_TICKS( 50 ) );
    iot_gpio_write_sync( reset, 1 );

    vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    uint8_t hasEvent = 0;
    while( hasEvent == 0 )
    {
        iot_gpio_read_sync( event, &hasEvent );
    }

    xSemaphoreTake( uart0TxSemaphore, portMAX_DELAY );

    uint8_t buffer[ 128 ] = { 0 };
    size_t received = 0;

    received = ExpressLinkCommand( WIFI_SSID_COMMAND,
                                   WIFI_SSID_COMMAND_LEN,
                                   buffer,
                                   sizeof( buffer ),
                                   pdMS_TO_TICKS( 1000 ) );

    configASSERT( received == ( sizeof( "OK" EOL ) - 1UL ) );

    received = ExpressLinkCommand( WIFI_PASSPHRASE_COMMAND,
                                   WIFI_PASSPHRASE_COMMAND_LEN,
                                   buffer,
                                   sizeof( buffer ),
                                   pdMS_TO_TICKS( 1000 ) );

    configASSERT( received == ( sizeof( "OK" EOL ) - 1UL ) );

    received = ExpressLinkCommand( "AT+CONF Topic1=myTopic" EOL,
                                   sizeof( "AT+CONF TOPIC1=myTopic" EOL ) - 1UL,
                                   buffer,
                                   sizeof( buffer ),
                                   pdMS_TO_TICKS( 1000 ) );

    configASSERT( received == ( sizeof( "OK" EOL ) - 1UL ) );

    received = ExpressLinkCommand( MQTT_ENDPOINT_COMMAND,
                                   MQTT_ENDPOINT_COMMAND_LEN,
                                   buffer,
                                   sizeof( buffer ),
                                   pdMS_TO_TICKS( 1000 ) );

    configASSERT( received == ( sizeof( "OK" EOL ) - 1UL ) );

    while( 1 )
    {
        received = ExpressLinkCommand( "AT+CONNECT" EOL,
                                       sizeof( "AT+CONNECT" EOL ) - 1UL,
                                       buffer,
                                       sizeof( buffer ),
                                       pdMS_TO_TICKS( 20000 ) );
        if( received == ( sizeof( "OK 1 CONNECTED" EOL ) - 1UL ) )
        {
            break;
        }
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }

    // Initialize xNextWakeTime - this only needs to be done once.
    xNextWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        received = ExpressLinkCommand( "AT+SEND1 Hello World!" EOL,
                                       sizeof( "AT+SEND1 Hello World!" EOL ) -
                                           1UL,
                                       buffer,
                                       sizeof( buffer ),
                                       pdMS_TO_TICKS( 1000 ) );

        /* Place this task in the blocked state
         *  until it is time to run again. */

        vTaskDelayUntil( &xNextWakeTime, pdMS_TO_TICKS( 1000U ) );
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
