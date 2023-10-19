/* FreeRTOS kernel includes. */
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

/* UART */
#include "iot_uart.h"

/* Target includes */
#include "target/core-v-mcu/include/core-v-mcu-system.h"

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
char * SOFTWARE_VERSION_STR = "UART demo v0.1 - NoInt \n";

static IotUARTHandle_t uart0;
static SemaphoreHandle_t shTxSemaphore;

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

static void openUart( void )
{
    uart0 = iot_uart_open( 0 );
    configASSERT( uart0 != NULL );
    iot_uart_set_callback( uart0, uartCallback, NULL );
}

static void prvSetupHardware( void )
{
    /* Init board hardware. */
    system_init();

    shTxSemaphore = xSemaphoreCreateBinary();

    openUart();

    xSemaphoreGive( shTxSemaphore );
}

/*-----------------------------------------------------------*/
static void UARTTask( void * pvParameters )
{
    ( void ) pvParameters;
    TickType_t xNextWakeTime;

    if( xSemaphoreTake( shTxSemaphore, portMAX_DELAY ) == pdTRUE )
    {
        iot_uart_write_async( uart0,
                              ( uint8_t * ) "UART Task\r\n",
                              sizeof( "UART Task\r\n" ) - 1U );
    }

    // Initialize xNextWakeTime - this only needs to be done once.
    xNextWakeTime = xTaskGetTickCount();

    for( ;; )
    {
        /* Place this task in the blocked state
         *  until it is time to run again. */

        vTaskDelayUntil( &xNextWakeTime, pdMS_TO_TICKS( 1000U ) );

        if( xSemaphoreTake( shTxSemaphore, portMAX_DELAY ) == pdTRUE )
        {
            iot_uart_write_async( uart0,
                                  ( uint8_t * ) "Hello World!\r\n",
                                  sizeof( "Hello World!\r\n" ) - 1U );
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
