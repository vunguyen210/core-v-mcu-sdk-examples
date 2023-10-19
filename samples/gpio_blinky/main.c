/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

/* c stdlib */
#include <stdbool.h>

/* GPIO */
#include "iot_gpio.h"

/* Target includes */
#include "target/core-v-mcu/include/core-v-mcu-system.h"

/******************************************************************************
 * This project provides a demo program: a simple blinky style project,
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
static void blinkyTask( void * pvParameters );
/*-----------------------------------------------------------*/
char * SOFTWARE_VERSION_STR = "blinky demo v0.1 - NoInt \n";

static IotGpioHandle_t led0;

int main( void )
{
    BaseType_t xResult;
    TaskHandle_t pxTaskHandle = { 0 };

    prvSetupHardware();

    xResult = xTaskCreate( &blinkyTask,
                           "Blinky",
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

static void prvSetupHardware( void )
{
    // GPIO configuration
    int32_t function = 2;
    IotGpioDirection_t dir = eGpioDirectionOutput;
    IotGpioOutputMode_t mode = eGpioPushPull;

    /* Init board hardware. */
    system_init();

    // IO_11 on mode 2 is mapped to gpio_4
    // IO_11 maps to LED[0] on the Nexys A7
    led0 = iot_gpio_open( 4 );
    iot_gpio_ioctl( led0, eSetGpioFunction, &function );

    // Set gpio_4 to digital output
    iot_gpio_ioctl( led0, eSetGpioDirection, &dir );
    iot_gpio_ioctl( led0, eSetGpioOutputMode, &mode );
    // Turn LED on to confirm initialization
    iot_gpio_write_sync( led0, 1U );
}

/*-----------------------------------------------------------*/
static void blinkyTask( void * pvParameters )
{
    ( void ) pvParameters;
    TickType_t xNextWakeTime;
    bool on;

    on = true;
    // Initialize xNextWakeTime - this only needs to be done once.
    xNextWakeTime = xTaskGetTickCount();

    for( ;; )
    {
        /* Place this task in the blocked state
         *  until it is time to run again. */

        vTaskDelayUntil( &xNextWakeTime, pdMS_TO_TICKS( 1000U ) );

        on = !on;
        iot_gpio_write_sync( led0, on ? 1U : 0U );
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
