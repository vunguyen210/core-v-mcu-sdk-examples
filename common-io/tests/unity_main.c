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
 *
 * unity_main.c
 *
 *  Created on: Sep 21, 2023
 *      Author: cookpate
 */

#include "unity.h"
#include "unity_fixture.h"
#include "unity_internals.h"

#include "FreeRTOS.h"
#include "task.h"

/* Target includes */
#include "target/core-v-mcu/include/core-v-mcu-system.h"

#include "iot_gpio.h"

static void unity_main( void *param );

int main( void )
{
	BaseType_t xResult;
	TaskHandle_t pxTaskHandle =
	{ 0 };

	system_init();

	xResult = xTaskCreate( &unity_main, "UnityTest", 3000U, // Task will allocate 3000 heap bytes for stack
		NULL, 2U,	// Created task at second-lowest priority
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
	
	configASSERT(pdFAIL);

	for ( ;; )
		;
}

static void unity_main( void *param )
{
	( void ) param;
	UNITY_BEGIN( );
	RUN_TEST_GROUP( Common_IO );
	( void ) UNITY_END( );

	// if the program execution reaches here, the unity tests have passed

	IotGpioHandle_t led0;
	// GPIO configuration
	int32_t function = 2;
	IotGpioDirection_t dir = eGpioDirectionOutput;
	IotGpioOutputMode_t mode = eGpioPushPull;

	// IO_11 on mode 2 is mapped to gpio_4
	// IO_11 maps to LED[0] on the Nexys A7
	led0 = iot_gpio_open( 11 - 7 );
	iot_gpio_ioctl( led0, eSetGpioFunction, &function );

	// Set gpio_4 to digital output
	iot_gpio_ioctl( led0, eSetGpioDirection, &dir );
	iot_gpio_ioctl( led0, eSetGpioOutputMode, &mode );

	// Turn LED on to confirm test success
	iot_gpio_write_sync( led0, 1U );

	vTaskDelay( portMAX_DELAY );
}

void SET_TEST_IOT_UART_CONFIG( int testSet )
{

}

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

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
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
