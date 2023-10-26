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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* #include "clock_config.h" *//* TODO: figure out our FLL/clock setup */

#include "target/core-v-mcu/include/core-v-mcu-config.h"
#include "hal/include/hal_apb_timer_unit_reg_defs.h"

#define DEFAULT_SYSTEM_CLOCK 5000000u /* Default System clock value */

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#include <stddef.h>
#ifdef __PULP_USE_LIBC
#include <assert.h>
#endif

/* Ensure stdint is only used by the compiler, and not the assembler. */
#if defined(__GNUC__)
#include <stdint.h>
#endif

#define configMTIME_BASE_ADDRESS	(TIMER_START_ADDR + REG_TIMER_VAL_LO)
#define configMTIMECMP_BASE_ADDRESS	(TIMER_START_ADDR + REG_TIMER_CMP_LO)
#define configUSE_PREEMPTION		1
#define configUSE_TICKLESS_IDLE		1
#define configUSE_IDLE_HOOK		0
#define configUSE_TICK_HOOK		0
#define configCPU_CLOCK_HZ		DEFAULT_SYSTEM_CLOCK
#define configTICK_RATE_HZ		((TickType_t)1000)
#define configMAX_PRIORITIES		(5)
/* Can be as low as 60 but some of the demo tasks that use this constant require it to be higher. */
#define configMINIMAL_STACK_SIZE ((unsigned short)800)
/* we want to put the heap into special section */
#define configAPPLICATION_ALLOCATED_HEAP	1
#define configTOTAL_HEAP_SIZE			((size_t)(64 * 1024))
#define configMAX_TASK_NAME_LEN			(16)
#define configUSE_TRACE_FACILITY		1 /* TODO: 0 */
#define configUSE_16_BIT_TICKS			0
#define configIDLE_SHOULD_YIELD			0
#define configUSE_MUTEXES			1
#define configQUEUE_REGISTRY_SIZE		8
#define configCHECK_FOR_STACK_OVERFLOW		2
#define configUSE_RECURSIVE_MUTEXES		1
#define configUSE_MALLOC_FAILED_HOOK		1
#define configUSE_APPLICATION_TASK_TAG		0
#define configUSE_COUNTING_SEMAPHORES		1
#define configGENERATE_RUN_TIME_STATS		0

// TODO: investigate (gw)
//#define configOVERRIDE_DEFAULT_TICK_CONFIGURATION    1
//#define configRECORD_STACK_HIGH_ADDRESS              1
//#define configUSE_POSIX_ERRNO                        1

/* newlib reentrancy */
#define configUSE_NEWLIB_REENTRANT	1
/* Co-routine definitions. */
#define configUSE_CO_ROUTINES		0
#define configMAX_CO_ROUTINE_PRIORITIES	(2)

/* Software timer definitions. */
#define configUSE_TIMERS		1
#define configTIMER_TASK_PRIORITY	(configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH	4
#define configTIMER_TASK_STACK_DEPTH	(configMINIMAL_STACK_SIZE)

/* Task priorities.  Allow these to be overridden. */
#ifndef uartPRIMARY_PRIORITY
#define uartPRIMARY_PRIORITY (configMAX_PRIORITIES - 3)
#endif

/* Set the following definitions to 1 to include the API function, or zero
 to exclude the API function. */
#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete			1
#define INCLUDE_vTaskCleanUpResources		1
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay			1
#define INCLUDE_eTaskGetState			1
#define INCLUDE_xTimerPendFunctionCall		1
#define INCLUDE_xTaskAbortDelay			1
#define INCLUDE_xTaskGetHandle			1
#define INCLUDE_xSemaphoreGetMutexHolder	1

/* Normal assert() semantics without relying on the provision of an assert.h
 header file. */
#ifdef __PULP_USE_LIBC
#define configASSERT(x) assert(x)
#else
#define configASSERT(x)                   \
	do {                                                                   \
		if ((x) == 0) {                                                \
			__asm volatile( "ebreak" );                            \
			taskDISABLE_INTERRUPTS();                              \
			for (;;)                                               \
				;                                              \
		}                                                              \
	} while (0)
#endif

#define configUSE_PORT_OPTIMISED_TASK_SELECTION	1
#define configKERNEL_INTERRUPT_PRIORITY		7

#endif /* FREERTOS_CONFIG_H */
