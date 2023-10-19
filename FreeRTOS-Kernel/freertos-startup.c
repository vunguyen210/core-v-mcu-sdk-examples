/*
 * freertos-port.c
 *
 *  Created on: Sep 25, 2023
 *      Author: cookpate
 */

#include "FreeRTOS.h"
#include "task.h"

#include "target/core-v-mcu/include/core-v-mcu-properties.h"

#include "hal/include/hal_timer_irq.h"

/* Allocate heap to special section. Note that we have no references in the
 * whole program to this variable (since its just here to allocate space in the
 * section for our heap), so when using LTO it will be removed. We force it to
 * stay with the "used" attribute
 */
__attribute__((section(".heap"), used))  uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];

/* Inform linker script (target/core-v-mcu/core-v-mcu.ld)
 * about .heap section size. Note: GNU ld seems to internally represent integers
 * with the bfd_vma type, that is a type that can
 * contain memory addresses (typdef'd to some int type depending on the
 * architecture). uint32_t seems to me the most fitting candidate for rv32.
 */
uint32_t __heap_size = configTOTAL_HEAP_SIZE;

void vPortSetupTimerInterrupt( void )
{
	timer_irq_init( ARCHI_FPGA_FREQUENCY / configTICK_RATE_HZ );
}

void freertos_risc_v_application_interrupt_handler( uint32_t mcause )
{
	/* defined in interrupts.c */
	extern void (*isr_table[ 32 ])( uint32_t );
	isr_table[ mcause & 0x1f ]( mcause & 0x1f );
}

void freertos_risc_v_application_exception_handler( uint32_t mcause )
{
	// ECALL (mcause == 11) is handled by the FreeRTOS port

	if ( mcause == 2 )
	{
		// illegal instruction
	}

	else if ( mcause == 3 )
	{
		// EBREAK
	}
}

