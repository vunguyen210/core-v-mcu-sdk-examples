/*
 * interrupts.c
 *
 *  Created on: Sep 25, 2023
 *      Author: cookpate
 */

/* https://docs.openhwgroup.org/projects/core-v-mcu/doc-src/interrupts.html */

#include "target/core-v-mcu/include/core-v-mcu-events.h"
#include "target/core-v-mcu/include/csr.h"

#include <stdint.h>

typedef void (*ISRHandler)( uint32_t cause );

void undefined_handler( uint32_t mcause );
extern void fc_soc_event_handler1( uint32_t mcause );

const ISRHandler isr_table[ 32 ] =
{
	undefined_handler,     // 0x00, exception handler, not handled by this ISR
	undefined_handler,     // 0x01
	undefined_handler,     // 0x02
	undefined_handler,     // 0x03, CLINT Software Event
	undefined_handler,     // 0x04
	undefined_handler,     // 0x05
	undefined_handler,     // 0x06
	// FreeRTOS tick handler
	// NOTE: target/vector.S uses the port MTIME interrupt handler
	// instead of user interrupt handler
	undefined_handler,     // 0x07, CLINT Timer Event
	undefined_handler,     // 0x08
	undefined_handler,     // 0x09
	undefined_handler,     // 0x0a
	// 2nd level event handler
	fc_soc_event_handler1, // 0x0b CLINT External Event
	undefined_handler,     // 0x0c
	undefined_handler,     // 0x0d
	undefined_handler,     // 0x0e
	undefined_handler,     // 0x0f
	undefined_handler,     // 0x10 CUSTOM FAST Timer_LO Event
	undefined_handler,     // 0x11 CUSTOM FAST Timer HI Event
	undefined_handler,     // 0x12 CUSTOM FAST Reference Clock Rise Event
	undefined_handler,     // 0x13 CUSTOM FAST Reference Clock Fall Event
	undefined_handler,     // 0x14 CUSTOM FAST I2C Event
	undefined_handler,     // 0x15 CUSTOM FAST Advanced-Timer Events0
	undefined_handler,     // 0x16 CUSTOM FAST Advanced-Timer Events1
	undefined_handler,     // 0x17 CUSTOM FAST Advanced-Timer Events2
	undefined_handler,     // 0x18 CUSTOM FAST Advanced-Timer Events3
	undefined_handler,     // 0x19 CUSTOM FAST eFPGA Events0
	undefined_handler,     // 0x1a CUSTOM FAST eFPGA Events1
	undefined_handler,     // 0x1b CUSTOM FAST eFPGA Events2
	undefined_handler,     // 0x1c CUSTOM FAST eFPGA Events3
	undefined_handler,     // 0x1d CUSTOM FAST eFPGA Events4
	undefined_handler,     // 0x1e CUSTOM FAST eFPGA Events5
	undefined_handler,     // 0x1f CUSTOM FAST Error Event
	};

uint32_t handler_count[ 32 ] =
{ 0 };
uint32_t gSpecialHandlingIRQCnt = 0;

void undefined_handler( uint32_t mcause )
{
	uint32_t RegReadVal = 0;
	if ( ( mcause == 18 ) || ( mcause == 19 ) || ( mcause == 31 ) )
	{
		gSpecialHandlingIRQCnt++;
		if ( gSpecialHandlingIRQCnt >= 20 )
		{
			RegReadVal = csr_read( CSR_MIE );
			if ( ( RegReadVal & BIT( mcause ) ) != 0 ) // Check if the event interrupt mask is open.
			{
				// close the event interrupt mask.
				csr_read_clear( CSR_MIE, BIT(mcause) );
			}
		}
	}
	else
	{
		handler_count[ mcause ]++;
	}
}

