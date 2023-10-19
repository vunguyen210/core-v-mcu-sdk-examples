/*
 * Copyright 2020 GreenWaves Technologies
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

/*
 * hal_fc_event.c
 *
 *  Created on: Feb 19, 2021
 *      Author: qlblue
 */

#include "hal/include/hal_fc_event.h"

#include "hal/include/hal_apb_event_cntrl_reg_defs.h"
#include "hal/include/hal_apb_soc_ctrl_reg_defs.h"

#include "target/core-v-mcu/include/csr.h"
#include "target/core-v-mcu/include/core-v-mcu-config.h"

#include <stdint.h>
#include <stddef.h>

/*******************************************************************************
 * Variables, macros, structures,... definition
 ******************************************************************************/

/*******************************************************************************
 * Function definition
 ******************************************************************************/

static void fc_event_null_event( void *arg );

static volatile pi_fc_event_handler_t fc_event_handlers[ NB_SOC_EVENTS ];
static void * fc_handler_params[ NB_SOC_EVENTS ];

void pi_fc_event_handler_init( uint32_t fc_event_irq )
{
	uint32_t event_id;
	ApbEventCntrl_t * const eventCtrl =
		( ApbEventCntrl_t* ) SOC_EVENT_GEN_START_ADDR;

	csr_read_clear(CSR_MIE, BIT(fc_event_irq));

	eventCtrl->event_mask0 = UINT32_MAX;
	eventCtrl->event_mask1 = UINT32_MAX;
	eventCtrl->event_mask2 = UINT32_MAX;
	eventCtrl->event_mask3 = UINT32_MAX;
	eventCtrl->event_mask4 = UINT32_MAX;
	eventCtrl->event_mask5 = UINT32_MAX;
	eventCtrl->event_mask6 = UINT32_MAX;
	eventCtrl->event_mask7 = UINT32_MAX;

	for( event_id = 0U; event_id < NB_SOC_EVENTS; ++event_id )
	{
		pi_fc_event_handler_clear( event_id );
	}

	csr_read_set(CSR_MIE, BIT(fc_event_irq));
}

void pi_fc_event_handler_set( uint32_t event_id,
	pi_fc_event_handler_t event_handler, void *handler_param )
{
	if ( event_handler != NULL )
	{
		fc_event_handlers[ event_id ] = event_handler;
		fc_handler_params[ event_id ] = handler_param;
	}
}

void pi_fc_event_handler_clear( uint32_t event_id )
{
	fc_event_handlers[ event_id ] =
		( pi_fc_event_handler_t ) fc_event_null_event;
	fc_handler_params[ event_id ] = NULL;
}

/* TODO: Use Eric's FIRQ ABI */
void fc_soc_event_handler1( uint32_t mcause )
{
	ApbEventCntrl_t * const eventCtrl =
		( ApbEventCntrl_t* ) SOC_EVENT_GEN_START_ADDR;

	const uintptr_t event_id = eventCtrl->event_fifo_b.event_id;

	( void ) mcause;

	/* redirect to handler with jump table */
	fc_event_handlers[ event_id ] ( fc_handler_params[ event_id ] );
}

void pi_fc_event_enable( uint32_t event_id )
{
	ApbEventCntrl_t * const eventCtrl =
		( ApbEventCntrl_t* ) SOC_EVENT_GEN_START_ADDR;

	if ( event_id < 256U )
	{
		const uint32_t offset = event_id / 32U;
		const uint32_t bit = 1U << ( event_id & 0x1FU );

		switch ( offset )
		{
		case 0:
			eventCtrl->event_mask0 &= ~bit;
			break;
		case 1:
			eventCtrl->event_mask1 &= ~bit;
			break;
		case 2:
			eventCtrl->event_mask2 &= ~bit;
			break;
		case 3:
			eventCtrl->event_mask3 &= ~bit;
			break;
		case 4:
			eventCtrl->event_mask4 &= ~bit;
			break;
		case 5:
			eventCtrl->event_mask5 &= ~bit;
			break;
		case 6:
			eventCtrl->event_mask6 &= ~bit;
			break;
		case 7:
			eventCtrl->event_mask7 &= ~bit;
			break;

		}
	}
}

void pi_fc_event_disable( uint32_t event_id )
{
	ApbEventCntrl_t * const eventCtrl =
		( ApbEventCntrl_t* ) SOC_EVENT_GEN_START_ADDR;

	if ( event_id < NB_SOC_EVENTS )
	{
		const uint32_t offset = event_id / 32U;
		const uint32_t bit = 1U << ( event_id & 0x1FU );

		switch ( offset )
		{
		case 0:
			eventCtrl->event_mask0 |= bit;
			break;
		case 1:
			eventCtrl->event_mask1 |= bit;
			break;
		case 2:
			eventCtrl->event_mask2 |= bit;
			break;
		case 3:
			eventCtrl->event_mask3 |= bit;
			break;
		case 4:
			eventCtrl->event_mask4 |= bit;
			break;
		case 5:
			eventCtrl->event_mask5 |= bit;
			break;
		case 6:
			eventCtrl->event_mask6 |= bit;
			break;
		case 7:
			eventCtrl->event_mask7 |= bit;
			break;
		}
	}
}

static void fc_event_null_event( void *arg )
{
	return;
}

