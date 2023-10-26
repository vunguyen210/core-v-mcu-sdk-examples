/*
 * Copyright (C) 2019 ETH Zurich and University of Bologna
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

/* Driver to configure PULP timer as periodic interrupt source */
/* Author: Robert Balas (balasr@iis.ee.ethz.ch)
 *         Germain Haugou (germain.haugou@iis.ee.ethz.ch)
 */

#include "hal/include/hal_timer_irq.h"
#include "target/core-v-mcu/include/core-v-mcu-config.h"
#include "hal/include/hal_apb_timer_unit_reg_defs.h"

/* TODO: used to measure elapsed time since last "visit" */
static uint32_t last_count;
int timer_irq_disable( void )
{
	ApbTimerUnit_t * timer = ( ApbTimerUnit_t* ) TIMER_START_ADDR;

	timer->cfg_reg_lo = 0U;

	return 0;
}

int timer_irq_init( uint32_t ticks )
{
	ApbTimerUnit_t * timer = ( ApbTimerUnit_t* ) TIMER_START_ADDR;

	/* set the interrupt interval */
	timer_irq_set_timeout( ticks, false );

	/* We use only one of the 32-bit timer, leaving the other half available
	 * as an additional timer. We didn't opt for using both together as
	 * 64-bit timer.
	 *
	 * Enable timer, use 32khz ref clock as source. Timer will reset
	 * automatically to zero after causing an interrupt.
	 */
	timer->cfg_reg_lo_b.irq_bit = 1;
	timer->cfg_reg_lo_b.iem_bit = 1;
	timer->cfg_reg_lo_b.cmp_clr_bit = 1;
	timer->cfg_reg_lo_b.mode_mtime_bit = 1;
	timer->cfg_reg_lo_b.mode_64_bit = 1;
	timer->cfg_reg_lo_b.enable_bit = 1;


	return 0;
}

int timer_irq_set_timeout( uint32_t ticks, bool idle )
{
	/* TODO: what does this parameter mean?*/
	( void ) idle;
	/* fast reset, value doesn't matter */
	ApbTimerUnit_t * timer = ( ApbTimerUnit_t* ) TIMER_START_ADDR;
	timer->timer_cmp_lo = ticks;
	timer->cfg_reg_lo_b.reset_bit = 1;

	return 0;
}

/* TODO: implement */
uint32_t timer_irq_clock_elapsed()
{
	return 0;
}

uint32_t timer_irq_cycle_get_32()
{
	ApbTimerUnit_t * timer = ( ApbTimerUnit_t* ) TIMER_START_ADDR;
	return timer->timer_val_lo_b.timer_val_lo;
}
