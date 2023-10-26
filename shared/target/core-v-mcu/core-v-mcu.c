/*
 * Copyright 2020 ETH Zurich
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
 * Author: Robert Balas (balasr@iis.ee.ethz.ch)
 */

#include <stdint.h>
#include <stddef.h>

#include "target/core-v-mcu/include/core-v-mcu-config.h"
#include "target/core-v-mcu/include/csr.h"

/* TODO: weird include */
#include "hal/include/hal_timer_irq.h"
#include "hal/include/hal_apb_soc_ctrl_reg_defs.h"
#include "hal/include/hal_fc_event.h"
#include "hal/include/hal_irq.h"

#include <assert.h>

/* test some assumptions we make about compiler settings */
static_assert(sizeof(uintptr_t) == 4,
              "uintptr_t is not 4 bytes. Make sure you are using -mabi=ilp32*");

/**
 * Board init code. Always call this before anything else.
 */

uint8_t setFLLFrequencyInIntegerMode(uint8_t aFLLNum, uint8_t aRefFreqInMHz,
                                     uint16_t aMultiplier,
                                     uint8_t aDivideRatio_R_Prescale,
                                     uint8_t aPS0_L1, uint8_t aPS0_L2) {
  uint8_t lSts = 0;
  volatile uint32_t *lPLLStartAddress = (uint32_t *)NULL;
  uint32_t lCounter = 0;
  uint32_t lCfgVal = 0;

  uint8_t lPS0_L1 = aPS0_L1 & 0x03;
  uint8_t lPS0_L2 = aPS0_L2 & 0xFF;

  if (aFLLNum == 0)
    lPLLStartAddress = (uint32_t *)FLL1_START_ADDR;
  else if (aFLLNum == 1)
    lPLLStartAddress = (uint32_t *)FLL2_START_ADDR;
  else if (aFLLNum == 2)
    lPLLStartAddress = (uint32_t *)FLL3_START_ADDR;
  else
    lPLLStartAddress = (uint32_t *)NULL;

  if (lPLLStartAddress != NULL) {
    if (aRefFreqInMHz >= 5) {
      if ((aMultiplier > 0) && (aMultiplier < 2048)) {
        if (aDivideRatio_R_Prescale < 16) {
          *lPLLStartAddress |= (1 << 19);      // Bypass on;
          *lPLLStartAddress |= (1 << 2);       // Reset high
          *lPLLStartAddress &= ~(uint32_t)(1U << 2);      // Reset low;
          *lPLLStartAddress &= ~(uint32_t)(1U << 18);     // PS0_EN is set to low
          *lPLLStartAddress |= (lPS0_L1 << 0); // PS0_L1 0 which gives L01 = 1
          *lPLLStartAddress |=
              (lPS0_L2
               << 4); // PS0_L2_INT 0 and PS0_L2_FRAC 0 which gives L02 = 1
          *lPLLStartAddress |=
              (0 << 12); // PS0_L2_INT 0 and PS0_L2_FRAC 0 which gives L02 = 1

          // FLL1 Config 1 register not configuring PS1
          *(lPLLStartAddress + 1) = 0;

          // FLL1 Config 2 register
          lCfgVal = 0;
          lCfgVal |= (aMultiplier << 4); // MULT_INT	0x28 = 40 (40*10 =
                                         // 400MHz) Multiplier cannot hold 0
          lCfgVal |= (1 << 27);          // INTEGER_MODE is enabled
          lCfgVal |= (aDivideRatio_R_Prescale
                      << 28); // PRESCALE value (Divide Ratio R = 1)

          *(lPLLStartAddress + 2) = lCfgVal;

          // FLL1 Config 3 register not configuring SSC
          *(lPLLStartAddress + 3) = 0;

          // FLL1 Config 4 register
          *(lPLLStartAddress + 4) = 0x64;

          // FLL1 Config 5 register
          *(lPLLStartAddress + 5) = 0x269;

          *lPLLStartAddress |= (1 << 2);  // Reset high
          *lPLLStartAddress |= (1 << 18); // PS0_EN;
          // lCounter = 0;
          while ((*(lPLLStartAddress + 4) & 0x80000000) ==
                 0) // Wait for lock detect to go high
          {
            lCounter++;
            if (lCounter >= 0x00010000) {
              lSts = 5; // Unable to achieve lock
              lCounter = 0;
              break;
            }
          }
          if (lSts == 0)
            *(lPLLStartAddress) &= ~(uint32_t)(1U << 19U); // Bypass off;
        } else {
          lSts = 1; // aDivideRatio_R_Prescale
        }
      } else {
        lSts = 2; // Invalid aMultiplier
      }
    } else {
      lSts = 3; // Invalid reference freq
    }
  } else {
    lSts = 4; // Invalid PLL number
  }
  return lSts;
}

uint8_t gQSPIIdNum = 0;

void system_init(void) {
  SocCtrl_t *soc = ( SocCtrl_t * ) SOC_CTRL_START_ADDR;

  soc->soft_reset = 1;

  timer_irq_disable();

#if FAKE_PLL == 1
  uint32_t *lFFL1StartAddress = (uint32_t *)FLL1_START_ADDR;
  uint32_t *lFFL2StartAddress = (uint32_t *)FLL2_START_ADDR;
  uint32_t *lFFL3StartAddress = (uint32_t *)FLL3_START_ADDR;
  // FLL1 is connected to soc_clk_o. Run at reference clock, use by pass.
  // FLL1 Config 0 register
  *lFFL1StartAddress = 0;
  // FLL1 Config 1 register
  *(lFFL1StartAddress + 1) =
      0x0000000C; // Already this is the default value set in HW.
  // FLL1 Config 2 register
  *(lFFL1StartAddress + 2) = 0;
  // FLL1 Config 3 register
  *(lFFL1StartAddress + 3) = 0;

  // FLL2 is connected to peripheral clock. Run at half of reference clock. Set
  // the divisor to 0 and disable bypass FLL2 Config 0 register
  *lFFL2StartAddress = 0; // Set divisor to half of reference clock.
  // FLL2 Config 1 register
  *(lFFL2StartAddress + 1) = 0; // Disable bypass.
  // FLL2 Config 2 register
  *(lFFL2StartAddress + 2) = 0;
  // FLL2 Config 3 register
  *(lFFL2StartAddress + 3) = 0;

  // FLL3 is connected to Cluster clock. Run at quarter of reference clock. Set
  // the divisor to 1 and disable bypass FLL3 Config 0 register
  *lFFL3StartAddress = 0x00000010; // Set divisor to quarter of reference clock.
  // FLL3 Config 1 register
  *(lFFL3StartAddress + 1) = 0; // Disable bypass.
  // FLL3 Config 2 register
  *(lFFL3StartAddress + 2) = 0;
  // FLL3 Config 3 register
  *(lFFL3StartAddress + 3) = 0;

#elif (PERCEPTIA_PLL == 1)
  setFLLFrequencyInIntegerMode(0, 10, 40, 1, 0, 1); // 400

  setFLLFrequencyInIntegerMode(1, 10, 40, 1, 0, 2); // 200

  setFLLFrequencyInIntegerMode(2, 10, 40, 1, 0, 4); // 100
#else
#error "Enable any one of the PLL configurations FAKE_PLL or PERCEPTIA_PLL"
#endif

  /* mtvec is set in crt0.S */

  /* Setup soc events handler. */
  // pi_fc_event_handler_init(FC_SOC_EVENT);
  pi_fc_event_handler_init( 11 );

  /* TODO: I$ enable*/
  /* enable core level interrupt (mie) */
  irq_clint_enable();

  ( void ) csr_read(CSR_MIE);
}
