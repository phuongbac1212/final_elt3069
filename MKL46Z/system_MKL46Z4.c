/*
** ###################################################################
**     Processors:          MKL46Z128VLH4
**                          MKL46Z128VLL4
**                          MKL46Z128VMC4
**                          MKL46Z256VLH4
**                          MKL46Z256VLL4
**                          MKL46Z256VMC4
**                          MKL46Z256VMP4
**
**     Compilers:           Keil ARM C/C++ Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    KL46P121M48SF4RM, Rev.2, Dec 2012
**     Version:             rev. 3.4, 2014-10-14
**     Build:               b151217
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright (c) 2015 Freescale Semiconductor, Inc.
**     All rights reserved.
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2012-10-16)
**         Initial version.
**     - rev. 2.0 (2012-12-12)
**         Update to reference manual rev. 1.
**     - rev. 2.1 (2013-04-05)
**         Changed start of doxygen comment.
**     - rev. 2.2 (2013-04-12)
**         SystemInit function fixed for clock configuration 1.
**         Name of the interrupt num. 31 updated to reflect proper function.
**     - rev. 2.3 (2013-08-15)
**         Update of I2S register TCR2.
**         Access restriction of some registers fixed.
**     - rev. 3.0 (2014-05-14)
**         Register accessor macros added to the memory map.
**         Symbols for Processor Expert memory map compatibility added to the memory map.
**         Update of SystemInit() and SystemCoreClockUpdate() functions.
**     - rev. 3.1 (2014-07-16)
**         Module access macro module_BASES replaced by module_BASE_PTRS.
**         System initialization and startup updated.
**     - rev. 3.2 (2014-07-25)
**         System initialization updated:
**         - Access to the registers unified using CMSIS device data structures.
**     - rev. 3.3 (2014-08-28)
**         Update of system files - default clock configuration changed, fix of OSC initialization.
**         Update of startup files - possibility to override DefaultISR added.
**     - rev. 3.4 (2014-10-14)
**         Renamed interrupt vector LPTimer to LPTMR0
**
** ###################################################################
*/

/*!
 * @file MKL46Z4
 * @version 3.4
 * @date 2014-10-14
 * @brief Device specific configuration file for MKL46Z4 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "fsl_device_registers.h"



/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {
#if (DISABLE_WDOG)
  /* Disable the WDOG module */
  /* SIM_COPC: COPT=0,COPCLKS=0,COPW=0 */
  SIM->COPC = (uint32_t)0x00u;
#endif /* (DISABLE_WDOG) */
#if (CLOCK_SETUP == 0)
  /* SIM->CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=2,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = (uint32_t)0x00020000UL; /* Update system prescalers */
  /* Switch to FEI Mode */
  /* MCG->C1: CLKS=0,FRDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (uint8_t)0x06U;
  /* MCG_C2: LOCRE0=0,RANGE0=0,HGO0=0,EREFS0=0,LP=0,IRCS=0 */
  MCG->C2 &= (uint8_t)~(uint8_t)0xBFU;
  /* MCG->C4: DMX32=0,DRST_DRS=1 */
  MCG->C4 = (uint8_t)((MCG->C4 & (uint8_t)~(uint8_t)0xC0U) | (uint8_t)0x20U);
  /* OSC0->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = (uint8_t)0x80U;
  /* MCG->C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0 */
  MCG->C5 = (uint8_t)0x00U;
  /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
  MCG->C6 = (uint8_t)0x00U;
  while((MCG->S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
  }
  while((MCG->S & 0x0CU) != 0x00U) {    /* Wait until output of the FLL is selected */
  }
#elif (CLOCK_SETUP == 1)
  /* SIM->SCGC5: PORTA=1 */
  SIM->SCGC5 |= (uint32_t)0x0200UL;     /* Enable clock gate for ports to enable pin routing */
  /* SIM->CLKDIV1: OUTDIV1=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = (uint32_t)0x10010000UL; /* Update system prescalers */
  /* PORTA->PCR18: ISF=0,MUX=0 */
  PORTA->PCR[18] &= (uint32_t)~0x01000700UL;
  /* PORTA->PCR19: ISF=0,MUX=0 */
  PORTA->PCR[19] &= (uint32_t)~0x01000700UL;
  /* Switch to FBE Mode */
  /* MCG_C2: LOCRE0=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0 */
  MCG->C2 = (uint8_t)((MCG->C2 & (uint8_t)~(uint8_t)0x9BU) | (uint8_t)0x24U);
  /* OSC0->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=1,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = (uint8_t)0x80U;
  /* MCG_C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (uint8_t)0x9AU;
  /* MCG->C4: DMX32=0,DRST_DRS=0 */
  MCG->C4 &= (uint8_t)~(uint8_t)0xE0U;
  /* MCG->C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=1 */
  MCG->C5 = (uint8_t)0x01U;
  /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
  MCG->C6 = (uint8_t)0x00U;
  while((MCG->S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
  }
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  /* Switch to PBE Mode */
  /* MCG->C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=0 */
  MCG->C6 = (uint8_t)0x40U;
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  while((MCG->S & MCG_S_LOCK0_MASK) == 0x00U) { /* Wait until locked */
  }
  /* Switch to PEE Mode */
  /* MCG->C1: CLKS=0,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (uint8_t)0x1AU;
  while((MCG->S & 0x0CU) != 0x0CU) {    /* Wait until output of the PLL is selected */
  }
#elif (CLOCK_SETUP == 2)
  /* SIM->SCGC5: PORTA=1 */
  SIM->SCGC5 |= (uint32_t)0x0200UL;     /* Enable clock gate for ports to enable pin routing */
  /* SIM->CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = (uint32_t)0x00000000UL; /* Update system prescalers */
  /* PORTA->PCR18: ISF=0,MUX=0 */
  PORTA->PCR[18] &= (uint32_t)~0x01000700UL;
  /* PORTA->PCR19: ISF=0,MUX=0 */
  PORTA->PCR[19] &= (uint32_t)~0x01000700UL;
  /* Switch to FBE Mode */
  /* MCG->C2: LOCRE0=0,??=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0 */
  MCG->C2 = (uint8_t)0x24U;
  /* OSC0->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=1,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = (uint8_t)0x80U;
  /* MCG->C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (uint8_t)0x9AU;
  /* MCG->C4: DMX32=0,DRST_DRS=0 */
  MCG->C4 &= (uint8_t)~(uint8_t)0xE0U;
  /* MCG->C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0 */
  MCG->C5 = (uint8_t)0x00U;
  /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
  MCG->C6 = (uint8_t)0x00U;
  while((MCG->S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
  }
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  /* Switch to BLPE Mode */
  /* MCG_C2: LOCRE0=0,RANGE0=2,HGO0=0,EREFS0=1,LP=1,IRCS=0 */
  MCG->C2 = (uint8_t)((MCG->C2 & (uint8_t)~(uint8_t)0x99U) | (uint8_t)0x26U);
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
#endif /* (CLOCK_SETUP == 2) */
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {

  uint32_t MCGOUTClock;                /* Variable to store output clock frequency of the MCG module */
  uint16_t Divider;

  if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x00U) {
    /* Output of FLL or PLL is selected */
    if ((MCG->C6 & MCG_C6_PLLS_MASK) == 0x00U) {
      /* FLL is selected */
      if ((MCG->C1 & MCG_C1_IREFS_MASK) == 0x00U) {
        /* External reference clock is selected */
        MCGOUTClock = CPU_XTAL_CLK_HZ; /* System oscillator drives MCG clock */
        if ((MCG->C2 & MCG_C2_RANGE0_MASK) != 0x00U) {
          switch (MCG->C1 & MCG_C1_FRDIV_MASK) {
          case 0x38U:
            Divider = 1536U;
            break;
          case 0x30U:
            Divider = 1280U;
            break;
          default:
            Divider = (uint16_t)(32LU << ((MCG->C1 & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT));
            break;
          }
        } else {/* ((MCG->C2 & MCG_C2_RANGE_MASK) != 0x00U) */
          Divider = (uint16_t)(1LU << ((MCG->C1 & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT));
        }
        MCGOUTClock = (MCGOUTClock / Divider); /* Calculate the divided FLL reference clock */
      } else { /* (!((MCG->C1 & MCG_C1_IREFS_MASK) == 0x00U)) */
        MCGOUTClock = CPU_INT_SLOW_CLK_HZ; /* The slow internal reference clock is selected */
      } /* (!((MCG->C1 & MCG_C1_IREFS_MASK) == 0x00U)) */
      /* Select correct multiplier to calculate the MCG output clock  */
      switch (MCG->C4 & (MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK)) {
        case 0x00U:
          MCGOUTClock *= 640U;
          break;
        case 0x20U:
          MCGOUTClock *= 1280U;
          break;
        case 0x40U:
          MCGOUTClock *= 1920U;
          break;
        case 0x60U:
          MCGOUTClock *= 2560U;
          break;
        case 0x80U:
          MCGOUTClock *= 732U;
          break;
        case 0xA0U:
          MCGOUTClock *= 1464U;
          break;
        case 0xC0U:
          MCGOUTClock *= 2197U;
          break;
        case 0xE0U:
          MCGOUTClock *= 2929U;
          break;
        default:
          break;
      }
    } else { /* (!((MCG->C6 & MCG_C6_PLLS_MASK) == 0x00U)) */
      /* PLL is selected */
      Divider = (((uint16_t)MCG->C5 & MCG_C5_PRDIV0_MASK) + 0x01U);
      MCGOUTClock = (uint32_t)(CPU_XTAL_CLK_HZ / Divider); /* Calculate the PLL reference clock */
      Divider = (((uint16_t)MCG->C6 & MCG_C6_VDIV0_MASK) + 24U);
      MCGOUTClock *= Divider;          /* Calculate the MCG output clock */
    } /* (!((MCG->C6 & MCG_C6_PLLS_MASK) == 0x00U)) */
  } else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x40U) {
    /* Internal reference clock is selected */
    if ((MCG->C2 & MCG_C2_IRCS_MASK) == 0x00U) {
      MCGOUTClock = CPU_INT_SLOW_CLK_HZ; /* Slow internal reference clock selected */
    } else { /* (!((MCG->C2 & MCG_C2_IRCS_MASK) == 0x00U)) */
      Divider = (uint16_t)(0x01LU << ((MCG->SC & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT));
      MCGOUTClock = (uint32_t) (CPU_INT_FAST_CLK_HZ / Divider); /* Fast internal reference clock selected */
    } /* (!((MCG->C2 & MCG_C2_IRCS_MASK) == 0x00U)) */
  } else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80U) {
    /* External reference clock is selected */
    MCGOUTClock = CPU_XTAL_CLK_HZ;     /* System oscillator drives MCG clock */
  } else { /* (!((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80U)) */
    /* Reserved value */
    return;
  } /* (!((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80U)) */
  SystemCoreClock = (MCGOUTClock / (0x01U + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT)));

}
