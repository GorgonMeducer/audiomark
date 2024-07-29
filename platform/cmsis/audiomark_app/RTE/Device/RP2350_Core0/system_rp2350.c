/*---------------------------------------------------------------------------
 * Name:    system_rp2350.c
 * Purpose: CMSIS-Core System File for Raspberry Pi RP2350
 *---------------------------------------------------------------------------*/
/*
 * Copyright (c) 2024 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rp2350.h"

#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3U)
   #include "partition_rp2350.h"
#endif

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define  XTAL            (12000000UL)     /* Oscillator frequency */

#define  SYSTEM_CLOCK    (XTAL / 2U)

#include <RTE_Components.h>
#undef __USE_EVENT_RECORDER__
#if defined(RTE_Compiler_EventRecorder) || defined(RTE_CMSIS_View_EventRecorder)
#   define __USE_EVENT_RECORDER__  1
#endif


#if __USE_EVENT_RECORDER__
#   include <EventRecorder.h>
#   include "EventRecorderConf.h"
#endif

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
extern const VECTOR_TABLE_Type __VECTOR_TABLE[];


/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = SYSTEM_CLOCK;  /* System Core Clock Frequency */



/*----------------------------------------------------------------------------
  System Core Clock update function
 *----------------------------------------------------------------------------*/

#if !defined(XSPI_2)
__attribute__((constructor(101)))
#endif
void clock_init (void)
{
    // Disable resus that may be enabled from previous software
    CLOCKS->CLK_SYS_RESUS_CTRL = 0;

    // Enable the xosc
    XOSC->STATUS = 1u << 24;
    XOSC->CTRL = (0xfab << 12) | (0xaa0);
    while ((XOSC->STATUS & (1u << 31)) == 0) __NOP();

    // Before we touch PLLs, switch sys and ref cleanly away from their aux sources.
    CLOCKS->CLK_SYS_CTRL &= ~0x00000001;    // xxxxxxx0 = clk_ref
    while (CLOCKS->CLK_SYS_SELECTED != 1) __NOP();
    CLOCKS->CLK_REF_CTRL &= ~0x00000003;
    while (CLOCKS->CLK_REF_SELECTED != 1) __NOP();

#ifdef USB_PLL
    // PLL USB: 12 / 1 = 12MHz * 100 = 1200MHz / 5 / 5 = 48MHz
    // reset PLL_USB
    RESETS->RESET |=  (1u << 15);
    RESETS->RESET &= ~(1u << 15);
    while ((RESETS->RESET_DONE & (1u << 15)) == 0) __NOP();
    // Load VCO-related dividers before starting VCO
    PLL_USB->CS = (1u << 30) | 1u;
    PLL_USB->FBDIV_INT = 100u;
    // Turn on PLL
    PLL_USB->PWR &= ~((1u << 5) | 1u);    // clear PD, VCOPD bits
    // Wait for PLL to lock
    while ((PLL_USB->CS & (1u << 31)) == 0) __NOP();
    // Set up post dividers
    PLL_USB->PRIM = (5u << 16) | (5u << 12);
    // Turn on post divider
    PLL_USB->PWR &= ~(1u << 3);
#else
    // // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHz / 6 / 2 = 125MHz
    // reset PLL_SYS
    RESETS->RESET |=  (1u << 14);
    RESETS->RESET &= ~(1u << 14);
    while ((RESETS->RESET_DONE & (1u << 14)) == 0) __NOP();
    // Load VCO-related dividers before starting VCO
    PLL_SYS->CS = (1u << 30) | 1u;
    PLL_SYS->FBDIV_INT = 125u;
    // Turn on PLL
    PLL_SYS->PWR &= ~((1u << 5) | 1u);    // clear PD, VCOPD bits
    // Wait for PLL to lock
    while ((PLL_SYS->CS & (1u << 31)) == 0) __NOP();
    // Set up post dividers
    PLL_SYS->PRIM = (6u << 16) | (1u << 12);
    // Turn on post divider
    PLL_SYS->PWR &= ~(1u << 3);

#endif 
    // CLK_REF is the XOSC source
    CLOCKS->CLK_REF_CTRL  = 0x2;            // 2 (= xosc_clksrc)
    while ((CLOCKS->CLK_REF_SELECTED & 15u) != 0x4) __NOP();      //? compare with 2 does not work

    // CLK SYS = CLK_REF
    CLOCKS->CLK_SYS_CTRL = 0x0;              // 0x0 (= clk_ref)
    while ((CLOCKS->CLK_SYS_SELECTED & 3u) != 1) __NOP();      //? compare with 0 does not work

#ifdef USB_PLL
    // CLK SYS = PLL USB (usually)  = 48MHz
    CLOCKS->CLK_SYS_CTRL  = (1u << 5);    // 1 (= clksrc_pll_usb) << 5 | clk_ref
    CLOCKS->CLK_SYS_CTRL |= 1u;           // 1 (= clksrc_pll_usb) << 5 | clksrc_clk_sys_aux
    while ((CLOCKS->CLK_SYS_SELECTED & 3u) != 2) __NOP();      //? compare with 1 does not work
#else
    // CLK SYS = PLL SYS = 125MHz
    CLOCKS->CLK_SYS_CTRL  = (0u << 5);    // 1 (= clksrc_pll_sys) << 5 | clk_ref
    CLOCKS->CLK_SYS_CTRL |= 1u;           // 1 (= clksrc_pll_sys) << 5 | clksrc_clk_sys_aux
    while ((CLOCKS->CLK_SYS_SELECTED & 3u) != 2) __NOP();      //? compare with 1 does not work
#endif
    // CLK PERI is CLK_SYS
    CLOCKS->CLK_PERI_CTRL  = (1u << 11);  // enable | clk_sys

#ifdef USB_PLL
    SystemCoreClock =  48000000u;
#else
    //SystemCoreClock = 125000000u;
    SystemCoreClock = 250000000ul;
#endif

#if __USE_EVENT_RECORDER__
    EventRecorderInitialize(0, 1);
#endif
}

__WEAK
void SystemCoreClockUpdate (void)
{

}



#if !__USE_EVENT_RECORDER__
void stdout_init (void)
{
    IO_BANK0->GPIO0_CTRL = 0x00000002; // UART0_TX
    IO_BANK0->GPIO1_CTRL = 0x00000002; // UART0_RX
    PADS_BANK0->GPIO0 = 0x09;          // Pull up enable, fast
    PADS_BANK0->GPIO1 = 0x49;          // Input enable, Pull up enable, fast
    //CLOCKS->CLK_GPOUT0_CTRL = (1 << 28) | (1 << 11) | (0xC << 5); // Enable, peripheral clock

    // Deassert Reset
    RESETS->RESET |=  (1u << 26);
    RESETS->RESET &= ~(1u << 26);
    while ((RESETS->RESET_DONE & (1u << 26)) == 0) __NOP();

    #ifdef USB_PLL
    UART0->UARTIBRD  =26;              // ibrd : 48MHz/115200/16 = 26
    UART0->UARTFBRD  =0;               // fbrd : 48MHz/115200 - 16*ibrd = 0
    #else
    UART0->UARTIBRD  =67;              // ibrd : 125MHz/115200/16 = 67
    UART0->UARTFBRD  =13;              // fbrd : 125MHz/115200 - 16*ibrd = 13
    #endif
    UART0->UARTLCR_H =0x60;            // Line control : 8N1
    UART0->UARTCR    =0x301;           // cr : Enable TX and RX, UART enable
    UART0->UARTRSR   =0xA;             // Clear buffer overrun if any

    return;
}

unsigned char uart_putc (unsigned char ch)
{
    while (UART0->UARTFR & 0x20);      // Wait if Transmit Holding register is full
    UART0->UARTDR = ch;                // write to transmit holding register

    return ch;
}


int stdout_putchar (int ch)
{
    uart_putc((unsigned char)ch);

    return ch;
}
#endif

/*----------------------------------------------------------------------------
  System initialization function
 *----------------------------------------------------------------------------*/
void SystemInit (void)
{

#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
    SCB->VTOR = (uint32_t) &(__VECTOR_TABLE[0]);
#endif

#if defined (__FPU_USED) && (__FPU_USED == 1U)
    SCB->CPACR |= ((3U << 10U*2U) |           /* enable CP10 Full Access */
                  (3U << 11U*2U)  );         /* enable CP11 Full Access */
#endif

#ifdef UNALIGNED_SUPPORT_DISABLE
    SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    TZ_SAU_Setup();
#endif

}
