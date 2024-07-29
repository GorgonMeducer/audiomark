/*---------------------------------------------------------------------------
 * Name:    startup_rp2350.c
 * Purpose: CMSIS-Core Startup File for Raspberry Pi RP2350
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


/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;
#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
extern uint32_t __STACK_SEAL;
#endif

extern __NO_RETURN void __PROGRAM_START(void);

/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler  (void);
            void Default_Handler(void);

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler            (void) __attribute__ ((weak));
void MemManage_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void SecureFault_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));

void TIMER0_IRQ_0_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER0_IRQ_1_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER0_IRQ_2_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER0_IRQ_3_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER1_IRQ_0_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER1_IRQ_1_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER1_IRQ_2_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER1_IRQ_3_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM_IRQ_WRAP_0_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM_IRQ_WRAP_1_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA_IRQ_0_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA_IRQ_1_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA_IRQ_2_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA_IRQ_3_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void USBCTRL_IRQ_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void PIO0_IRQ_0_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void PIO0_IRQ_1_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void PIO1_IRQ_0_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void PIO1_IRQ_1_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void PIO2_IRQ_0_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void PIO2_IRQ_1_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void IO_IRQ_BANK0_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void IO_IRQ_BANK0_NS_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void IO_IRQ_QSPI_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void IO_IRQ_QSPI_NS_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void SIO_IRQ_FIFO_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void SIO_IRQ_BELL_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void SIO_IRQ_FIFO_NS_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void SIO_IRQ_BELL_NS_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void SIO_IRQ_MTIMECMP_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void CLOCKS_IRQ_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI0_IRQ_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQ_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void UART0_IRQ_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQ_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC_IRQ_FIFO_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQ_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_IRQ_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void OTP_IRQ_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void TRNG_IRQ_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void PROC0_IRQ_CTI_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));
void PROC1_IRQ_CTI_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));
void PLL_SYS_IRQ_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void PLL_USB_IRQ_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void POWMAN_IRQ_POW_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void POWMAN_IRQ_TIMER_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));




/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/

#if defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

extern const VECTOR_TABLE_Type __VECTOR_TABLE[];
       const VECTOR_TABLE_Type __VECTOR_TABLE[] __VECTOR_TABLE_ATTRIBUTE = {
  (VECTOR_TABLE_Type)(&__INITIAL_SP),       /*     Initial Stack Pointer */
  Reset_Handler,                            /*     Reset Handler */
  NMI_Handler,                              /* -14 NMI Handler */
  HardFault_Handler,                        /* -13 Hard Fault Handler */
  MemManage_Handler,                        /* -12 MPU Fault Handler */
  BusFault_Handler,                         /* -11 Bus Fault Handler */
  UsageFault_Handler,                       /* -10 Usage Fault Handler */
  SecureFault_Handler,                      /*  -9 Secure Fault Handler */
  0,                                        /*     Reserved */
  0,                                        /*     Reserved */
  0,                                        /*     Reserved */
  SVC_Handler,                              /*  -5 SVCall Handler */
  DebugMon_Handler,                         /*  -4 Debug Monitor Handler */
  0,                                        /*     Reserved */
  PendSV_Handler,                           /*  -2 PendSV Handler */
  SysTick_Handler,                          /*  -1 SysTick Handler */

  /* Interrupts */
  TIMER0_IRQ_0_Handler,                     /*   0 TIMER0_IRQ_0 */
  TIMER0_IRQ_1_Handler,                     /*   1 TIMER0_IRQ_1 */
  TIMER0_IRQ_2_Handler,                     /*   2 TIMER0_IRQ_2 */
  TIMER0_IRQ_3_Handler,                     /*   3 TIMER0_IRQ_3 */
  TIMER1_IRQ_0_Handler,                     /*   4 TIMER1_IRQ_0 */
  TIMER1_IRQ_1_Handler,                     /*   5 TIMER1_IRQ_1 */
  TIMER1_IRQ_2_Handler,                     /*   6 TIMER1_IRQ_2 */
  TIMER1_IRQ_3_Handler,                     /*   7 TIMER1_IRQ_3 */
  PWM_IRQ_WRAP_0_Handler,                   /*   8 PWM_IRQ_WRAP_0 */
  PWM_IRQ_WRAP_1_Handler,                   /*   9 PWM_IRQ_WRAP_1 */
  DMA_IRQ_0_Handler,                        /*  10 DMA_IRQ_0 */
  DMA_IRQ_1_Handler,                        /*  11 DMA_IRQ_1 */
  DMA_IRQ_2_Handler,                        /*  12 DMA_IRQ_2 */
  DMA_IRQ_3_Handler,                        /*  13 DMA_IRQ_3 */
  USBCTRL_IRQ_Handler,                      /*  14 USBCTRL_IRQ */
  PIO0_IRQ_0_Handler,                       /*  15 PIO0_IRQ_0 */
  PIO0_IRQ_1_Handler,                       /*  16 PIO0_IRQ_1 */
  PIO1_IRQ_0_Handler,                       /*  17 PIO1_IRQ_0 */
  PIO1_IRQ_1_Handler,                       /*  18 PIO1_IRQ_1 */
  PIO2_IRQ_0_Handler,                       /*  19 PIO2_IRQ_0 */
  PIO2_IRQ_1_Handler,                       /*  20 PIO2_IRQ_1 */
  IO_IRQ_BANK0_Handler,                     /*  21 IO_IRQ_BANK0 */
  IO_IRQ_BANK0_NS_Handler,                  /*  22 IO_IRQ_BANK0_NS */
  IO_IRQ_QSPI_Handler,                      /*  23 IO_IRQ_QSPI */
  IO_IRQ_QSPI_NS_Handler,                   /*  24 IO_IRQ_QSPI_NS */
  SIO_IRQ_FIFO_Handler,                     /*  25 SIO_IRQ_FIFO */
  SIO_IRQ_BELL_Handler,                     /*  26 SIO_IRQ_BELL */
  SIO_IRQ_FIFO_NS_Handler,                  /*  27 SIO_IRQ_FIFO_NS */
  SIO_IRQ_BELL_NS_Handler,                  /*  28 SIO_IRQ_BELL_NS */
  SIO_IRQ_MTIMECMP_Handler,                 /*  29 SIO_IRQ_MTIMECMP */
  CLOCKS_IRQ_Handler,                       /*  30 CLOCKS_IRQ */
  SPI0_IRQ_Handler,                         /*  31 SPI0_IRQ */
  SPI1_IRQ_Handler,                         /*  32 SPI1_IRQ */
  UART0_IRQ_Handler,                        /*  33 UART0_IRQ */
  UART1_IRQ_Handler,                        /*  34 UART1_IRQ */
  ADC_IRQ_FIFO_Handler,                     /*  35 ADC_IRQ_FIFO */
  I2C0_IRQ_Handler,                         /*  36 I2C0_IRQ */
  I2C1_IRQ_Handler,                         /*  37 I2C1_IRQ */
  OTP_IRQ_Handler,                          /*  38 OTP_IRQ */
  TRNG_IRQ_Handler,                         /*  39 TRNG_IRQ */
  PROC0_IRQ_CTI_Handler,                    /*  40 PROC0_IRQ_CTI */
  PROC1_IRQ_CTI_Handler,                    /*  41 PROC1_IRQ_CTI  */
  PLL_SYS_IRQ_Handler,                      /*  42 PLL_SYS_IRQ */
  PLL_USB_IRQ_Handler,                      /*  43 PLL_USB_IRQ */
  POWMAN_IRQ_POW_Handler,                   /*  44 POWMAN_IRQ_POW */
  POWMAN_IRQ_TIMER_Handler,                 /*  45 POWMAN_IRQ_TIMER */
  0,                                        /*  46 SPAREIRQ_IRQ_0 */
  0,                                        /*  47 SPAREIRQ_IRQ_1 */
  0,                                        /*  48 SPAREIRQ_IRQ_2 */
  0,                                        /*  49 SPAREIRQ_IRQ_3 */
  0,                                        /*  50 SPAREIRQ_IRQ_4 */
  0                                         /*  51 SPAREIRQ_IRQ_5 */
};

#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
__NO_RETURN __USED
void Reset_Handler(void)
{
  __set_PSP((uint32_t)(&__INITIAL_SP));

  __set_MSPLIM((uint32_t)(&__STACK_LIMIT));
  __set_PSPLIM((uint32_t)(&__STACK_LIMIT));

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
  __TZ_set_STACKSEAL_S((uint32_t *)(&__STACK_SEAL));
#endif

  SystemInit();                             /* CMSIS System Initialization */
  __PROGRAM_START();                        /* Enter PreMain (C library entry point) */
}


#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

/*----------------------------------------------------------------------------
  Hard Fault Handler
 *----------------------------------------------------------------------------*/
void HardFault_Handler(void)
{
  while(1);
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void)
{
  while(1);
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic pop
#endif

