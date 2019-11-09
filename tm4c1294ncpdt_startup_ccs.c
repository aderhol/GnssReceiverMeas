//*****************************************************************************
//
// Startup code for use with TI's Code Composer Studio.
//
// Copyright (c) 2011-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
//*****************************************************************************

#include <stdint.h>

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
//static void NmiSR(void);
//static void FaultISR(void);
//static void IntDefaultHandler(void);

//*****************************************************************************
//
// External declaration for the reset handler that is to be called when the
// processor is started
//
//*****************************************************************************
extern void _c_int00(void);

//*****************************************************************************
//
// Linker variable that marks the top of the stack.
//
//*****************************************************************************
extern uint32_t __STACK_TOP;

//*****************************************************************************
//
// External declarations for the interrupt handlers used by the application.
//
//*****************************************************************************
extern void xPortSysTickHandler(void);
extern void xPortPendSVHandler(void);
extern void vPortSVCHandler(void);

//Debug (main)
extern void ISR_TIMER1_A(void);

//UART0
extern void uart0_ISR(void);

//UART4
extern void uart4_ISR(void);

//UART3
extern void uart3_ISR(void);

//UART6
extern void uart6_ISR(void);

//Sensor HUB
extern void I2C7_ISR(void);

//Skew Measurement
extern void ISR_TIMER0_A(void);
extern void ISR_TIMER2_A(void);
extern void ISR_TIMER2_B(void);
extern void ISR_TIMER3_A(void);
extern void ISR_TIMER3_B(void);

//WDT
extern void ISR_WDT(void);
extern void ISR_WDT_NmiSR(void);
extern void ISR_WDT_HardFault(void);
extern void ISR_WDT_MpuFault(void);
extern void ISR_WDT_BusFault(void);
extern void ISR_WDT_UsageFault(void);
extern void ISR_Default(void);

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000 or at the start of
// the program if located at a start address other than 0.
//
//*****************************************************************************
#pragma DATA_SECTION(g_pfnVectors, ".intvecs")
void (* const g_pfnVectors[])(void) =
{
    (void (*)(void))((uint32_t)&__STACK_TOP),
                                            // The initial stack pointer
    ResetISR,                               // The reset handler
    ISR_WDT_NmiSR,                                  // The NMI handler
    ISR_WDT_HardFault,                       // The hard fault handler
    ISR_WDT_MpuFault,                      // The MPU fault handler
    ISR_WDT_BusFault,                      // The bus fault handler
    ISR_WDT_UsageFault,                      // The usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    vPortSVCHandler,                      // SVCall handler
    ISR_Default,                      // Debug monitor handler
    0,                                      // Reserved
    xPortPendSVHandler,                      // The PendSV handler
    xPortSysTickHandler,                      // The SysTick handler
    ISR_Default,                      // GPIO Port A
    ISR_Default,                      // GPIO Port B
    ISR_Default,                      // GPIO Port C
    ISR_Default,                      // GPIO Port D
    ISR_Default,                      // GPIO Port E
    uart0_ISR,                      // UART0 Rx and Tx
    ISR_Default,                      // UART1 Rx and Tx
    ISR_Default,                      // SSI0 Rx and Tx
    ISR_Default,                      // I2C0 Master and Slave
    ISR_Default,                      // PWM Fault
    ISR_Default,                      // PWM Generator 0
    ISR_Default,                      // PWM Generator 1
    ISR_Default,                      // PWM Generator 2
    ISR_Default,                      // Quadrature Encoder 0
    ISR_Default,                      // ADC Sequence 0
    ISR_Default,                      // ADC Sequence 1
    ISR_Default,                      // ADC Sequence 2
    ISR_Default,                      // ADC Sequence 3
    ISR_WDT,                      // Watchdog timer
    ISR_TIMER0_A,                      // Timer 0 subtimer A
    ISR_Default,                      // Timer 0 subtimer B
    ISR_TIMER1_A,                      // Timer 1 subtimer A
    ISR_Default,                      // Timer 1 subtimer B
    ISR_TIMER2_A,                      // Timer 2 subtimer A
    ISR_TIMER2_B,                      // Timer 2 subtimer B
    ISR_Default,                      // Analog Comparator 0
    ISR_Default,                      // Analog Comparator 1
    ISR_Default,                      // Analog Comparator 2
    ISR_Default,                      // System Control (PLL, OSC, BO)
    ISR_Default,                      // FLASH Control
    ISR_Default,                      // GPIO Port F
    ISR_Default,                      // GPIO Port G
    ISR_Default,                      // GPIO Port H
    ISR_Default,                      // UART2 Rx and Tx
    ISR_Default,                      // SSI1 Rx and Tx
    ISR_TIMER3_A,                      // Timer 3 subtimer A
    ISR_TIMER3_B,                      // Timer 3 subtimer B
    ISR_Default,                      // I2C1 Master and Slave
    ISR_Default,                      // CAN0
    ISR_Default,                      // CAN1
    ISR_Default,                      // Ethernet
    ISR_Default,                      // Hibernate
    ISR_Default,                      // USB0
    ISR_Default,                      // PWM Generator 3
    ISR_Default,                      // uDMA Software Transfer
    ISR_Default,                      // uDMA Error
    ISR_Default,                      // ADC1 Sequence 0
    ISR_Default,                      // ADC1 Sequence 1
    ISR_Default,                      // ADC1 Sequence 2
    ISR_Default,                      // ADC1 Sequence 3
    ISR_Default,                      // External Bus Interface 0
    ISR_Default,                      // GPIO Port J
    ISR_Default,                      // GPIO Port K
    ISR_Default,                      // GPIO Port L
    ISR_Default,                      // SSI2 Rx and Tx
    ISR_Default,                      // SSI3 Rx and Tx
    uart3_ISR,                      // UART3 Rx and Tx
    uart4_ISR,                      // UART4 Rx and Tx
    ISR_Default,                      // UART5 Rx and Tx
    uart6_ISR,                      // UART6 Rx and Tx
    ISR_Default,                      // UART7 Rx and Tx
    ISR_Default,                      // I2C2 Master and Slave
    ISR_Default,                      // I2C3 Master and Slave
    ISR_Default,                      // Timer 4 subtimer A
    ISR_Default,                      // Timer 4 subtimer B
    ISR_Default,                      // Timer 5 subtimer A
    ISR_Default,                      // Timer 5 subtimer B
    ISR_Default,                      // FPU
    0,                                      // Reserved
    0,                                      // Reserved
    ISR_Default,                      // I2C4 Master and Slave
    ISR_Default,                      // I2C5 Master and Slave
    ISR_Default,                      // GPIO Port M
    ISR_Default,                      // GPIO Port N
    0,                                      // Reserved
    ISR_Default,                      // Tamper
    ISR_Default,                      // GPIO Port P (Summary or P0)
    ISR_Default,                      // GPIO Port P1
    ISR_Default,                      // GPIO Port P2
    ISR_Default,                      // GPIO Port P3
    ISR_Default,                      // GPIO Port P4
    ISR_Default,                      // GPIO Port P5
    ISR_Default,                      // GPIO Port P6
    ISR_Default,                      // GPIO Port P7
    ISR_Default,                      // GPIO Port Q (Summary or Q0)
    ISR_Default,                      // GPIO Port Q1
    ISR_Default,                      // GPIO Port Q2
    ISR_Default,                      // GPIO Port Q3
    ISR_Default,                      // GPIO Port Q4
    ISR_Default,                      // GPIO Port Q5
    ISR_Default,                      // GPIO Port Q6
    ISR_Default,                      // GPIO Port Q7
    ISR_Default,                      // GPIO Port R
    ISR_Default,                      // GPIO Port S
    ISR_Default,                      // SHA/MD5 0
    ISR_Default,                      // AES 0
    ISR_Default,                      // DES3DES 0
    ISR_Default,                      // LCD Controller 0
    ISR_Default,                      // Timer 6 subtimer A
    ISR_Default,                      // Timer 6 subtimer B
    ISR_Default,                      // Timer 7 subtimer A
    ISR_Default,                      // Timer 7 subtimer B
    ISR_Default,                      // I2C6 Master and Slave
    I2C7_ISR,                      // I2C7 Master and Slave
    ISR_Default,                      // HIM Scan Matrix Keyboard 0
    ISR_Default,                      // One Wire 0
    ISR_Default,                      // HIM PS/2 0
    ISR_Default,                      // HIM LED Sequencer 0
    ISR_Default,                      // HIM Consumer IR 0
    ISR_Default,                      // I2C8 Master and Slave
    ISR_Default,                      // I2C9 Master and Slave
    ISR_Default,                      // GPIO Port T
    ISR_Default,                      // Fan 1
    0,                                      // Reserved
};

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void
ResetISR(void)
{
    //
    // Jump to the CCS C initialization routine.  This will enable the
    // floating-point unit as well, so that does not need to be done here.
    //
    __asm("    .global _c_int00\n"
          "    b.w     _c_int00");
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void
NmiSR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
FaultISR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
IntDefaultHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}
