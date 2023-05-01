
//*****************************************************************************
//
// Application Name     - int_sw
// Application Overview - The objective of this application is to demonstrate
//							GPIO interrupts using SW2 and SW3.
//							NOTE: the switches are not debounced!
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup int_sw
//! @{
//
//****************************************************************************

// Standard includes
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "hw_nvic.h"
#include "systick.h"

// Common interface includes
#include "uart_if.h"

#include "pin_mux_config.h"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
// some helpful macros for systick

// the cc3200's fixed clock frequency of 80 MHz
// note the use of ULL to indicate an unsigned long long constant
#define SYSCLKFREQ 80000000ULL

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 3200000UL

// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 0;

extern void (* const g_pfnVectors[])(void);
volatile unsigned long pin_intcount = 0;
volatile unsigned long pin_intflag = 0;
// read the countdown register and compute elapsed cycles
int delta;

// convert elapsed cycles to microseconds
int delta_us;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

/**
 * Reset SysTick Counter
 */
static inline void SysTickReset(void) {
    // any write to the ST_CURRENT register clears it
    // after clearing it automatically gets reset without
    // triggering exception logic
    // see reference manual section 3.2.1
    HWREG(NVIC_ST_CURRENT) = 1;

    // clear the global count variable
    systick_cnt = 0;
}

/**
 * SysTick Interrupt Handler
 *
 * Keep track of whether the systick counter wrapped
 */
static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_cnt++;
}

static void GPIOA1IntHandler(void) {
    unsigned long ulStatus;
    // reset the countdown register
    SysTickReset();
    // wait for a fixed number of cycles
    // should be 3000 i think (see utils.c)
    UtilsDelay(1000);

    // read the countdown register and compute elapsed cycles
    uint64_t delta = SYSTICK_RELOAD_VAL - SysTickValueGet();

    // convert elapsed cycles to microseconds
    uint64_t delta_us = TICKS_TO_US(delta);
    ulStatus = MAP_GPIOIntStatus(GPIOA1_BASE, true);
    MAP_GPIOIntClear(GPIOA1_BASE, ulStatus);		// clear interrupts on GPIOA1

    pin_intcount++;
    pin_intflag=1;
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void) {
	MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

/**
 * Initializes SysTick Module
 */
static void SysTickInit(void) {

    // configure the reset value for the systick countdown register
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    // register interrupts on the systick module
    MAP_SysTickIntRegister(SysTickHandler);

    // enable interrupts on systick
    // (trigger SysTickHandler when countdown reaches 0)
    MAP_SysTickIntEnable();

    // enable the systick module itself
    MAP_SysTickEnable();


}
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//!
//! \return None.
//
//****************************************************************************
int main() {
	unsigned long ulStatus;

    BoardInit();
    
    PinMuxConfig();
    
    // Enable SysTick
    SysTickInit();

    InitTerm();

    ClearTerm();

    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(GPIOA1_BASE, GPIOA1IntHandler);

    //
    // Configure falling edge interrupts on SW2 and SW3
    //
    MAP_GPIOIntTypeSet(GPIOA1_BASE, 0x2, GPIO_FALLING_EDGE);	// GPIO Pin

    ulStatus = MAP_GPIOIntStatus (GPIOA1_BASE, false);
    MAP_GPIOIntClear(GPIOA1_BASE, ulStatus);			// clear interrupts on GPIOA1

    // Enable interrupt
    MAP_GPIOIntEnable(GPIOA1_BASE, 0x2);
    while (1) {
        printf("interrupt count: %d, %d\n", pin_intcount, pin_intflag);

    }

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
