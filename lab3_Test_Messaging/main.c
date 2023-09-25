
//*****************************************************************************
//
// Application Name     - int_sw
// Application Overview - The objective of this application is to demonstrate
//                          GPIO interrupts using SW2 and SW3.
//                          NOTE: the switches are not debounced!
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
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

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
#include "uart.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "spi.h"

// Common interface includes
#include "uart_if.h"
#include "gpio.h"
#include "test.h"

#include "pin_mux_config.h"

#define APPLICATION_VERSION     "1.1.1"

#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
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

// systick reload value set to 80ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 6400000UL

// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 1;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
volatile unsigned long pin_intcount = 0;
volatile unsigned long pin_intflag = 0;
// read the countdown register and compute elapsed cycles
uint64_t delta;
int valid_signal = 0;

// convert elapsed cycles to microseconds
uint64_t delta_us;
uint32_t decoded_value = 0;

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

char *preset_colors[5] = {"MAGENTA", "GREEN", "RED", "YELLOW", "BLUE"};

int num = 0;
int msgCount = 0;
int colorCount = 0;

char letter = ' ';
char newLetter = ' ';
char msg[30] = "";

void PrintMsg(void) {
    int i;
    int j = 0;
    int count = 0;

    for (i = 0; i < sizeof(msg); i++) {
        if (i >= 127) {
            i = 0;
            j += 7;
        }
        drawChar(i*7, j, (unsigned char)msg[count], BLACK, MAGENTA, 1);
        count++;
    }
}

void GetLetter (char c, int n) {
    letter = newLetter;

    switch(n) {
        case 0:
            newLetter = ' ';
            break;
        case 2:
            if (c == 'a') {newLetter = 'b';}
            else if (c == 'b') {newLetter = 'c';}
            else newLetter = 'a';
            break;
        case 3:
            if (c == 'd') {newLetter = 'e';}
            else if (c == 'e') {newLetter = 'f';}
            else newLetter = 'd';
            break;
        case 4:
            if (c == 'g') {newLetter = 'h';}
            else if (c == 'h') {newLetter = 'i';}
            else newLetter = 'g';
            break;
        case 5:
            if (c == 'j') {newLetter = 'k';}
            else if (c == 'k') {newLetter = 'l';}
            else newLetter = 'j';
            break;
        case 6:
            if (c == 'm') {newLetter = 'n';}
            else if (c == 'n') {newLetter = 'o';}
            else newLetter = 'm';
            break;
        case 7:
            if (c == 'p') {newLetter = 'q';}
            else if (c == 'q') {newLetter = 'r';}
            else if (c == 'r') {newLetter = 's';}
            else newLetter = 'p';
            break;
        case 8:
            if (c == 't') {newLetter = 'u';}
            else if (c == 'u') {newLetter = 'v';}
            else newLetter = 't';
            break;
        case 9:
            if (c == 'w') {newLetter = 'x';}
            else if (c == 'x') {newLetter = 'y';}
            else if (c == 'y') {newLetter = 'z';}
            else newLetter = 'w';
            break;
    }
}

void DeleteLetter(void) {
    if (msgCount == 0) {
        msgCount = msgCount;
    }
    else {
        msg[msgCount - 1] = '\0';   // set value to NULL
        msgCount--;
    }
}

void ColorFontChange(void) {
    if (colorCount == 4) {
        colorCount = 0;
    }
    fillScreen(MAGENTA);
    char message[50] = "Font color: ";
    Outstr(strcat(message, preset_colors[colorCount]));
    colorCount++;
}

void StoreLetter(char c) {
    msg[msgCount] = c;
    msgCount++;
}

/* **************** FIXME ****************** */
/* **************** FIXME ****************** */
/* **************** FIXME ****************** */

void SendMsg(void) {
    msgCount = 0;   // reset count
    int i = 0;
    for (i = 0; i < sizeof(msg); i++) {
        //while(UARTBusy(UARTA1_BASE)){;}
        UARTCharPut(UARTA1_BASE, (unsigned char)msg[i]);
        UtilsDelay(500);
    }
    // Terminate string with NULL value
    UARTCharPut(UARTA1_BASE,'\0');
    UtilsDelay(500);

    PrintMsg();

    msg[0] = '\0';
}

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
    systick_cnt = 1;
}

void Decode() {
    if(delta_us >= 13400 && delta_us <= 13700){
        valid_signal = 1;
    }

    if (valid_signal) {
        if (delta_us >= 1100 && delta_us <= 1250){
            decoded_value = decoded_value << 1;
            pin_intcount++;
        } else if (delta_us >= 2100 && delta_us <= 2400){
            decoded_value = decoded_value << 1;
            decoded_value = decoded_value + 1;
            pin_intcount++;
        } else {
            decoded_value = decoded_value;
        }
    }
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

    ulStatus = MAP_GPIOIntStatus(GPIOA1_BASE, true);
    MAP_GPIOIntClear(GPIOA1_BASE, ulStatus);        // clear interrupts on GPIOA1

//    pin_intflag=1;
    // read the countdown register and compute elapsed cycles
    delta = (SYSTICK_RELOAD_VAL * systick_cnt) - SysTickValueGet();
    // convert elapsed cycles to microseconds
    delta_us = TICKS_TO_US(delta); // Convert to milliseconds

    Decode();
    // reset the countdown register
    SysTickReset();
}

static void UARTHandler(void){
    unsigned long uartStatus;

    uartStatus = UARTIntStatus(UARTA1_BASE, true);
    UARTIntClear(UARTA1_BASE, uartStatus);

    while (UARTCharsAvail(UARTA1_BASE)){
        char c = UARTCharGetNonBlocking(UARTA1_BASE);
        StoreLetter(c);
    }
    PrintMsg();
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

void MasterMain()
{

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);
}

void ConfigureUART(){
    //MAP_UARTConfigSetExpClk(UARTA1_BASE,
    //                        SYSCLKFREQ,
    //                        115200,
    //                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTConfigSetExpClk(UARTA1_BASE,PRCMPeripheralClockGet(PRCM_UARTA1),
                      UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));

    UARTFIFODisable(UARTA1_BASE);
   // UARTDMADisable(UARTA1_BASE, (UART_DMA_RX | UART_DMA_TX));
    UARTIntRegister(UARTA1_BASE, UARTHandler);
    UARTFIFOLevelSet(UARTA1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    unsigned long uartStatus;
    uartStatus = UARTIntStatus(UARTA1_BASE, false);
    UARTIntClear(UARTA1_BASE, uartStatus);
    UARTIntEnable(UARTA1_BASE, UART_INT_RX);
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
//  unsigned long ulStatus;

    BoardInit();
    PinMuxConfig();
    // Enable SysTick
    SysTickInit();
    InitTerm();
    ClearTerm();

    MasterMain();

    // Initialize OLED screen
    Adafruit_Init();
    fillScreen(MAGENTA);

    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(GPIOA1_BASE, GPIOA1IntHandler);

    //
    // Configure falling edge interrupts on pin 64
    //
    MAP_GPIOIntTypeSet(GPIOA1_BASE, 0x2, GPIO_FALLING_EDGE);    // GPIO Pin

    // Enable interrupt
    MAP_GPIOIntEnable(GPIOA1_BASE, 0x2);

    ConfigureUART();
    while (1) {
        while (pin_intcount!=32) {;}
        pin_intcount = 0;
        valid_signal = 0;
        switch (decoded_value){
            // only assigning num to buttons that have letters
            case 554107135ULL:
                printf("button 0 was pressed\n");
                num = 0;
                GetLetter(newLetter, num);
                PrintMsg();
                break;
            case 554139775ULL:
                printf("button 1 was pressed\n");
//                ColorFontChange();
                break;
            case 554123455ULL:
                printf("button 2 was pressed\n");
                num = 2;
                // if we are already at the first letter of a number, we then want the next letter
                GetLetter(newLetter, num);
                break;
            case 554156095ULL:
                printf("button 3 was pressed\n");
                num = 3;
                GetLetter(newLetter, num);
                break;
            case 554115295ULL:
                printf("button 4 was pressed\n");
                num = 4;
                GetLetter(newLetter, num);
                break;
            case 554147935ULL:
                printf("button 5 was pressed\n");
                num = 5;
                GetLetter(newLetter, num);
                break;
            case 554131615ULL:
                printf("button 6 was pressed\n");
                num = 6;
                GetLetter(newLetter, num);
                break;
            case 554164255ULL:
                printf("button 7 was pressed\n");
                num = 7;
                GetLetter(newLetter, num);
                break;
            case 554111215ULL:
                printf("button 8 was pressed\n");
                num = 8;
                GetLetter(newLetter, num);
                break;
            case 554143855ULL:
                printf("button 9 was pressed\n");
                num = 9;
                GetLetter(newLetter, num);
                break;
            case 554166805ULL:
                printf("button LAST was pressed\n");
                DeleteLetter();
                PrintMsg();
                break;
            case 554116825ULL:
                printf("button MUTE was pressed\n");
                SendMsg();
                break;
            case 554109685ULL:
                printf("button + was pressed\n");
                StoreLetter(newLetter);
                PrintMsg();
                break;
        }
        printf("old letter: %c\tnew letter: %c\n", letter, newLetter);
        printf("string: %s\n", msg);
        decoded_value = 0;
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
