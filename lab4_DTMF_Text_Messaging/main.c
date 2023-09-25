// Standard includes
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
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
#include "timer.h"
#include "spi.h"

// Common interface includes
#include "uart_if.h"
#include "gpio.h"
#include "test.h"
#include "timer_if.h"
#include "i2c_if.h"
#include "gpio_if.h"

#include "pin_mux_config.h"

//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION     "1.1.1"
#define MASTER_MODE      1
#define FOREVER          1
#define SPI_IF_BIT_RATE  400000
#define TR_BUFF_SIZE     100

#define SYSCLKFREQ 80000000ULL // the cc3200's fixed clock frequency of 80 MHz

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 80ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 6400000UL
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
//*****************************************************************************
//                      Global Variables for DTMF
//*****************************************************************************
volatile int N = 410;
volatile int count;
volatile int samples[410];
volatile bool flag;         // flag set when the samples buffer is full with N samples
volatile bool isFull;       // Each time the handler is called
volatile bool new_digit;      // flag set when inter-digit interval (pause) is detected
long int power_all[7];       // array to store calculated power of 8 frequencies
long int coeff[7] = {31548, 31281, 30951, 30556, 29144, 28361, 27409};
unsigned long sample_freq = 5000ULL;

//*****************************************************************************
//                      Global Variables for OLED
//*****************************************************************************
// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

char *preset_colors[5] = {"BLUE", "GREEN", "RED", "YELLOW", "MAGENTA"};

int msgCount = 0;
int colorCount = 0;
int button;
int newButton;

int x = 0;
int y = 0;
int tempx;
int tempy;

unsigned char letter = ' ';
unsigned char newLetter = ' ';
unsigned char msg[30];

bool shouldSend = false;
//*****************************************************************************
//                      Global Variables for Vector Table
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//
// Globals used by the timer interrupt handler.
//
//*****************************************************************************
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************
//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      HANDLERS
//*****************************************************************************
unsigned short GetADC(){
    unsigned char byte1;
    unsigned char byte2;

    // Set CS gpio pin to low
    GPIOPinWrite(GPIOA1_BASE, 0x10, 0x0);
    MAP_SPICSEnable(GSPI_BASE);

    MAP_SPITransfer(GSPI_BASE, 0, &byte1, 1, SPI_CS_ENABLE | SPI_CS_DISABLE);
    MAP_SPITransfer(GSPI_BASE, 0, &byte2, 1,  SPI_CS_ENABLE |SPI_CS_DISABLE);

    // Convert big endian data to small endian
    unsigned short data = ((0x1f & byte1) << 5) | ((0xf8 & byte2) >> 3);

    // Set CS gpio pin to high
    GPIOPinWrite(GPIOA1_BASE, 0x10, 0x10);
    MAP_SPICSDisable(GSPI_BASE);

    return data;
}

// generate interrupts at 16 kHz to periodically sample the ADC.
static void CollectSamplesHandler(void){
    //
    // Clear the timer interrupt.
    //
    unsigned long ulStatus = MAP_TimerIntStatus(TIMERA0_BASE, false);
    MAP_TimerIntClear(TIMERA0_BASE, ulStatus);

    count++;
    if (count == N){
        isFull = true; // Batch is full
    } else {
        samples[count] = ((unsigned short) GetADC());
    }
}

void PrintMsg(void) {
    if (x >= 127) {
        x = 0;
        y += 7;
    }

    // overwrite the previous letter
    if (button == newButton){
        printf("same button\n");
        drawChar(tempx, tempy, newLetter, RED, BLACK, 1);
    }

    else {
        printf("different button\n");
        x += 7;
        drawChar(x, y, newLetter, RED, BLACK, 1);
//        x += 7;
    }
    tempx = x;
    tempy = y;
}


void RecievedMsg(void) {
    int i;
    int j = 63;
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

static void UARTHandler(void){
    unsigned long uartStatus;

    uartStatus = UARTIntStatus(UARTA1_BASE, true);
    UARTIntClear(UARTA1_BASE, uartStatus);

    while (UARTCharsAvail(UARTA1_BASE)){
        char c = UARTCharGetNonBlocking(UARTA1_BASE);
    }
    RecievedMsg();
}

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

void StoreLetter(unsigned char c){
    msg[msgCount] = c;
    msgCount++;
}

void SendMsg(void) {
    int i = 0;
    for (i = 0; i < msgCount; i++) {
        UARTCharPut(UARTA1_BASE, msg[i]);
        UtilsDelay(1000);
    }
    fillScreen(BLACK);
    msgCount = 0;   // reset count
    x = 0;
    y = 0;
    button = 1;
    // Terminate string with NULL value
    UARTCharPut(UARTA1_BASE, '\0');
    UtilsDelay(1000);
}

void DeleteLetter(){
    // Delete from OLED
    drawChar(tempx, tempy, newLetter, BLACK, BLACK, 1);

    // Delete from string
    if(msgCount == 0){
        msgCount = msgCount;
    } else{
        msg[msgCount - 1] = '\0';
        msgCount--;
    }
}

void GetLetter (char c) {
    letter = newLetter;
    button = newButton;

    switch(c) {
        case '0':
            newButton = 0;
            newLetter = ' ';
            printf("button 0\n");
            break;
        case '2':
            newButton = 2;
            if (newLetter == 'a') {newLetter = 'b';}
            else if (newLetter == 'b') {newLetter = 'c';}
            else newLetter = 'a';
            printf("button 2\n");
            break;
        case '3':
            newButton = 3;
            if (newLetter == 'd') {newLetter = 'e';}
            else if (newLetter == 'e') {newLetter = 'f';}
            else newLetter = 'd';
            printf("button 3\n");
            break;
        case '4':
            newButton = 4;
            if (newLetter == 'g') {newLetter = 'h';}
            else if (newLetter == 'h') {newLetter = 'i';}
            else newLetter = 'g';
            printf("button 4\n");
            break;
        case '5':
            newButton = 5;
            if (newLetter == 'j') {newLetter = 'k';}
            else if (newLetter == 'k') {newLetter = 'l';}
            else newLetter = 'j';
            printf("button 5\n");
            break;
        case '6':
            newButton = 6;
            if (newLetter == 'm') {newLetter = 'n';}
            else if (newLetter == 'n') {newLetter = 'o';}
            else newLetter = 'm';
            printf("button 6\n");
            break;
        case '7':
            newButton = 7;
            if (newLetter == 'p') {newLetter = 'q';}
            else if (newLetter == 'q') {newLetter = 'r';}
            else if (newLetter == 'r') {newLetter = 's';}
            else newLetter = 'p';
            printf("button 7\n");
            break;
        case '8':
            newButton = 8;
            if (newLetter == 't') {newLetter = 'u';}
            else if (newLetter == 'u') {newLetter = 'v';}
            else newLetter = 't';
            printf("button 8\n");
            break;
        case '9':
            newButton = 9;
            if (newLetter == 'w') {newLetter = 'x';}
            else if (newLetter == 'x') {newLetter = 'y';}
            else if (newLetter == 'y') {newLetter = 'z';}
            else newLetter = 'w';
            printf("button 9\n");
            break;
    }
    StoreLetter(newLetter);
    PrintMsg();
}

unsigned short DCBias() {
    int j;
    unsigned short avg = 0;
    for (j = 0; j<N; j++){
        avg += samples[j];
    }
    avg = avg/N;
    return avg;
}

long int goertzel(long int coeff)
{
    //initialize variables to be used in the function
    int Q, Q_prev, Q_prev2,i;
    long prod1, prod2, prod3, power;

    Q_prev = 0;         //set delay element1 Q_prev as zero
    Q_prev2 = 0;        //set delay element2 Q_prev2 as zero
    power=0;            //set power as zero

    unsigned short adc_avg = DCBias();

    for (i = 0; i < N; i++) // loop SAMPLE_SPACE times and calculate Q, Q_prev, Q_prev2 at each iteration
    {
        Q = (samples[i] - adc_avg) + ((coeff* Q_prev)>>14) - (Q_prev2); // >>14 used as the coeff was used in Q15 format
        Q_prev2 = Q_prev;                                    // shuffle delay elements
        Q_prev = Q;
    }

    //calculate the three products used to calculate power
    prod1=((long) Q_prev*Q_prev);
    prod2=((long) Q_prev2*Q_prev2);
    prod3=((long) Q_prev *coeff)>>14;
    prod3=(prod3 * Q_prev2);

    power = ((prod1+prod2-prod3))>>8; //calculate power using the three products and scale the result down

    return power;
}

void post_test()
{
    //initialize variables to be used in the function
    int max_power,i, row, col;

    char row_col[4][4] =      // array with the order of the digits in the DTMF system
    {
      {'1', '2', '3', 'A'},
      {'4', '5', '6', 'B'},
      {'7', '8', '9', 'C'},
      {'*', '0', '#', 'D'}
    };

    // find the maximum power in the row frequencies and the row number
    max_power=0;            //initialize max_power=0
    for(i=0;i<4;i++) {      //loop 4 times from 0>3 (the indecies of the rows)
        if (power_all[i] > max_power) { //if power of the current row frequency > max_power
            max_power=power_all[i];     //set max_power as the current row frequency
            row=i;                      //update row number
        }
    }

    // find the maximum power in the column frequencies and the column number
    max_power=0;            //initialize max_power=0
    for(i=4;i<7;i++) {      //loop 3 times from 4>7 (the indecies of the columns)
        if (power_all[i] > max_power) { //if power of the current column frequency > max_power
            max_power=power_all[i];     //set max_power as the current column frequency
            col=i;                      //update column number
        }
    }

    if(power_all[col] > 60000 && power_all[row] > 60000)
        new_digit = 1;

    if((power_all[col]> 60000 && power_all[row]> 60000) && (new_digit == 1)) {
        new_digit = 0;
        if (row_col[row][col - 4] == '*'){
            DeleteLetter();
        } else if (row_col[row][col - 4] == '#'){
            SendMsg();
        } else{
            GetLetter(row_col[row][col - 4]);
        }
    }
}

void Decode(){
    // Disable Timer
    unsigned long ulStatus = MAP_TimerIntStatus(TIMERA0_BASE, false);
    MAP_TimerIntClear(TIMERA0_BASE, ulStatus);
    MAP_TimerDisable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);

    //reset count
    count = 0;
    size_t i;
    for (i = 0; i < 7; i++)
      power_all[i] = goertzel(coeff[i]);   // call goertzel to calculate the power at each frequency and store it in the power_all array

    post_test();

    isFull = 0;         //reset flag
    // Enable timer again
    MAP_TimerLoadSet(TIMERA0_BASE, TIMER_A, sample_freq);
    MAP_TimerIntClear(TIMERA0_BASE, ulStatus);
    MAP_TimerEnable(TIMERA0_BASE, TIMER_A);
}

//*****************************************************************************
//                      CONFIGURATION FUNCTIONS
//*****************************************************************************
void DTMFTimer() {
    g_ulBase = TIMERA0_BASE;

    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, CollectSamplesHandler);

    MAP_TimerLoadSet(TIMERA0_BASE, TIMER_A, sample_freq);
    unsigned long ulStatus = MAP_TimerIntStatus(TIMERA0_BASE, false);
    MAP_TimerIntClear(TIMERA0_BASE, ulStatus);
    MAP_TimerEnable(TIMERA0_BASE, TIMER_A);
}

void MasterMain()
{
    // Enable the SPI module clock
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    // Reset the peripheral
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    // Reset SPI
    MAP_SPIReset(GSPI_BASE);

    // Configure SPI interface
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    // Enable SPI for communication
    MAP_SPIEnable(GSPI_BASE);
}

void ConfigureUart(){
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

int main(){
    //
    // Initialize board configurations
    BoardInit();
    //
    // Pinmuxing for LEDs
    //
    PinMuxConfig();

    ConfigureUart();

    //I2C_IF_Open(I2C_MASTER_MODE_FST);

    MasterMain();

    Adafruit_Init();
    fillScreen(BLACK);
    //
    // Configure general timer for sampling DTMF
    DTMFTimer();
    //
    // Loop forever while the timer runs.
    //
    while(FOREVER){
        while (isFull == 0);  // wait till N samples are read in the buffer and the flag set by the ADC ISR
        Decode();
    }
}
