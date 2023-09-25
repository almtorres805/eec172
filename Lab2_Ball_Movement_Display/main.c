//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - SPI Demo
// Names: Alejandro Torres and Allen Benjamin
// Application Overview - The demo application focuses on showing the required 
//                        initialization sequence to enable the CC3200 SPI 
//                        module in full duplex 4-wire master and slave mode(s).
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_SPI_Demo
// or
// docs\examples\CC32xx_SPI_Demo.pdf
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <string.h>
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"

// Common interface includes
#include <math.h>
#include "uart_if.h"
#include "i2c_if.h"
#include "pin_mux_config.h"
#include "test.h"

#define APPLICATION_VERSION     "1.1.1"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
int r = 4;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void MasterMain()
{


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

void SmallBall() {
    fillScreen(0);
    // Initial position
    int x = 63;
    int y = 63;
    fillCircle(x,y,r,0x07FF);

    // reads data from the Accelerometer
    int xRetVal, yRetVal, tilt_y, tilt_x;
    unsigned char dev_addr = 0x18;
    unsigned char x_reg_addr = 0x03;
    unsigned char y_reg_addr = 0x05;
    unsigned char ucRdLen = 1;
    unsigned char x_RdDataBuf;
    unsigned char y_RdDataBuf;
    while (1) {
        fillCircle(x,y,r,0x0000);
        // Write the register address to be read from acc_x
        I2C_IF_Write(dev_addr,&x_reg_addr,1,0);
        // Read the specified length of data
        xRetVal = I2C_IF_Read(dev_addr, &x_RdDataBuf, ucRdLen);
        if (xRetVal != 0){
            fprintf(stderr, "Error reading acc_x value: %d", xRetVal);
        }

        // Write the register address to be read from acc_y
        I2C_IF_Write(dev_addr,&y_reg_addr,1,0);
        // Read the specified length of data
        yRetVal = I2C_IF_Read(dev_addr, &y_RdDataBuf, ucRdLen);
        if (yRetVal != 0){
            fprintf(stderr, "Error reading acc_y value: %d", yRetVal);
        }

        tilt_x += x_RdDataBuf;
        tilt_y += y_RdDataBuf;
        if (tilt_x>63){
            tilt_x = x_RdDataBuf - 256;
        }
        if (y>63){
            tilt_y = y_RdDataBuf - 256;
        }
        fillCircle(tilt_x,tilt_y,r,0x07FF);
        fillCircle(tilt_y,tilt_y,r,0x0000);
        printf("x: %d, y: %d\n", x, y);
    }
}

void DrawPatterns() {

    fillScreen(0);
    // Print out txt file
    int x, y;
    int i = 0;
    for(y = 0; y <= 128 - 7; y += 7)
    {
        if (i == 255)
            break;
        for(x = 0; x <= 128 - 6; x += 6)
        {
            drawChar(x, y, i, 0x07E0, 0x00, 1);
            ++i;
        }
    }
    delay(500);
    fillScreen(0);
    Outstr("Hello World!");
    delay(500);

    // Display pattern horizontally and vertically
    lcdTestPattern2();
    delay(500);
    lcdTestPattern();
    delay(500);

    // Rest of test functions
    testlines(0xF800);
    delay(500);
    testfastlines(0x07FF, 0xF800);
    delay(500);
    testdrawrects(0xF800);
    delay(500);
    testfillrects(0x07FF, 0xF800);
    delay(500);
    testfillcircles(3, 0x07FF);
    delay(500);
    testroundrects();
    delay(500);
    testtriangles();
    delay(500);
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

//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    printf("In main");
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    I2C_IF_Open(I2C_MASTER_MODE_FST);

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    MasterMain();

    // Initialize OLED screen
    Adafruit_Init();

//    DrawPatterns();
    SmallBall();

//    while(1)
//    {
//
//    }

}

