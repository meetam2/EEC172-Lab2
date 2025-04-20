//*****************************************************************************
//
// Application Name     - SPI
// Application Overview - The demo application focuses on showing the required 
//                        initialization sequence to enable the CC3200 SPI 
//                        module in full duplex 4-wire master and slave mode(s).
//
//*****************************************************************************

//*****************************************************************************
// Pin Connections
//   MOSI (SI)  P7 on P2 header
//   SCK (CL)   P5 on P1 header
//   DC         P2 on P1 header
//   RESET (R)  P18 on P2 header
//   OLEDCS (OC) P4 on P1 header
//   SDCS (SC)  n.c. (no connection)
//   MISO (SO)  n.c. (no connection)
//   CD         n.c. (no connection)
//   3V         n.c. (no connection)
//*****************************************************************************


// Standard includes
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


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

// Common interface includes
#include "uart_if.h"
#include "pin_mux_config.h"
#include "Adafruit_SSD1351.h"
#include "Adafruit_GFX.h"
#include "glcdfont.h"
#include "oled_test.h"
#include "i2c_if.h"

//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION     "1.4.0"
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
int DELAY = 8000000;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************


//****************************************************************************
//
//! Parses the readreg command parameters and invokes the I2C APIs
//! i2c readreg 0x<dev_addr> 0x<reg_offset> <rdlen>
//!
//! \param pcInpString pointer to the readreg command parameters
//!         dev_addr - slave address of the i2c device
//!         reg_offset - register address in the i2c device
//!
//! This function
//!    1. Parses the readreg command parameters.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ProcessReadRegCommand(unsigned char ucDevAddr, unsigned char ucRegOffset, unsigned char ucRdLen)
{
    printf("BEGIN READREG COMMAND");
    unsigned char aucRdDataBuf[256];

    //RETERR_IF_TRUE(ucLen > sizeof(aucDataBuf));

    //
    // Write the register address to be read from.
    // Stop bit implicitly assumed to be 0.
    //
    RET_IF_ERR(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0));

    //
    // Read the specified length of data
    //
    RET_IF_ERR(I2C_IF_Read(ucDevAddr, &aucRdDataBuf[0], ucRdLen));

    printf("Read contents %x",aucRdDataBuf[0]);
    return SUCCESS;
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
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_SPIReset(GSPI_BASE);

    MAP_SPIConfigSetExpClk(GSPI_BASE, MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                           8000000, SPI_MODE_MASTER, SPI_SUB_MODE_0,
                           (SPI_SW_CTRL_CS |
                            SPI_4PIN_MODE |
                            SPI_TURBO_OFF |
                            SPI_CS_ACTIVELOW |    // most SSD1351 modules use ACTIVELOW
                            SPI_WL_8));

    MAP_SPIEnable(GSPI_BASE);
    //
    // Initialising the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Initialize Adafruit OLED
    //
    Adafruit_Init();

    //temporary drawChar
    fillScreen(RED);  // Red screen
    printf("begin\n");
    ProcessReadRegCommand(0x18, 0x03, 0x01);
    MAP_UtilsDelay(DELAY);
    drawChar(SSD1351WIDTH/2, SSD1351HEIGHT/2, 0x6A, BLACK, RED, SSD1351WIDTH/2);

    int i = 0;
    while(1){
        fillScreen(RED);
        drawChar(rand()%10, rand()%10, i, BLACK, RED, 3);
        MAP_UtilsDelay(DELAY);
        if(i >= 255){
            i = 0;
        } else{
            i++;
        }
        //ProcessReadRegCommand(0x18, 0x3, 1);
    }
    //
}
