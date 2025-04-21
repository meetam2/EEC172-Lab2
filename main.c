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
//   MOSI (SI)  P7 on P1 header
//   SCK (CL)   P5 on P1 header
//   DC         P62 on P1 header !!!!CHANGED FROM P2
//   RESET (R)  P18 on P2 header
//   OLEDCS (OC) P4 on P1 header
//   SDCS (SC)  n.c. (no connection)
//   MISO (SO)  n.c. (no connection)
//   CD         n.c. (no connection)
//   3V         n.c. (no connection)
//   Vin (+)    3.3V
//   GND (G)    GND
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
int DELAY = 800000;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************

void testChar(){
    int i = 0;
    for(i = 0; i <= 255; i++){
        fillScreen(BLACK);
        drawChar(rand()%10, rand()%10, i, RED, BLACK, 3);
        MAP_UtilsDelay(DELAY);
    }

    return;
}

//****************************************************************************
//
//! Parses the readreg command parameters and invokes the I2C APIs
//!
//! \param ucDevAddr is the 7-bit I2C slave address
//! \param pucData is the pointer to the read data to be placed
//! \param ucRegOffset is the register address in the i2c device
//! \param ucLen is the length of data to be read
//!     To read the new data flag and the acceleration data for the x, y and z axes, use parameters (0x18, buffer, 0x2, 6)
//!     To read the acceleration data for the x, y and z axes, respectively, use parameters (0x18, buffer, 3, 1) (0x18, buffer, 5, 1) (0x18, buffer, 7, 1)
//!
//! This function
//!    1. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
readReg(unsigned char ucDevAddr, unsigned char *pucData, unsigned char ucRegOffset, unsigned char ucRdLen)
{
    //unsigned char aucRdDataBuf[256];
    //
    // Write the register address to be read from.
    // Stop bit implicitly assumed to be 0.
    //
    RET_IF_ERR(I2C_IF_Write(ucDevAddr, &ucRegOffset,1,0));

    //
    // Read the specified length of data
    //
    RET_IF_ERR(I2C_IF_Read(ucDevAddr, &pucData[0], ucRdLen));

    //  Uncomment to print read contents
//
//    int bufferIndex = 0;
//    printf("READ CONTENTS (0x): ");
//    while(bufferIndex < ucRdLen){
//        printf("%x, ", pucData[bufferIndex]);
//        bufferIndex++;
//    }
//    printf("\n");


    return SUCCESS;
}
//int
//ProcessReadRegCommand(unsigned char ucDevAddr, unsigned char ucRegOffset1, unsigned char ucRdLen)
//{
//    unsigned char aucRdDataBuf[256];
//    unsigned char ucRegOffset = ucRegOffset1;
//    //
//    // Write the register address to be read from.
//    // Stop bit implicitly assumed to be 0.
//    //
//    RET_IF_ERR(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0));
//    //I2C_IF_Write(ucDevAddr, &ucRegOffset,1,0);
//
//    //
//    // Read the specified length of data
//    //
//    RET_IF_ERR(I2C_IF_Read(ucDevAddr, &aucRdDataBuf[0], ucRdLen));
//    //I2C_IF_Read(ucDevAddr, &aucRdDataBuf[0], ucRdLen);
//
//    int bufferIndex = 0;
//    printf("READ CONTENTS (0x): ");
//    while(bufferIndex < ucRdLen){
//        printf("%x, ", aucRdDataBuf[bufferIndex]);
//        bufferIndex++;
//    }
//    printf("\n");
//    return SUCCESS;
//}

//****************************************************************************
//
//! Reads BMA222 accelerometer for the acceleration data for the x, y and z axes, respectively
//!
//! \param pucData is the pointer to array[3] where data will be placed
//!
//! This function
//!    1. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int getAcc(unsigned char *pucData){
    unsigned char buffer[6];
    RET_IF_ERR(readReg(0x18, &buffer[0], 2, 6));
    pucData[0] = buffer[1];
    pucData[1] = buffer[3];
    pucData[2] = buffer[5];
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
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    //
    // Initialize Adafruit OLED
    //
    Adafruit_Init();


    //*******************temporary delete later
    fillScreen(RED);  // Red screen
    printf("begin\n");


    MAP_UtilsDelay(DELAY);
    printf("AAHHHH\n");
    //ProcessReadRegCommand(24, &buffer[0], 2, 6);
    while(1){
        //ProcessReadRegCommand(0x18, 0x2, 6);
        //readReg(0x18, &buffer[0], 2, 6);
        unsigned char acc[3];
        getAcc(&acc[0]);
        int i = 0;
        for(i = 0; i < 3; i++){
            printf("%d, ", acc[i]);
        }
        printf("\n");


        //MAP_UtilsDelay(DELAY);
    }
    //*******************
}
