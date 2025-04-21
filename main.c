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
//   DC         P62 on P1 header !!!!CHANGED FROM P2
//   RESET (R)  P18 on P2 header
//   OLEDCS (OC) P63 on  p3 header        !!!CHANGED FROM P4
//   SDCS (SC)  n.c. (no connection)
//   MISO (SO)  n.c. (no connection)
//   CD         n.c. (no connection)
//   3V         n.c. (no connection)
//   Vin (+)    3.3V
//   GND (G)    GND
//*****************************************************************************


// Standard includes
//#include <string.h>
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
#include "gpio.h"

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
float MAX_SPEED = 0.2;
int RADIUS = 4;
int INVERTY = 1;
int INVERTX = 0;
int PROGBALL = 0;
int PROGCHAR = 1;
int PROGREADREG  = 2;
int program = 0;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************

void testChar(){
    int i = 0;
    for(i = 0; i <= 255; i++){
        if(program != PROGCHAR){
            return;
        }
        fillScreen(BLACK);
        drawChar(rand()%10, rand()%10, i, RED, BLACK, 3);
        MAP_UtilsDelay(DELAY);
    }

    return;
}

//void SW2(){
//    if(program > 0){
//        program--;
//    }else{
//        program = 2;
//    }
//    return;
//}
//
//void SW3(){
//    if(program < 2){
//        program ++;
//    } else{
//        program = 0;
//    }
//}

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
//    int bufferIndex = 0;
//    printf("READ CONTENTS (0x): ");
//    while(bufferIndex < ucRdLen){
//        printf("%d, ", pucData[bufferIndex]);
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
//!         values range between -64, 64
//!
//! This function
//!    1. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int getAcc(int iData[3]){
    unsigned char buffer[6];
    RET_IF_ERR(readReg(0x18, &buffer[0], 2, 6));

    // Convert and assign to integer array. If INVERT is true, then the value will be 255 - value
    iData[0] = (INVERTX)? 255 - (int)buffer[1] : (int)buffer[1];
    iData[1] = (INVERTY)? 255 - (int)buffer[3] : (int)buffer[3];
    iData[2] = (int)buffer[5];

    //  Map the range from 0-255 to -64-64
    int i = 0;
    for(i=0; i<3; i++){
        if(iData[i] > 128){
            iData[i] = iData[i] - 255;
        }
        if(iData[i] < -64) {iData[i] = -64;}
        else if(iData[i] > 64) {iData[i] = 64;}
    }

    return SUCCESS;
}

void slidingBall(){
    printf("begin sliding ball \n");
    float pos[2] = {WIDTH/2, HEIGHT/2}; //position
    int acc[3];   //acceleration
    fillCircle((int)pos[0], (int)pos[1], RADIUS, MAGENTA);

    while(1){
        fillCircle(pos[0], pos[1], RADIUS, GREEN);

        //  Update the x y position
        getAcc(acc);
        int i = 0;
        for(i = 0; i < 2; i++){
            pos[i] = pos[i] + MAX_SPEED*(float)acc[i]/(float)64;

            //  X Y Boundaries
            if(pos[i] > WIDTH-RADIUS){
                pos[i] = WIDTH-RADIUS;
            }else if(pos[i] < 0+RADIUS){
                pos[i] = RADIUS;
            }
        }
        //printf("%d, %d\n", acc[0], acc[1]);
        fillCircle((int)pos[0], (int)pos[1], RADIUS, MAGENTA);
    }
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
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);
    unsigned char buf[1];
    while(I2C_IF_Read(0x18, &buf[0], 1) != SUCCESS){
        printf("Waiting for I2C...\n");
    }

    //
    // Initialize Adafruit OLED
    //
    Adafruit_Init();
    fillScreen(BLACK);

    //
    //  Enable Interrupts for Switches
    //
//    GPIOIntRegister(GPIOA1_BASE, &SW3);
//    GPIOIntRegister(GPIOA2_BASE, &SW2);
//    GPIOIntClear(GPIOA1_BASE, 0x20);
//    GPIOIntClear(GPIOA2_BASE, 0x40);
//    GPIOIntEnable(GPIOA1_BASE, 0x20);
//    GPIOIntEnable(GPIOA2_BASE, 0x40);

    //Sliding Ball
    slidingBall();

//  TODO: For implementing switch interrupts
//    while(1){
//        switch(program){
//        case PROGBALL:
//            printf("PROGRAM: SLIDING BALL \n");
//            slidingBall();
//            break;
//        case PROGCHAR:
//            printf("PROGRAM: DRAW CHAR \n");
//            testChar();
//            break;
//        case PROGREADREG:
//            unsigned char buffer[1];
//            RET_IF_ERR(readReg(0x18, &buffer[0], 3, 1));
//            printf("x: %d, ", buffer[0]);
//            RET_IF_ERR(readReg(0x18, &buffer[0], 5, 1));
//            printf("y: %d\n ", buffer[0]);
//            MAP_UtilsDelay(DELAY);
//            break;
//        }
//    }

    //  READ X AND Y REGISTERS
//    unsigned char buffer[1];
//    while(1){
//        RET_IF_ERR(readReg(0x18, &buffer[0], 3, 1));
//        printf("x: %d, ", buffer[0]);
//        RET_IF_ERR(readReg(0x18, &buffer[0], 5, 1));
//        printf("y: %d\n ", buffer[0]);
//        MAP_UtilsDelay(DELAY);
//    }

    return;
}
