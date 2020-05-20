/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include <stdio.h>
#include "ata5702_flash.h"
#include "ata5702_eeprom.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appObject;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/

DRV_HANDLE SPIHandle;
DRV_SPI_BUFFER_HANDLE spi_buffer_handle;


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

//ata5702 sig
#define SIG_BYTE1 0x1e
#define SIG_BYTE2 0x95
#define SIG_BYTE3 0x69
#define BUF_LENGTH 4
#define WORD_IN_PAGE 32	//ata5702 Note 32 words is for flash, eeprom is 16
#define READ_CORE_TIMER()   _CP0_GET_COUNT()          // Read the MIPS Core Time

int page=0;
int addr=0x4000;
int data=0;
int i=0;

struct packet {
	uint8_t byte[4];
} tx_packet;

uint8_t tx_buffer[BUF_LENGTH];
uint8_t rx_buffer[BUF_LENGTH];

uint8_t prog_en[BUF_LENGTH]= {
0xac, 0x53, 0x00, 0x00
};

uint8_t read_sig1[BUF_LENGTH]= {
	0x30, 0x00, 0x00, 0x00
};
uint8_t read_sig2[BUF_LENGTH]= {
	0x30, 0x00, 0x01, 0x00
};
uint8_t read_sig3[BUF_LENGTH]= {
	0x30, 0x00, 0x02, 0x00
};

uint8_t chip_erase[BUF_LENGTH]= {
	0xac, 0x80, 0x00, 0x00
};

uint8_t poll_flag[BUF_LENGTH]= {
	0xf0, 0x00, 0x00, 0x00
};

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
       
/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appObject.state = APP_STATE_INIT;
 
    LED1Off();
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

void spi_error(void)
{
	printf("\r\nERROR!!!\r\n");
	while(1);
}

void BSP_DelayUs(uint16_t microseconds)
{
    uint32_t time;
    
    time = READ_CORE_TIMER(); // Read Core Timer    
    time += (SYS_CLK_FREQ / 2 / 1000000) * microseconds; // calc the Stop Time    
    while ((int32_t)(time - READ_CORE_TIMER()) > 0){};    
}

void APP_Tasks(void)
{

    switch (appObject.state)
    {
        case APP_STATE_INIT:
            appObject.state = APP_ISP_PROGRAM_ATA;
            SPIHandle = DRV_SPI_Open(DRV_SPI_INDEX_0, DRV_IO_INTENT_READWRITE);
            
            break;
         
        case APP_ISP_PROGRAM_ATA:
            printf("Enable ATA ISP\r\n");
            
            CS1Off();   //drive ATA nRESET low
            BSP_DelayUs(400);   //per BU ~375us from cs to sck

            spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, prog_en, 4, rx_buffer, 4, 0, 0);
            while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle)); //wait until spi is done         

            

            
#if 1	//read signature
            printf("Read Signature: ");
            spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, read_sig1, 4, rx_buffer, 4, 0, 0);
            while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done
            printf("0x%02x, ", rx_buffer[3]);

            spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, read_sig2, 4, rx_buffer, 4, 0, 0);
            while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done
            printf("0x%02x, ", rx_buffer[3]);

            spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, read_sig3, 4, rx_buffer, 4, 0, 0);
            while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done
            printf("0x%02x\r\n", rx_buffer[3]);

#endif

#if 1   //chiperase
            printf("Chip Erase...");
            spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, chip_erase, 4, rx_buffer, 4, 0, 0);
            while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done
            if(rx_buffer[2] != 0x80) spi_error();

            //poll
            rx_buffer[3] = 0xff;
            while(rx_buffer[3] == 0xff){
                spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, poll_flag, 4, rx_buffer, 4, 0, 0);
                while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done

            }
            printf("..Done\r\n");
#endif

#if 1	//flash MUST be programmed first before eeprom. ~16sec
            printf("Programming flash...");
            for (page=0; page<sizeof(flash)/(WORD_IN_PAGE*2)+1; page++) {	//calculate number of page needed. last page can have < WORD_IN_PAGE
                for(i=0; i<WORD_IN_PAGE; i++) {	//fill the 64-byte buffer (=32 words)
                    //fill the page buffer
                    tx_packet.byte[0] = 0x40;	//lower byte
                    tx_packet.byte[1] = (uint8_t)(addr>>8);
                    tx_packet.byte[2] = (uint8_t)addr;
                    tx_packet.byte[3] =	flash[data++];
                    spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, &tx_packet, 4, rx_buffer, 4, 0, 0);
                    while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done
                    tx_packet.byte[0] = 0x48;	//upper byte
                    tx_packet.byte[1] = (uint8_t)(addr>>8);
                    tx_packet.byte[2] = (uint8_t)addr;
                    tx_packet.byte[3] = flash[data++];
                    spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, &tx_packet, 4, rx_buffer, 4, 0, 0);
                    while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done

                    addr++;
                }

                //commit to write
                tx_packet.byte[0] = 0x4c;	//write page
                tx_packet.byte[1] = (uint8_t)((addr-WORD_IN_PAGE)>>8);
                tx_packet.byte[2] = (uint8_t)(addr-WORD_IN_PAGE);
                tx_packet.byte[3] = 0;
                spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, &tx_packet, 4, rx_buffer, 4, 0, 0);
                while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done

                //poll
                rx_buffer[3] = 0xff;
                while(rx_buffer[3] == 0xff){
                    spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, poll_flag, 4, rx_buffer, 4, 0, 0);
                    while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done
                }   //for i

                printf(".");	//note every page

            }   //for page

            printf("..Done\r\n");
#endif

//Verify flash	~13sec
#if 1
            printf("Verifying flash...");
            addr=0x4000;

            for(i=0; i<flash_size; i++) {	
                tx_packet.byte[0] = 0x20;	//read low byte
                tx_packet.byte[1] = (uint8_t)(addr>>8);
                tx_packet.byte[2] = (uint8_t)(addr);
                tx_packet.byte[3] = 0;
                spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, &tx_packet, 4, rx_buffer, 4, 0, 0);
                while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done
                if(rx_buffer[3]!=flash[i++])	{
                    printf("Verify fail at addr %04x expecting %02x but read %02x\r\n", addr, flash[i], rx_buffer[3]);
                    while(1);
                }

                tx_packet.byte[0] = 0x28;	//read high byte
                tx_packet.byte[1] = (uint8_t)(addr>>8);
                tx_packet.byte[2] = (uint8_t)(addr);
                tx_packet.byte[3] = 0;
                spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, &tx_packet, 4, rx_buffer, 4, 0, 0);
                while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done
                if(rx_buffer[3]!=flash[i])	{
                    printf("Verify fail at addr %04x expecting %02x but read %02x\r\n", addr, flash[i+1], rx_buffer[3]);
                    while(1);
                }

                addr++;
            }
            printf("..Done\r\n");
#endif


#if 1	//EEPROM	 ~3sec
            printf("Programming eeprom...");
            addr=0;	//reuse but reset variables
            data=0;

            for (page=0; page<sizeof(eeprom)/(WORD_IN_PAGE/2)+1; page++) {	//calculate number of page needed. last page can have < WORD_IN_PAGE
                for(i=0; i<WORD_IN_PAGE/2; i++) {	//fill the 16-byte eeprom page buffer
                    //fill the page buffer
                    tx_packet.byte[0] = 0xc1;	//fill page eeprom
                    tx_packet.byte[1] = (uint8_t)addr>>8;
                    tx_packet.byte[2] = (uint8_t)addr;
                    tx_packet.byte[3] =	eeprom[data++];
                    spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, &tx_packet, 4, rx_buffer, 4, 0, 0);
                    while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done
                    addr++;
                }

                //commit to write
                tx_packet.byte[0] = 0xc2;	//write page
                tx_packet.byte[1] = (uint8_t)((addr-WORD_IN_PAGE/2)>>8);
                tx_packet.byte[2] = (uint8_t)(addr-WORD_IN_PAGE/2);
                tx_packet.byte[3] = 0;
                spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, &tx_packet, 4, rx_buffer, 4, 0, 0);
                while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done

                BSP_DelayUs(3000);

                //poll
                rx_buffer[3] = 0xff;
                while(rx_buffer[3] == 0xff){
                    spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, poll_flag, 4, rx_buffer, 4, 0, 0);
                    while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done
                }
                printf(".");
            }
            printf("..Done\r\n");
#endif

#if 1   //Verify EEPROM	~1sec
            printf("Verifying eeprom...");
            addr=0x0;

            for(i=0; i<eeprom_size; i++) {
                tx_packet.byte[0] = 0xa0;	//read byte
                tx_packet.byte[1] = (uint8_t)(addr>>8);
                tx_packet.byte[2] = (uint8_t)(addr);
                tx_packet.byte[3] = 0;
                spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, &tx_packet, 4, rx_buffer, 4, 0, 0);
                while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done

                //	printf("%02x %02x\r\n", rx_buffer[3], eeprom[i]);
                if(rx_buffer[3]!=eeprom[i])	{
                    printf("verify fail at addr %04x expecting %02x but read %02x\r\n", addr, eeprom[i], rx_buffer[3]);
                    while(1);
                }

                addr++;

            }
            printf("..Done\r\n");
#endif

#if 1   //Lock the device. Can't read device memories via ICE if locked!
            printf("Locking Device...");
            tx_packet.byte[0] = 0xac;	//write lock bits command
            tx_packet.byte[1] = 0xe0;
            tx_packet.byte[2] = 0;
            tx_packet.byte[3] = 0;		//lock device
            spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, &tx_packet, 4, rx_buffer, 4, 0, 0);
            while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done
            BSP_DelayUs(3000);

            //poll
            rx_buffer[3] = 0xff;
            while(rx_buffer[3] == 0xff){
                spi_buffer_handle = DRV_SPI_BufferAddWriteRead(SPIHandle, poll_flag, 4, rx_buffer, 4, 0, 0);
                while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle));        //wait until spi is done
            }
            printf("..Done\r\n");
#endif

            printf("\r\nProgramming Complete!\r\n");

            CS1On();   //drive ATA nRESET high

            LED1On();
            appObject.state = APP_SEND_SPI_COMMAND;
            break;

        case APP_SEND_SPI_COMMAND:
            tx_buffer[0] = 0x40;
            printf("\r\nSend SPI command 0x%02x\r\n", tx_buffer[0]);
            CS2Off();   //drive ATA CS low
            BSP_DelayUs(400);   //per BU ~375us from cs to sck

            spi_buffer_handle = DRV_SPI_BufferAddWrite(SPIHandle, tx_buffer, 1, 0, 0);
            while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(spi_buffer_handle)); //wait until spi is done         

            appObject.state = APP_STATE_COMPLETE;
            CS2On();    //drive ATA CS high
            
            break;
            
        case APP_STATE_COMPLETE:
            break;
    }   //switch
}


 

/*******************************************************************************
 End of File
 */
