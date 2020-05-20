//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/init/src/init_flash.c $
  $LastChangedRevision: 591974 $
  $LastChangedDate: 2020-03-16 09:23:12 -0600 (Mon, 16 Mar 2020) $
  $LastChangedBy: grueter $
-------------------------------------------------------------------------------
  Project:      ATA5700
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 6.30.1
-------------------------------------------------------------------------------

******************************************************************************
* Copyright 2017, Microchip Technology Incorporated and its subsidiaries.     *
*                                                                             *
* This software is owned by the Microchip Technology Incorporated.            *
* Microchip hereby grants to licensee a personal                              *
* non-exclusive, non-transferable license to copy, use, modify, create        *
* derivative works of, and compile the Microchip Source Code and derivative   *
* works for the sole and exclusive purpose of creating custom software in     *
* support of licensee product to be used only in conjunction with a Microchip *
* integrated circuit as specified in the applicable agreement. Any            *        
* reproduction, modification, translation, compilation, or representation of  *
* this software except as specified above is prohibited without the express   *
* written permission of Microchip.                                            *
*                                                                             *
* Disclaimer: MICROCHIP MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,    *
* WITH REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED    *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.         *
* Microchip reserves the right to make changes without further notice to the  *
* materials described herein. Microchip does not assume any liability arising *
* out of the application or use of any product or circuit described herein.   *
* Microchip does not authorize its products for use as critical components in *
* life-support systems where a malfunction or failure may reasonably be       *
* expected to result in significant injury to the user. The inclusion of      *
* Microchip products in a life-support systems application implies that the   *
* manufacturer assumes all risk of such use and in doing so indemnifies       *
* Microchip against all charges.                                              *
*                                                                             *
* Use may be limited by and subject to the applicable Microchip software      *
* license agreement.                                                          *
******************************************************************************/
/** \file init_flash.c
*/

//lint -restore

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "..\..\init\src\init_flash.h"
#include "..\..\spi\src\ata5700_command_set_flash.h"
#include "..\..\eep\src\eep.h"
#include "..\..\eep\src\eep_flash.h"
#include "..\..\globals\src\globals.h"
#include "..\..\spi\src\spi.h"
#include "..\..\spi\src\spi_flash.h"

#include "..\..\timer1\src\timer1.h"
#include "..\..\timer2\src\timer2.h"
#include "..\..\timer3\src\timer3.h"
#include "..\..\timer4\src\timer4.h"
#include "..\..\timer5\src\timer5.h"

#include "..\..\..\appl\appFlash\src\FlashApplVars.h"
extern sFlashApplState gFlashApplState;
/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
/** \brief <b>tmpAryApp</b>
    used as scratch memory e.g. for block read from EEPROM.
*/
#pragma location = ".sram_FlashApp_TempArray"
//__no_init static uint8_t m_bTempArray_flash[10];
__no_init uint8_t m_bTempArray_flash[10];

#pragma location = ".sram_FlashApp_Sleep"
__no_init uint8_t g_bSleepModeConfig_flash;

#pragma location = ".sram_FlashApp_EventHandling"
__no_init sSramFlashAppEventHandling g_sEventHandling_flash;

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initAta5700_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initAta5700_flash_C(void)
{
    // ------------------------------------------------------------------------
    // SW initialization
    // ------------------------------------------------------------------------
    ATA_timer1Init_C();
    ATA_timer2Init_C();
    ATA_timer3Init_C();
    ATA_timer4Init_C();
    ATA_timer5Init_C();

    // ------------------------------------------------------------------------
    // HW initialization
    // ------------------------------------------------------------------------
    // IO Module CLK
    if (ATA_initIoModuleClk_flash_C() == FAIL){return FAIL;}
    // IO Module INT
    if (ATA_initIoModuleInt_flash_C() == FAIL){return FAIL;}
    // IO Module CPU
    if (ATA_initIoModuleCpu_flash_C() == FAIL){return FAIL;}
    // IO Module DEBOUNCE
    if (ATA_initIoModuleDebounce_flash_C() == FAIL){return FAIL;}
    // IO Module DEBUG
    if (ATA_initIoModuleDebug_flash_C() == FAIL){return FAIL;}
    // IO Module SUP
    if (ATA_initIoModuleSup_flash_C() == FAIL){return FAIL;}
    // IO Module TIMER0_WDT
    if (ATA_initIoModuleTimer0Wdt_flash_C() == FAIL){return FAIL;}
    return OK;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>gpio_init</b>
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC gpio_init(void)
{
  // Set ports for LED
  bit_clear(LED1);
  bit_clear(LED2);
  bit_set(LED1_DDR);
  bit_set(LED2_DDR);
  
  // Set pins for switches
  bit_set(SW1); 
  bit_set(SW2);
  bit_set(SW3);
  bit_set(SW1_DDR);
  bit_set(SW2_DDR);
  bit_set(SW3_DDR);
  
  // Set pins for Transparent mode TX
  bit_clear(CLKOUT);
  bit_set(CLKOUT_DDR);          
  bit_clear(TMDI_STATE);        
  bit_clear(TMDI_DDR);      
  bit_set(TXACTIVE);      //PB0 INT0 pull up active
  bit_clear(TXACTIVE_DDR);
            
            
  /* enable pin change interrupts */
  bit_set(SW1_INTR);
  bit_set(SW2_INTR);
  bit_set(SW3_INTR);
  
 // PCICR = 2;          // enable pin change int on PCINT[7:0]
  
 // PCICR |= (1<<PCIE1);      //enable pin change int on PCINT[15:8]
  SPCR &= ~((1<<SPIE) | (1<<SPE)); // Disable SPI port
  SPSR &= ~((1<<SPIF) | (1<<TXIF) | (1<<RXIF));
    
  gFlashApplState.Buttons = 0x00U;//Clear button state
 // PCICR = 2;          // enable pin change int on PCINT[7:0]

  
    EICRA = (1<<ISC01); // Falling edge of INT0 (NSS) generates interrupt
  //  EIFR = 0x01; //Clear INT0 flag
  //  DDRD |= 0x02;
    PCMSK1 |= 0x04;//Enable Pin change interrupt 10
 //   PCIFR |= 0x02; //clear pin change interrupt bank 1 flag
  //  PCMSK1 |= 0x02; //enable pin cahnge interrupt bank 1  
    PCICR |= 0x02;
  
}
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initEventHandling_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initEventHandling_flash_C(void)
{
    uint8_t fRetVal = FAIL;
    g_sEepFlashAppEventHandling_flash.bComponents = 0;
    if (ATA_eepReadBytes_C((uint8_t *)&g_sEventHandling_flash,(uint16_t)&g_sEepFlashAppEventHandling_flash,sizeof(sEepFlashAppEventHandling)) == EEC_NO_ERROR) {
        fRetVal = OK;
    }
    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initIoModuleClk_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initIoModuleClk_flash_C(void)
{
    uint8_t fRetVal = FAIL;
    uint8_t bFrcAlwaysOn;
    
    if (ATA_eepReadBytes_C(m_bTempArray_flash,(uint16_t)&g_sEepFlashAppClk_flash,sizeof(sEepFlashAppClk)) == EEC_NO_ERROR)
    {
        PRR0   = m_bTempArray_flash[0];
        PRR1   = m_bTempArray_flash[1] & (~(BM_PRLFPH | BM_PRLFTP | BM_PRLFR));
        PRR2   = m_bTempArray_flash[2];
        
        /* Enable Clock Output clock domain to set corresponding registesr */
        uint8_t bPrr0 = PRR0;
        ATA_POWERON_C(PRR0, PRCO)
        CLKOD  = m_bTempArray_flash[3];
        CLKOCR = m_bTempArray_flash[4];
        CLKOD = 0x05;
        CLKOCR |= 0x03; //CLK SOURCE XTO (CLKOUT = 4.08 MHz)
        CLKOCR |= 0x04; //Enable CLKOUT
        PRR0 = bPrr0;

        /* FRC always on setting, that is FRC may get activated immediately */
        bFrcAlwaysOn = m_bTempArray_flash[5] & BM_FRCAO;
        if ( bFrcAlwaysOn != 0x00U )
        {
            CMOCR |= bFrcAlwaysOn;
            do
            {
                _NOP;
            } while((CMOCR & BM_FRCACT) == 0U);
        }
        
        /* Since FRC activation requires that MVCC and DVCC high enable are
           enabled before, just setting the FRCAO bit is no longer allowed. */
        if ( (m_bTempArray_flash[5] & BIT_MASK_7) == 0x00U )
        {
            /* Activate FRC to be able to process SPI commands at a high data rate */
            ATA_globalsClkSwitchFrc_C();
        }

        /* Set prescaler value system timer to 1 and system clock to 1 */
        ATA_globalsSetClk_C(BM_CLTPS0);

        fRetVal = OK;
    }

    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initIoModuleCpu_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initIoModuleCpu_flash_C(void)
{
    uint8_t fRetVal = FAIL;
    uint8_t bTempMcucr = MCUCR;
    
    if (ATA_eepReadBytes_C(m_bTempArray_flash,(uint16_t)&g_sEepFlashAppCpu_flash,sizeof(sEepFlashAppCpu)) == EEC_NO_ERROR)
    {
        /* Do not modify Interrupt Vector Table location */
        bTempMcucr &= (BM_IVSEL|BM_IVL1|BM_IVL0);
        m_bTempArray_flash[0] &= ~(BM_IVSEL|BM_IVL1|BM_IVL0);
        m_bTempArray_flash[0] |= bTempMcucr;
        
        MCUCR = m_bTempArray_flash[0];

        /* Do not set Sleep Enable during initialization */
        g_bSleepModeConfig_flash = m_bTempArray_flash[1];

        fRetVal = OK;
    }
    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initIoModuleDebounce_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initIoModuleDebounce_flash_C(void)
{
    uint8_t fRetVal = FAIL;
    if (ATA_eepReadBytes_C(m_bTempArray_flash,(uint16_t)&g_sEepFlashAppDebounce_flash,sizeof(sEepFlashAppDebounce)) == EEC_NO_ERROR) {
        DBCR   = m_bTempArray_flash[0];
        DBTC   = m_bTempArray_flash[1];
        DBENB  = m_bTempArray_flash[2];
        DBENC  = m_bTempArray_flash[3];
        DBEND  = m_bTempArray_flash[4];
        fRetVal = OK;
    }
    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initIoModuleDebug_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initIoModuleDebug_flash_C(void)
{
    uint8_t fRetVal = FAIL;
    uint8_t bSreg = SREG;

    if (ATA_eepReadBytes_C(m_bTempArray_flash,(uint16_t)&g_sEepFlashAppDebug_flash,sizeof(sEepFlashAppDebug)) == EEC_NO_ERROR) {
        // enable tracer if selected in variable g_sEepFlashAppDebug_flash.trace
        if (m_bTempArray_flash[0] & BIT_MASK_0)
        {
            __disable_interrupt();
            MCUCR |= BM_TRCCE;
            MCUCR |= BM_TRCEN;
            SREG = bSreg;
        }

        DBGSW  = m_bTempArray_flash[1];
        fRetVal = OK;
    }
    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initIoModuleInt_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initIoModuleInt_flash_C(void)
{
    uint8_t fRetVal = FAIL;
    if (ATA_eepReadBytes_C(m_bTempArray_flash,(uint16_t)&g_sEepFlashAppInt_flash,sizeof(sEepFlashAppInt)) == EEC_NO_ERROR) {
        PCICR  = m_bTempArray_flash[0];
        EIMSK  = m_bTempArray_flash[1];
        EICRA  = m_bTempArray_flash[2];
        PCMSK0 = m_bTempArray_flash[3];
        //PCMSK1 = m_bTempArray_flash[4];
        fRetVal = OK;
    }
    return fRetVal;
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initIoModulePortB_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initIoModulePortB_flash_C(void)
{
    uint8_t fRetVal = FAIL;
    if (ATA_eepReadBytes_C(m_bTempArray_flash,(uint16_t)&g_sEepFlashAppPortB_flash,sizeof(sEepFlashAppPort)) == EEC_NO_ERROR) {
        DDRB   = m_bTempArray_flash[0];
        PORTB  = m_bTempArray_flash[1];
        
        /* EVENT pin initialization */
        DDRB |= BM_DDRB3;
        if( g_sEventHandling_flash.bConfig & BM_ATA5700_EVENTS_CONFIG_EVENTPIN_POL )
        {
            PORTB &= (uint8_t)~BM_PORTB3;  // EVENTPIN_POL = 1 --> high active
        }
        else
        {
            PORTB |= BM_PORTB3;            // EVENTPIN_POL = 0 --> low active
        }
        
        fRetVal = OK;
    }
    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initIoModulePortC_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initIoModulePortC_flash_C(void)
{
    uint8_t fRetVal = FAIL;
    if (ATA_eepReadBytes_C(m_bTempArray_flash,(uint16_t)&g_sEepFlashAppPortC_flash,sizeof(sEepFlashAppPort)) == EEC_NO_ERROR) {
        DDRC   = m_bTempArray_flash[0];
        PORTC  = m_bTempArray_flash[1];

        fRetVal = OK;
    }
    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initIoModulePortD_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initIoModulePortD_flash_C(void)
{
    uint8_t fRetVal = FAIL;
    if (ATA_eepReadBytes_C(m_bTempArray_flash,(uint16_t)&g_sEepFlashAppPortD_flash,sizeof(sEepFlashAppPort)) == EEC_NO_ERROR) {
        DDRD   = m_bTempArray_flash[0];
        PORTD  = m_bTempArray_flash[1];
        fRetVal = OK;
    }
    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initIoModuleSpi_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initIoModuleSpi_flash_C(void)
{
    uint8_t fRetVal = FAIL;
    if (ATA_eepReadBytes_C(m_bTempArray_flash,(uint16_t)&g_sEepFlashAppSpi_flash,sizeof(sEepFlashAppSpi)) == EEC_NO_ERROR)
    {
        /* Do not initialize SPI if the system is starting due to a Transponder Reset.
           If VBAT is not available, this will result in undefined behaviour. */
        if ( ((g_sAta5700_flash.events_reset & BM_TPRF) == 0x00U) ||
             ((m_bTempArray_flash[0] & BM_SPIE) != 0x00U )
           )
        {
            /* If SPI interface is configured as master */
            if ( m_bTempArray_flash[0] & BM_MSTR )
            {
                fRetVal = ATA_spiOpen_C(m_bTempArray_flash[0], m_bTempArray_flash[1]);
            }
            else
            {
                fRetVal = ATA_spiSlaveOpen_flash_C(m_bTempArray_flash[0], m_bTempArray_flash[1]);
            }
        }
        else
        {
            fRetVal = OK;
        }
    }
    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initIoModuleSup_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initIoModuleSup_flash_C(void)
{
    uint8_t fRetVal = FAIL;
    if (ATA_eepReadBytes_C(m_bTempArray_flash,(uint16_t)&g_sEepFlashAppSup_flash,sizeof(sEepFlashAppSup)) == EEC_NO_ERROR)
    {
        /* Do not activate VMEM */
        m_bTempArray_flash[0] &= (BM_VMEMEN|BM_VMRESM);
        SUPCR |= m_bTempArray_flash[0];
        // VMCR  setting
        ATA_globalsSetVoltageMonitor_C(m_bTempArray_flash[1]);
        fRetVal = OK;
    }
    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initIoModuleTimer0Wdt_flash_C</b>
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_initIoModuleTimer0Wdt_flash_C(void)
{
    uint8_t retVal = FAIL;
    if (ATA_eepReadBytes_C(m_bTempArray_flash,(uint16_t)&g_sEepFlashAppTimer0Wdt_flash.WDTCR, 1U) == EEC_NO_ERROR) {
        ATA_globalsWdtEnable_C(m_bTempArray_flash[0]);
        retVal = OK;
    }
    return retVal;
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_initIoModuleTimer0Wdt_flash_C</b>
    --> can be also done in clib in low_level_init.c
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_initAvrRegister_flash_C(void)
{
    __asm("LDI  R31,0");
    __asm("LDI  R30,0");
    /* R29 used for STACK */
    /* R28 used for STACK */
    __asm("LDI  R27,0");
    __asm("LDI  R26,0");
    __asm("LDI  R25,0");
    __asm("LDI  R24,0");
    __asm("LDI  R23,0");
    __asm("LDI  R22,0");
    __asm("LDI  R21,0");
    __asm("LDI  R20,0");
    __asm("LDI  R19,0");
    __asm("LDI  R18,0");
    __asm("LDI  R17,0");
    __asm("LDI  R16,0");
    __asm("MOV  R15,R30");
    __asm("MOV  R14,R30");
    __asm("MOV  R13,R30");
    __asm("MOV  R12,R30");
    __asm("MOV  R11,R30");
    __asm("MOV  R10,R30");
    __asm("MOV  R9,R30");
    __asm("MOV  R8,R30");
    __asm("MOV  R7,R30");
    __asm("MOV  R6,R30");
    __asm("MOV  R5,R30");
    __asm("MOV  R4,R30");
    __asm("MOV  R3,R30");
    __asm("MOV  R2,R30");
    __asm("MOV  R1,R30");
    __asm("MOV  R0,R30");
}