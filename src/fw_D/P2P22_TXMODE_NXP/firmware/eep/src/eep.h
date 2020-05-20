//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/eep/src/eep.h $
  $LastChangedRevision: 458065 $
  $LastChangedDate: 2017-05-02 04:55:50 -0600 (Tue, 02 May 2017) $
  $LastChangedBy: krishna.balan $
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
/** \file firmware/eep/src/eep.h 
    Include relevant EEPROM structures.
*/
//lint -restore

#ifndef EEP_H
#define EEP_H

#ifdef __IAR_SYSTEMS_ICC__

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../stdc/src/stdc.h"
#include "../../globals/src/globals.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/** \brief <b>EEP_SECRET_KEY_LENGTH</b>
    defines the length of the each single copy of a Secret Key.
*/
#define EEP_SECRET_KEY_LENGTH                               0x10U

/** \brief <b>EEP_NUM_LF_CALIB_REG</b>
    defines the number of LF calibration registers (LFCALR1..53).
*/
#define EEP_NUM_LF_CALIB_REG                                0x35U

/** \brief <b>EEP_NUM_LF_DECODER_REG</b>
    defines the number of LF calibration registers (LFDSR1..11).
*/
#define EEP_NUM_LF_DECODER_REG                              0x0BU

/** \brief <b>EEP_XROW_UID_SIZE</b>
    defines the number of UID bytes.
*/
#define EEP_XROW_UID_SIZE                                   0x04U

/** \brief <b>EEP_ACC_FUSE</b>
    Flag to adjust EEP settings via the Fuses.
*/
#define EEP_ACC_FUSE                                        0x80U

/** \brief <b>EEP_SECTION_ID_MASK</b>
    Mask to isolate the desired section ID
*/
#define EEP_SECTION_ID_MASK                                 0x0FU

/** \brief <b>EEP_FIRST_SECTION_FUSE_ID</b>
    defines the first fusable section ID (section IDs smaller
    are only protectable via registers EEPR1..4)
*/
#define EEP_FIRST_SECTION_FUSE_ID                           0x05U

/** \brief <b>EEP_NUM_SECTION_ID_PER_FUSE_BYTE</b>
    defines the number of sections covered by a fuse byte 
    (high or low)
*/
#define EEP_NUM_SECTION_PER_FUSE_REG_BYTE                   0x04U

/** \brief <b>EEP_ACCESS_RIGHT_INVERT_MASK</b>
    defines the bit mask to invert the access rights of a given 
    access right byte
*/
#define EEP_ACCESS_RIGHT_INVERT_MASK 					    0xFFU

/** \brief <b>EEP_SECTION_ID_IN_EEPR0</b>
    defines EEPR0 register which contains section IDs 0 to 3
*/
#define EEP_SECTION_ID_IN_EEPR0 					        0x00U

/** \brief <b>EEP_SECTION_ID_IN_EEPR1</b>
    defines EEPR0 register which contains section IDs 4 to 7
*/
#define EEP_SECTION_ID_IN_EEPR1 					        0x01U

/** \brief <b>EEP_SECTION_ID_IN_EEPR2</b>
    defines EEPR0 register which contains section IDs 8 to 11
*/
#define EEP_SECTION_ID_IN_EEPR2 					        0x02U

/** \brief <b>EEP_SECTION_ID_IN_EEPR3</b>
    defines EEPR0 register which contains section IDs 12
*/
#define EEP_SECTION_ID_IN_EEPR3 					        0x03U

#define FUSE_EP_L                                           0x00U
#define FUSE_EP_H                                           0x01U
#define FUSE_EP_F                                           0x02U
#define BM_GET_FUSE_EP_L                                    0x29U
#define BM_GET_FUSE_EP_H                                    0x39U
#define BM_GET_FUSE_SECF                                    0x19U
#define HAVE_NO_AESKR                                       0x04U

/** \brief <b>EEP_ACCESS_RIGHT_MASK</b>
    defines the bit mask for one EEPROM section
*/
#define EEP_ACCESS_RIGHT_MASK 								0x03U

/** \brief <b>EEP_ADDR_START_OUT_OF_RANGE_SEC</b>
    defines the start of the EEPROM address range which is considered
    out of range.
*/
#define EEP_ADDR_START_OUT_OF_RANGE_SEC                     0x0980U

/** \brief <b>EEP_ADDR_END_ATMEL_SEC</b>
    defines the end of the Atmel EEPROM section
*/
#define EEP_ADDR_END_ATMEL_SEC                              0x097FU

/** \brief <b>EEP_ADDR_START_ATMEL_SEC</b>
    defines the beginning of the Atmel EEPROM section, which is read only and
    therefore marks the first address which is considered out of range when it
    comes to an EEPROM write access.
*/
#define EEP_ADDR_START_ATMEL_SEC                            0x0880U

/** \brief <b>EEP_ADDR_CUSTO_SEC</b>
    defines the end of the Customer EEPROM section
*/
#define EEP_ADDR_END_CUSTO_SEC                              0x087FU

/** \brief <b>EEP_ADDR_CUSTO_SEC</b>
    defines the beginning of the Customer EEPROM section
*/
#define EEP_ADDR_START_CUSTO_SEC                            0x0800U

/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/
/*----------------------------------------------------------------------------- */
/** \brief <b>sAtmelEEPromSection</b>
    Structure used to store the ATMEL specific EEProm content. Each P2P leaving
    ATMEL will have this section programmed and configured and locked, the user
    will not be able to alter anything inside this section. The sections size is
    128 bytes. The section contains all the Device-Die specific data.
*/
/*----------------------------------------------------------------------------- */
typedef struct{
	/**
	 * Variable is read only.
	 * ResetSM, used by digital HW.
	 * Device Id used by AtmelStudio.
	 */
	uint8_t eepChipId[3];      
	/**
	 * Variable is read only.
	 * ResetSM, used by digital HW.
	 */
	uint8_t eepXROWFUSE;
	/**
	 * Variable is read only.
	 * ResetSM, used by digital HW.
	 */
	uint8_t eepMRCCAL;   
	/**
	 * Variable is read only.
	 * ResetSM, used by digital HW.
	 */
	uint8_t eepFRCCAL;
	/**
      * Variable is read only.
      * ResetSM, used by digital HW.
      */
    uint8_t eepRCTCAL1; 
	/**
	 * Variable is read only.
	 * ResetSM, used by digital HW.
	 */     
	uint8_t eepSUPCAL[10];   
	/**
	 * Variable is read only.
	 * ResetSM, used by digital HW.
	 */
	uint8_t eepMEMCAL[3];   
	/**
	 * Variable is read only.
	 * ResetSM, used by digital HW.
	 */
	uint8_t eepDevId1Dw;       
	/**
	 * Variable is read only.
	 * ResetSM, used by digital HW.
	 */
	uint8_t eepTPCAL[10];
	/**
	 * Variable is read only.
	 * ResetSM, used by digital HW.
	 */
	uint8_t eepCalRdy;

    /**
	 * Variable is read only.
	 * LOT/Wafer/Die/Foundry traceability
	 */
	uint8_t eepTraceability[9];
    /**
	 * Variable is read only.
	 * AOIP protocol version, ROM version
	 */
	uint8_t eepDeviceInfo[4];
    /**
	 * Spare value 1
	 */
	uint8_t eepAtmelSecSpareOne;
    /**
	 * Variable is read only.
	 * SRC temperature calibration setting.
	 */
	uint8_t eepSRCTCAL;
    /**
	 * Variable is read only.
	 * SRC calibration setting.
	 */
	uint8_t eepSRCCAL;
    /**
	 * Variable is read only.
	 * Device unique ID.
	 */
	uint8_t eepUID[4];
    /**
	 * Variable is read only.
	 * RSSI range correction values per channel switch.
	 */
    uint8_t eepRssiRangeCorrectionValues[9];
    /**
	 * Variable is read only.
	 * RSSI pre-divider switch value.
	 */
    uint8_t eepRssiPreDividerSwitchValue;
    
    /**
     */
    uint8_t eepRssiNamelessRegister;
	/**
	 * Spare value 2
	 */
	uint8_t eepAtmelSecSpareTwo[1];
    /**
	 * Variable is read only.
	 * LF Receiver calibration settings.
	 */
	uint8_t eepLFCALR[53];
    /**
	 * Variable is read only.
	 * Transponder calibration settings.
	 */
    uint8_t eepTPCALR11;
    uint8_t eepTPCALR12;
    uint8_t eepTPCALR13;
    /**
	 * Variable is read only.
	 * LF Freq decoder settings.
	 */
	uint8_t eepLFDSR[11];
    /**
	 * Variable is read only.
	 * Calibration value for the front end FEBT register.
	 */
	uint8_t eepFEBT;
    /**
	 * Variable is read only.
	 * Calibration value for the front end FETN4 register.
	 */
	uint8_t eepFETN4;
	/**
	 * Variable is read only.
	 * Offset calibration value for the front end VCO.
	 */
	uint8_t eepFEVCOoffset;
	/**
	 * Spare value 3
	 */
	uint8_t eepAtmelSecSpareThree[120];
	/**
	 * Variable is read only.
	 * CRC-16 checksum of the sections content.
	 */
	uint16_t eepLockCrc16;
}
 sAtmelEEPromSection;
/*----------------------------------------------------------------------------- */
/** \brief <b>sCustomerEEPromSection</b>
    Structure used to store the Customers specific EEProm content. Each P2P leaving
    ATMEL will have this section unprogrammed and unconfigured and unlocked, the
    user will have to write the sections content to his needs. The sections size is
    128 bytes. The section will after the setup be locked by the Customer.
*/
/*----------------------------------------------------------------------------- */
typedef struct{
    /**
	 * Read only variable.
	 * Setting for P2P Transponder
	 */
	uint8_t eepTpConfig;
    /**
	 * Read only variable.
	 * Setting for register TPCR1
	 */
	uint8_t eepTPCR1;
	/**
	 * Read only variable.
	 * Setting for register TPCR2
	 */
	uint8_t eepTPCR2;
	/**
	 * Read only variable.
	 * Setting for register TPCR5
	 */
	uint8_t eepTPCR5;
	/**
	 * Read only variable.
	 * Setting for authentication Challenge Length
	 */
	uint8_t eepCalLen;
	/**
	 * Read only variable.
	 * Setting for authentication Response Length
	 */
	uint8_t eepResLen;
	/**
	 * Read only variable.
	 * Setting for register WDTCR
	 */
	uint8_t eepWDTCR;
	/**
	 * Read only variable.
	 * Setting for register VMCR
	 */
	uint8_t eepVMCR;
	/**
	 * Read only variable.
	 * Setting for TP response delay after RX of the request
	 */
	uint8_t eepTPresponseDelay;
	/**
	 * Read only variable.
	 * Setting for register TPDCR1
	 */
	uint8_t eepTPDCR1;
	/**
	 * Read only variable.
	 * Setting for register TPDCR2
	 */
	uint8_t eepTPDCR2;
	/**
	 * Read only variable.
	 * Setting for register TPDCR3
	 */
	uint8_t eepTPDCR3;
	/**
	 * Read only variable.
	 * Setting for register TPDCR4
	 */
	uint8_t eepTPDCR4;
	/**
	 * Read only variable.
	 * Setting for register TPDCR5
	 */
	uint8_t eepTPDCR5;
	/**
	 * Read only variable.
	 * Setting for register PHCRPL
	 */
	uint8_t eepPHCSTL4;
	/**
	 * Read only variable.
	 * Setting for register PHCSTL
	 */
	uint8_t eepPHCRPL4;
	/**
	 * Read only variable.
	 * Setting for register PHCRPL
	 */
	uint8_t eepPHCSTL8;
	/**
	 * Read only variable.
	 * Setting for register PHCSTL
	 */
	uint8_t eepPHCRPL8;
	/**
	 * Read only variable.
	 * Setting for TP sync length
	 */
	uint8_t eepTPsyncLen;
	/**
	 * Read only variable.
	 * Setting for TP preamble length
	 */
	uint8_t eepTPpreaLen;
	/**
	 * Read only variable.
	 * Setting for register TP synchronization
	 */
	uint8_t eepTPSync[4];
	/**
	 * Read only variable.
	 * Setting for register TPECMR
	 */
	uint8_t eepTPECMR;
    /**
	 * Read only variable.
	 * Setting for system clock prescaling. Written to register CLPR.
	 */
	uint8_t eepSysClocking;
    /**
	 * Read only variable.
	 * Setting for delaying the calculation of the response data. 
     * Complete Response delay will be an addition of this and eepTResponseDelay
	 */
	uint8_t eepTCalcDelay;
    /**
	 * Read only variable.
	 * Setting for a temporary offset to the damping level.
	 */
	uint8_t eepDampAdjust;
    /**
	 * Read only variable.
	 * Ptr to a ProgMem location definable by the user.
	 */
	uint8_t eepFlashPtr_l;
	uint8_t eepFlashPtr_h;
    /**
	 * Read only variable.
	 * TBD
	 */
	uint8_t eepDevIdVarAdr_l;
	uint8_t eepDevIdVarAdr_h;
    /**
     * Read only variable.
     * Address to the variable detailing the enhanced mode flag.
     */
    uint8_t eepEnhancedModeVarAdr_l;
    uint8_t eepEnhancedModeVarAdr_h;
	/**
	 * Read only variable.
	 * Address to the variable detailing the used Key ID.
	 */
	uint8_t eepKeyIdSelVarAdr_l;
    uint8_t eepKeyIdSelVarAdr_h;
	/**
	 * Read only variable.
	 * Address to the common key 2. common sub key 2 has an offset of 16 bytes.
	 */
	uint8_t eepComKeyAddr1_l;
    uint8_t eepComKeyAddr1_h;
	/**
	 * Read only variable.
	 * Address to the common key 2. common sub key 2 has an offset of 16 bytes.
	 */
	uint8_t eepComKeyAddr2_l;
    uint8_t eepComKeyAddr2_h;
	/**
	 * Read only variable.
	 * Address to the common key 3. common sub key 3 has an offset of 16 bytes.
	 */
	uint8_t eepComKeyAddr3_l;
    uint8_t eepComKeyAddr3_h;
	/**
	 * Read only variable.
	 * Address to the secret key 0 - 15. secret sub key 0-15 has an offset of 16 bytes
	 * related to the linked secret key! keyL,keyH
	 */
	uint8_t eepSecKeyAddrA[32];
	/**
	 * Read only variable.
	 * Address to the secret key 0 - 15. secret sub key 0-15 has an offset of 16 bytes
	 * related to the linked secret key! keyL,keyH
	 */
	uint8_t eepSecKeyAddrB[32];
    /**
	 * Read only variable.
	 * Ptr to the SRC/FRC calibration configuration location definable by the user.
	 */
    uint8_t eepCalibConfPtr_l;
    uint8_t eepCalibConfPtr_h;
    /**
	 * Read only variable.
	 * Ptr to the Rf Tx service 0 location definable by the user.
	 */
    uint8_t eepRfTxSer0Ptr_l;
    uint8_t eepRfTxSer0Ptr_h;
    /**
	 * Read only variable.
	 * Ptr to the Rf Tx service 1 location definable by the user.
	 */
    uint8_t eepRfTxSer1Ptr_l;
    uint8_t eepRfTxSer1Ptr_h;
    /**
	 * Read only variable.
	 * Setting needed to have traceability GUI<=> EEProm settings.
	 * Versioning etc...
	 */
	uint8_t eepGuiRef[8];
	/**
	 * Read only variable.
	 * Free to be used for any customer purpose.
	 */
	uint8_t eepCustomerSpecific[8];
}
sCustomerEEPromSection;

/*----------------------------------------------------------------------------- */
/** \brief <b>eEepErrorCode</b>
    defines the EEPROM access error codes returned to the Application SW.
 */
/*----------------------------------------------------------------------------- */
typedef enum {

  EEC_NO_ERROR = 0,
  EEC_ADDR_LOCKED_FOR_READING = 1,
  EEC_ADDR_LOCKED_FOR_WRITING = 2,
  EEC_ADDR_OUT_OF_RANGE = 3,
  EEC_ERROR_CORRECTION_OCCURED = 4,
  EEC_ACCESS_FUSE_WR_FAIL = 5,
  EEC_SECTION_INVALID = 6,
  EEC_NOT_USED = 7

} eEepErrorCode;


/*===========================================================================*/
/*  EXTERNAL PROTOTYPES                                                      */
/*===========================================================================*/

extern __root uint8_t ATA_eepFuseRead_C(uint8_t bCtrl);
extern __root eEepErrorCode ATA_eepFuseWrite_C(uint8_t bCtrl, uint8_t bVal);
extern __root eEepErrorCode ATA_eepEEPaccessRightsChange_C(uint8_t bSectionID, uint8_t bVal);
extern __root eEepErrorCode  ATA_eepFusesFix_C(uint8_t bVal);
extern __root eEepErrorCode ATA_eepReadBytes_C(uint8_t* pDes, uint16_t wSrc, uint8_t bCount);
extern __root eEepErrorCode ATA_eepWriteBytes_C(uint8_t* pSrc, uint16_t wDes, uint8_t bCount);


extern sAtmelEEPromSection g_sAtmelEEPromSection;
extern sCustomerEEPromSection g_sCustomerEEPromSection;

#elif defined __IAR_SYSTEMS_ASM__
/*startSimExtraction*/
ATMELSEC_ChipId                     EQU  0
ATMELSEC_XROWFUSE                   EQU  ATMELSEC_ChipId+3
ATMELSEC_MRCCAL                     EQU  ATMELSEC_XROWFUSE+1
ATMELSEC_FRCCAL                     EQU  ATMELSEC_MRCCAL+1
ATMELSEC_RCTCAL1                    EQU  ATMELSEC_FRCCAL+1
ATMELSEC_SUPCAL                     EQU  ATMELSEC_RCTCAL1+1
ATMELSEC_MEMCAL                     EQU  ATMELSEC_SUPCAL+10
ATMELSEC_DEVID1DW                   EQU  ATMELSEC_MEMCAL+3
ATMELSEC_TPCAL                      EQU  ATMELSEC_DEVID1DW+1
ATMELSEC_CalRdy                     EQU  ATMELSEC_TPCAL+10
ATMELSEC_Traceability               EQU  ATMELSEC_CalRdy+1
ATMELSEC_DeviceInfo                 EQU  ATMELSEC_Traceability+9
ATMELSEC_SPARE_ONE                  EQU  ATMELSEC_DeviceInfo+4
ATMELSEC_SRCTCAL                    EQU  ATMELSEC_SPARE_ONE+1
ATMELSEC_SRCCAL                     EQU  ATMELSEC_SRCTCAL+1
ATMELSEC_UID                        EQU  ATMELSEC_SRCCAL+1
ATMELSEC_RSSI_RNG_COR               EQU  ATMELSEC_UID+4
ATMELSEC_RSSI_PRE_DIV               EQU  ATMELSEC_RSSI_RNG_COR+9
ATMELSEC_RSSI_NAMELESS              EQU  ATMELSEC_RSSI_PRE_DIV + 1
ATMELSEC_SPARE_TWO                  EQU  ATMELSEC_RSSI_NAMELESS+1
ATMELSEC_LFCALR                     EQU  ATMELSEC_SPARE_TWO+1
ATMELSEC_TPCALR11                   EQU  ATMELSEC_LFCALR+53
ATMELSEC_TPCALR12                   EQU  ATMELSEC_TPCALR11+1
ATMELSEC_TPCALR13                   EQU  ATMELSEC_TPCALR12+1
ATMELSEC_LFDSR                      EQU  ATMELSEC_TPCALR13+1
ATMELSEC_FEBT                       EQU  ATMELSEC_LFDSR+11
ATMELSEC_FETN4                      EQU  ATMELSEC_FEBT+1
ATMELSEC_FEVCO                      EQU  ATMELSEC_FETN4+1
ATMELSEC_SPARE_THREE                EQU  ATMELSEC_FEVCO+1
ATMELSEC_EEPCRC16_H                 EQU  ATMELSEC_SPARE_THREE+120
ATMELSEC_EEPCRC16_L                 EQU  ATMELSEC_EEPCRC16_H+1


CUSTOMSEC_TpConfig                  EQU  0
CUSTOMSEC_TPCR1                     EQU  CUSTOMSEC_TpConfig+1
CUSTOMSEC_TPCR2                     EQU  CUSTOMSEC_TPCR1+1
CUSTOMSEC_TPCR5                     EQU  CUSTOMSEC_TPCR2+1
CUSTOMSEC_CalLen                    EQU  CUSTOMSEC_TPCR5+1
CUSTOMSEC_ResLen                    EQU  CUSTOMSEC_CalLen+1
CUSTOMSEC_WDTCR                     EQU  CUSTOMSEC_ResLen+1
CUSTOMSEC_VMCR                      EQU  CUSTOMSEC_WDTCR+1
CUSTOMSEC_ResponseDelay             EQU  CUSTOMSEC_VMCR+1
CUSTOMSEC_TPDCR1                    EQU  CUSTOMSEC_ResponseDelay+1
CUSTOMSEC_TPDCR2                    EQU  CUSTOMSEC_TPDCR1+1
CUSTOMSEC_TPDCR3                    EQU  CUSTOMSEC_TPDCR2+1
CUSTOMSEC_TPDCR4                    EQU  CUSTOMSEC_TPDCR3+1
CUSTOMSEC_TPDCR5                    EQU  CUSTOMSEC_TPDCR4+1
CUSTOMSEC_CRC4startVal              EQU  CUSTOMSEC_TPDCR5+1
CUSTOMSEC_CRC4poly                  EQU  CUSTOMSEC_CRC4startVal+1
CUSTOMSEC_CRC8startVal              EQU  CUSTOMSEC_CRC4poly+1
CUSTOMSEC_CRC8poly                  EQU  CUSTOMSEC_CRC8startVal+1
CUSTOMSEC_TPsyncLen                 EQU  CUSTOMSEC_CRC8poly+1
CUSTOMSEC_TPpreaLen                 EQU  CUSTOMSEC_TPsyncLen+1
CUSTOMSEC_TPsync                    EQU  CUSTOMSEC_TPpreaLen+1
CUSTOMSEC_TPECMR                    EQU  CUSTOMSEC_TPsync+4
CUSTOMSEC_SysClocking               EQU  CUSTOMSEC_TPECMR+1
CUSTOMSEC_TCalcDelay                EQU  CUSTOMSEC_SysClocking+1
CUSTOMSEC_DampAdjust                EQU  CUSTOMSEC_TCalcDelay+1
CUSTOMSEC_FlashPtrLow               EQU  CUSTOMSEC_DampAdjust+1
CUSTOMSEC_FlashPtrHigh              EQU  CUSTOMSEC_FlashPtrLow+1
CUSTOMSEC_DevIdVarAdrLow            EQU  CUSTOMSEC_FlashPtrHigh+1
CUSTOMSEC_DevIdVarAdrHigh           EQU  CUSTOMSEC_DevIdVarAdrLow+1
CUSTOMSEC_EnhancedModeVarAdrL       EQU  CUSTOMSEC_DevIdVarAdrHigh+1
CUSTOMSEC_EnhancedModeVarAdrH       EQU  CUSTOMSEC_EnhancedModeVarAdrL+1
CUSTOMSEC_KeyIdVarAdrL              EQU  CUSTOMSEC_EnhancedModeVarAdrH+1
CUSTOMSEC_KeyIdVarAdrH              EQU  CUSTOMSEC_KeyIdVarAdrL+1
CUSTOMSEC_ComKey1AddrL              EQU  CUSTOMSEC_KeyIdVarAdrH+1
CUSTOMSEC_ComKey1AddrH              EQU  CUSTOMSEC_ComKey1AddrL+1
CUSTOMSEC_ComKey2AddrL              EQU  CUSTOMSEC_ComKey1AddrH+1
CUSTOMSEC_ComKey2AddrH              EQU  CUSTOMSEC_ComKey2AddrL+1
CUSTOMSEC_ComKey3AddrL              EQU  CUSTOMSEC_ComKey2AddrH+1
CUSTOMSEC_ComKey3AddrH              EQU  CUSTOMSEC_ComKey3AddrL+1
CUSTOMSEC_SecKeyAddrA               EQU  CUSTOMSEC_ComKey3AddrH+1
CUSTOMSEC_SecKeyAddrB               EQU  CUSTOMSEC_SecKeyAddrA+32
CUSTOMSEC_CalibConfAdrLow           EQU  CUSTOMSEC_SecKeyAddrB+32
CUSTOMSEC_CalibConfAdrHigh          EQU  CUSTOMSEC_CalibConfAdrLow+1
CUSTOMSEC_RfTxSer0AdrLow            EQU  CUSTOMSEC_CalibConfAdrHigh+1
CUSTOMSEC_RfTxSer0AdrHigh           EQU  CUSTOMSEC_RfTxSer0AdrLow+1
CUSTOMSEC_RfTxSer1AdrLow            EQU  CUSTOMSEC_RfTxSer0AdrHigh+1
CUSTOMSEC_RfTxSer1AdrHigh           EQU  CUSTOMSEC_RfTxSer1AdrLow+1
CUSTOMSEC_GuiRef                    EQU  CUSTOMSEC_RfTxSer1AdrHigh+1

/* sEEPromConfigValid */
EEP_CONFIGVALID_CONFINITFLAGS       EQU 0x00
EEP_CONFIGVALID_CONFVAL             EQU EEP_CONFIGVALID_CONFINITFLAGS + 1
EEP_CONFIGVALID_CONFVMCSR           EQU EEP_CONFIGVALID_CONFVAL + 1
EEP_CONFIGVALID_FUNCPTR             EQU EEP_CONFIGVALID_CONFVMCSR + 1

/* sEEPromIOConfig */
EEP_IOCONFIG_CONF_DDRB              EQU 0x00
EEP_IOCONFIG_CONF_DDRC              EQU EEP_IOCONFIG_CONF_DDRB + 1
EEP_IOCONFIG_CONF_MCUCR             EQU EEP_IOCONFIG_CONF_DDRC + 1
EEP_IOCONFIG_CONF_PCICR             EQU EEP_IOCONFIG_CONF_MCUCR + 1
EEP_IOCONFIG_CONF_PCMSK0            EQU EEP_IOCONFIG_CONF_PCICR + 1
EEP_IOCONFIG_CONF_PCMSK1            EQU EEP_IOCONFIG_CONF_PCMSK0 + 1
EEP_IOCONFIG_CONF_PORTB             EQU EEP_IOCONFIG_CONF_PCMSK1 + 1
EEP_IOCONFIG_CONF_PORTC             EQU EEP_IOCONFIG_CONF_PORTB + 1

/* sEEPromTrxConfig */
EEP_TRXCONFIG_CLKCONFIG             EQU 0x00
EEP_TRXCONFIG_SPICONFIG             EQU EEP_TRXCONFIG_CLKCONFIG + 1
EEP_TRXCONFIG_SYSCONFIG             EQU EEP_TRXCONFIG_SPICONFIG + 1
EEP_TRXCONFIG_CLKOUTCONFIG          EQU EEP_TRXCONFIG_SYSCONFIG + 1
EEP_TRXCONFIG_SYSSTARTCONF          EQU EEP_TRXCONFIG_CLKOUTCONFIG + 1
EEP_TRXCONFIG_SYSSTARTSVCCHCONFIG   EQU EEP_TRXCONFIG_SYSSTARTCONF + 1
EEP_TRXCONFIG_SVCINITCONF           EQU EEP_TRXCONFIG_SYSSTARTSVCCHCONFIG + 1

EEP_TRXCONF_SYSCONFIG_AVCCCONFIG_LOWBATT_DISABLE    EQU BIT_0
EEP_TRXCONF_SYSCONFIG_AVCCCONFIG_AVCCLOW_DISABLE    EQU BIT_1
EEP_TRXCONF_SYSCONFIG_DFIFO_OFL_UFL_RX_DISABLE      EQU BIT_2
EEP_TRXCONF_SYSCONFIG_SFIFO_OFL_UFL_RX_DISABLE      EQU BIT_3
EEP_TRXCONF_SYSCONFIG_VS5V                          EQU BIT_4
EEP_TRXCONF_SYSCONFIG_VS22V                         EQU BIT_5

BM_EEP_TRXCONF_SYSCONFIG_AVCCCONFIG_LOWBATT_DISABLE EQU BIT_MASK_0
BM_EEP_TRXCONF_SYSCONFIG_AVCCCONFIG_AVCCLOW_DISABLE EQU BIT_MASK_1
BM_EEP_TRXCONF_SYSCONFIG_DFIFO_OFL_UFL_RX_DISABLE   EQU BIT_MASK_2
BM_EEP_TRXCONF_SYSCONFIG_SFIFO_OFL_UFL_RX_DISABLE   EQU BIT_MASK_3
BM_EEP_TRXCONF_SYSCONFIG_VS5V                       EQU BIT_MASK_4
BM_EEP_TRXCONF_SYSCONFIG_VS22V                      EQU BIT_MASK_5

/* sEEPromWDTimer */
EEP_WDTIMER_CONFT0CR                EQU 0x00
EEP_WDTIMER_CONFT0IFR               EQU 0x01
EEP_WDTIMER_CONFWDTCR               EQU 0x02

/* sEEPromEventConfig */
EEP_EVENTCONFIG_PINEVENTCONF        EQU 0x00
EEP_EVENTCONFIG_SYSEVENTCONF        EQU EEP_EVENTCONFIG_PINEVENTCONF + 1
EEP_EVENTCONFIG_CMDRDYCONF          EQU EEP_EVENTCONFIG_SYSEVENTCONF + 1

/* sEEPromSleepMode */
EEP_SLEEPMODE_SLEEPMODECONF         EQU 0x00

/* sEEPromDebug */
EEP_DEBUG_DBGSW                     EQU 0x00

/* sEEPromVsPa */
EEP_VSPA_SUPCA1                     EQU 0x00

/* EEPROM error codes */
EEC_NO_ERROR                        EQU 0x00
EEC_ADDR_LOCKED_FOR_READING         EQU 0x01
EEC_ADDR_LOCKED_FOR_WRITING         EQU 0x02
EEC_ADDR_OUT_OF_RANGE               EQU 0x03
EEC_ERROR_CORRECTION_OCCURED        EQU 0x04
EEC_ACCESS_FUSE_WR_FAIL             EQU 0x05
EEC_SECTION_INVALID                 EQU 0x06
EEC_NOT_USED                        EQU 0x07

/*stopSimExtraction*/
#endif

#endif /* EEP_H */
