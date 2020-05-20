/* Created by plibgen $Revision: 1.31 $ */

#ifndef _EBI_P32MZ2048EFM100_H
#define _EBI_P32MZ2048EFM100_H

/* Section 1 - Enumerate instances, define constants, VREGs */

#include <xc.h>
#include <stdbool.h>

#include "peripheral/peripheral_common_32bit.h"

/* Default definition used for all API dispatch functions */
#ifndef PLIB_INLINE_API
    #define PLIB_INLINE_API extern inline
#endif

/* Default definition used for all other functions */
#ifndef PLIB_INLINE
    #define PLIB_INLINE extern inline
#endif

typedef enum {

    EBI_ID_0 = _EBI_BASE_ADDRESS,
    EBI_NUMBER_OF_MODULES = 1

} EBI_MODULE_ID;

typedef enum {

    CS_TIMING_0 = 0x00,
    CS_TIMING_1 = 0x01,
    CS_TIMING_2 = 0x02

} EBI_CS_TIMING;

typedef enum {

    SRAM = 0x01,
    NORFLASH = 0x02

} EBI_MEMORY_TYPE;

typedef enum {

    CSNOTUSED = 0x00,
    MEMORY_SIZE_64KB = 0x01,
    MEMORY_SIZE_128KB = 0x02,
    MEMORY_SIZE_256KB = 0x03,
    MEMORY_SIZE_512KB = 0x04,
    MEMORY_SIZE_1MB = 0x05,
    MEMORY_SIZE_2MB = 0x06,
    MEMORY_SIZE_4MB = 0x07,
    MEMORY_SIZE_8MB = 0x08,
    MEMORY_SIZE_16MB = 0x09

} EBI_MEMORY_SIZE;

typedef enum {

    PAGE_WORD4 = 0x00,
    PAGE_WORD8 = 0x01,
    PAGE_WORD16 = 0x02,
    PAGE_WORD32 = 0x03

} EBI_PAGE_SIZE;

typedef enum {

    MEMORY_WIDTH_8BIT = 0x04,
    MEMORY_WIDTH_16BIT = 0x00

} EBI_STATIC_MEMORY_WIDTH;

typedef enum {

    EBI_EBIADDR_PIN0 = 0x01,
    EBI_EBIADDR_PIN1 = 0x03,
    EBI_EBIADDR_PIN2 = 0x07,
    EBI_EBIADDR_PIN3 = 0x0F,
    EBI_EBIADDR_PIN4 = 0x1F,
    EBI_EBIADDR_PIN5 = 0x3F,
    EBI_EBIADDR_PIN6 = 0x7F,
    EBI_EBIADDR_PIN7 = 0xFF,
    EBI_EBIADDR_PIN8 = 0x1FF,
    EBI_EBIADDR_PIN9 = 0x3FF,
    EBI_EBIADDR_PIN10 = 0x7FF,
    EBI_EBIADDR_PIN11 = 0xFFF,
    EBI_EBIADDR_PIN12 = 0x1FFF,
    EBI_EBIADDR_PIN13 = 0x3FFF,
    EBI_EBIADDR_PIN14 = 0x7FFF,
    EBI_EBIADDR_PIN15 = 0xFFFF,
    EBI_EBIADDR_PIN16 = 0x1FFFF,
    EBI_EBIADDR_PIN17 = 0x3FFFF,
    EBI_EBIADDR_PIN18 = 0x7FFFF,
    EBI_EBIADDR_PIN19 = 0xFFFFF,
    EBI_ADDR_PRESET8 = 0xFF,
    EBI_ADDR_PRESET12 = 0xFFF,
    EBI_ADDR_PRESET16 = 0xFFFF,
    EBI_ADDR_PRESET_ALL = 0xFFFFF

} EBI_ADDRESS_PORT;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/ebi_Base_Address_Default.h"
#include "../templates/ebi_MemoryCharacteristics_Default.h"
#include "../templates/ebi_MemoryTimingConfig_Default.h"
#include "../templates/ebi_ReadyMode_Default.h"
#include "../templates/ebi_MemoryPaging_Default.h"
#include "../templates/ebi_PageMode_Default.h"
#include "../templates/ebi_PageReadTime_Default.h"
#include "../templates/ebi_DataTurnAroundTime_Default.h"
#include "../templates/ebi_WritePulseWidth_Default.h"
#include "../templates/ebi_AddressHoldTime_Default.h"
#include "../templates/ebi_AddressSetupTime_Default.h"
#include "../templates/ebi_ReadCycleTime_Default.h"
#include "../templates/ebi_FlashTiming_Default.h"
#include "../templates/ebi_StaticMemoryWidthRegister_Default.h"
#include "../templates/ebi_FlashPowerDownMode_Default.h"
#include "../templates/ebi_ControlEnable_Default.h"
#include "../templates/ebi_AddressPinEnableBits_Default.h"
#include "../templates/ebi_ReadyPin3Config_Default.h"
#include "../templates/ebi_ReadyPin2Config_Default.h"
#include "../templates/ebi_ReadyPin1Config_Default.h"
#include "../templates/ebi_ReadyPinSens_Default.h"
#include "../templates/ebi_FlashResetPin_Unsupported.h"
#include "../templates/ebi_WriteOutputControl_Default.h"
#include "../templates/ebi_ByteSelectPin_Unsupported.h"
#include "../templates/ebi_ChipSelectEnable_EBIP32_100_124pin.h"
#include "../templates/ebi_DataEnable_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void PLIB_EBI_BaseAddressSet(EBI_MODULE_ID index, int ChipSelectNumber, uint32_t BaseAddress)
{
     EBI_BaseAddressSet_Default(index, ChipSelectNumber, BaseAddress);
}

PLIB_INLINE_API uint32_t PLIB_EBI_BaseAddressGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return EBI_BaseAddressGet_Default(index, ChipSelectNumber);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsBaseAddress(EBI_MODULE_ID index)
{
     return EBI_ExistsBaseAddress_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_MemoryCharacteristicsSet(EBI_MODULE_ID index, int ChipSelectNumber, EBI_MEMORY_TYPE MemoryType, EBI_MEMORY_SIZE MemorySize, EBI_CS_TIMING TimingReg)
{
     EBI_MemoryCharacteristicsSet_Default(index, ChipSelectNumber, MemoryType, MemorySize, TimingReg);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsMemoryCharacteristics(EBI_MODULE_ID index)
{
     return EBI_ExistsMemoryCharacteristics_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_MemoryTimingConfigSet(EBI_MODULE_ID index, int CS_Timing_Reg, int PageReadTime, int DataTurnAroundTime, int WritePulseWidth, int AddressHoldTime, int AddressSetupTime, int ReadCycleTime)
{
     EBI_MemoryTimingConfigSet_Default(index, CS_Timing_Reg, PageReadTime, DataTurnAroundTime, WritePulseWidth, AddressHoldTime, AddressSetupTime, ReadCycleTime);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsMemoryTimingConfig(EBI_MODULE_ID index)
{
     return EBI_ExistsMemoryTimingConfig_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_ReadyModeSet(EBI_MODULE_ID index, bool ReadyPin0, bool ReadyPin1, bool ReadyPin2)
{
     EBI_ReadyModeSet_Default(index, ReadyPin0, ReadyPin1, ReadyPin2);
}

PLIB_INLINE_API bool PLIB_EBI_ReadyModeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return EBI_ReadyModeGet_Default(index, ChipSelectNumber);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsReadyMode(EBI_MODULE_ID index)
{
     return EBI_ExistsReadyMode_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_MemoryPagingSet(EBI_MODULE_ID index, int ChipSelectNumber, bool PageMode, EBI_PAGE_SIZE MemoryPageSize)
{
     EBI_MemoryPagingSet_Default(index, ChipSelectNumber, PageMode, MemoryPageSize);
}

PLIB_INLINE_API EBI_PAGE_SIZE PLIB_EBI_MemoryPageSizeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return EBI_MemoryPageSizeGet_Default(index, ChipSelectNumber);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsMemoryPaging(EBI_MODULE_ID index)
{
     return EBI_ExistsMemoryPaging_Default(index);
}

PLIB_INLINE_API bool PLIB_EBI_PageModeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return EBI_PageModeGet_Default(index, ChipSelectNumber);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsPageMode(EBI_MODULE_ID index)
{
     return EBI_ExistsPageMode_Default(index);
}

PLIB_INLINE_API int PLIB_EBI_PageReadCycleTimeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return EBI_PageReadCycleTimeGet_Default(index, ChipSelectNumber);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsPageReadTime(EBI_MODULE_ID index)
{
     return EBI_ExistsPageReadTime_Default(index);
}

PLIB_INLINE_API int PLIB_EBI_DataTurnAroundTimeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return EBI_DataTurnAroundTimeGet_Default(index, ChipSelectNumber);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsDataTurnAroundTime(EBI_MODULE_ID index)
{
     return EBI_ExistsDataTurnAroundTime_Default(index);
}

PLIB_INLINE_API int PLIB_EBI_WritePulseWidthGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return EBI_WritePulseWidthGet_Default(index, ChipSelectNumber);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsWritePulseWidth(EBI_MODULE_ID index)
{
     return EBI_ExistsWritePulseWidth_Default(index);
}

PLIB_INLINE_API int PLIB_EBI_AddressHoldTimeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return EBI_AddressHoldTimeGet_Default(index, ChipSelectNumber);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsAddressHoldTime(EBI_MODULE_ID index)
{
     return EBI_ExistsAddressHoldTime_Default(index);
}

PLIB_INLINE_API int PLIB_EBI_AddressSetupTimeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return EBI_AddressSetupTimeGet_Default(index, ChipSelectNumber);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsAddressSetupTime(EBI_MODULE_ID index)
{
     return EBI_ExistsAddressSetupTime_Default(index);
}

PLIB_INLINE_API int PLIB_EBI_ReadCycleTimeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return EBI_ReadCycleTimeGet_Default(index, ChipSelectNumber);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsReadCycleTime(EBI_MODULE_ID index)
{
     return EBI_ExistsReadCycleTime_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_FlashTimingSet(EBI_MODULE_ID index, int FlashTiming)
{
     EBI_FlashTimingSet_Default(index, FlashTiming);
}

PLIB_INLINE_API int PLIB_EBI_FlashTimingGet(EBI_MODULE_ID index)
{
     return EBI_FlashTimingGet_Default(index);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsFlashTiming(EBI_MODULE_ID index)
{
     return EBI_ExistsFlashTiming_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_StaticMemoryWidthRegisterSet(EBI_MODULE_ID index, int RegisterNumber, EBI_STATIC_MEMORY_WIDTH StaticMemoryWidthRegister)
{
     EBI_StaticMemoryWidthRegisterSet_Default(index, RegisterNumber, StaticMemoryWidthRegister);
}

PLIB_INLINE_API EBI_STATIC_MEMORY_WIDTH PLIB_EBI_StaticMemoryWidthRegisterGet(EBI_MODULE_ID index, int RegisterNumber)
{
     return EBI_StaticMemoryWidthRegisterGet_Default(index, RegisterNumber);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsStaticMemoryWidthRegister(EBI_MODULE_ID index)
{
     return EBI_ExistsStaticMemoryWidthRegister_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_FlashPowerDownModeSet(EBI_MODULE_ID index, bool FlashPowerDownMode)
{
     EBI_FlashPowerDownModeSet_Default(index, FlashPowerDownMode);
}

PLIB_INLINE_API bool PLIB_EBI_FlashPowerDownModeGet(EBI_MODULE_ID index)
{
     return EBI_FlashPowerDownModeGet_Default(index);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsFlashPowerDownMode(EBI_MODULE_ID index)
{
     return EBI_ExistsFlashPowerDownMode_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_ControlEnableSet(EBI_MODULE_ID index, bool EnableBit)
{
     EBI_ControlEnableSet_Default(index, EnableBit);
}

PLIB_INLINE_API bool PLIB_EBI_ControlEnableGet(EBI_MODULE_ID index)
{
     return EBI_ControlEnableGet_Default(index);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsControlEnable(EBI_MODULE_ID index)
{
     return EBI_ExistsControlEnable_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_AddressPinEnableBitsSet(EBI_MODULE_ID index, EBI_ADDRESS_PORT AddressPort)
{
     EBI_AddressPinEnableBitsSet_Default(index, AddressPort);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsAddressPinEnableBits(EBI_MODULE_ID index)
{
     return EBI_ExistsAddressPinEnableBits_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_ReadyPin3ConfigSet(EBI_MODULE_ID index, bool ReadyPin3Enable, bool ReadyPin3Invert)
{
     EBI_ReadyPin3ConfigSet_Default(index, ReadyPin3Enable, ReadyPin3Invert);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsReadyPin3Config(EBI_MODULE_ID index)
{
     return EBI_ExistsReadyPin3Config_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_ReadyPin2ConfigSet(EBI_MODULE_ID index, bool ReadyPin2Enable, bool ReadyPin2Invert)
{
     EBI_ReadyPin2ConfigSet_Default(index, ReadyPin2Enable, ReadyPin2Invert);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsReadyPin2Config(EBI_MODULE_ID index)
{
     return EBI_ExistsReadyPin2Config_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_ReadyPin1ConfigSet(EBI_MODULE_ID index, bool ReadyPin1Enable, bool ReadyPin1Invert)
{
     EBI_ReadyPin1ConfigSet_Default(index, ReadyPin1Enable, ReadyPin1Invert);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsReadyPin1Config(EBI_MODULE_ID index)
{
     return EBI_ExistsReadyPin1Config_Default(index);
}

PLIB_INLINE_API void PLIB_EBI_ReadyPinSensSet(EBI_MODULE_ID index, bool SensitivityControl)
{
     EBI_ReadyPinSensSet_Default(index, SensitivityControl);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsReadyPinSens(EBI_MODULE_ID index)
{
     return EBI_ExistsReadyPinSens_Default(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_FlashResetPinSet(EBI_MODULE_ID index, bool FlashResetPin)
{
     EBI_FlashResetPinSet_Unsupported(index, FlashResetPin);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_EBI_FlashResetPinGet(EBI_MODULE_ID index)
{
     return EBI_FlashResetPinGet_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsFlashResetPin(EBI_MODULE_ID index)
{
     return EBI_ExistsFlashResetPin_Unsupported(index);
}

PLIB_INLINE_API void PLIB_EBI_WriteOutputControlSet(EBI_MODULE_ID index, bool WriteEnable, bool OutputEnable)
{
     EBI_WriteOutputControlSet_Default(index, WriteEnable, OutputEnable);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsWriteOutputControl(EBI_MODULE_ID index)
{
     return EBI_ExistsWriteOutputControl_Default(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_ByteSelectPinSet(EBI_MODULE_ID index, bool ByteSelect0, bool ByteSelect1)
{
     EBI_ByteSelectPinSet_Unsupported(index, ByteSelect0, ByteSelect1);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsByteSelectPin(EBI_MODULE_ID index)
{
     return EBI_ExistsByteSelectPin_Unsupported(index);
}

PLIB_INLINE_API void PLIB_EBI_ChipSelectEnableSet(EBI_MODULE_ID index, bool ChipSelect0, bool ChipSelect1, bool ChipSelect2, bool ChipSelect3)
{
     EBI_ChipSelectEnableSet_EBIP32_100_124pin(index, ChipSelect0, ChipSelect1, ChipSelect2, ChipSelect3);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsChipSelectEnable(EBI_MODULE_ID index)
{
     return EBI_ExistsChipSelectEnable_EBIP32_100_124pin(index);
}

PLIB_INLINE_API void PLIB_EBI_DataEnableSet(EBI_MODULE_ID index, bool DataUpperByte, bool DataLowerByte)
{
     EBI_DataEnableSet_Default(index, DataUpperByte, DataLowerByte);
}

PLIB_INLINE_API bool PLIB_EBI_ExistsDataEnable(EBI_MODULE_ID index)
{
     return EBI_ExistsDataEnable_Default(index);
}

#endif
