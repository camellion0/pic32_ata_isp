/**************************************************************
 **             - INTRINSICS.H -
 **
 **     Intrinsics for iccAVR
 **
 **     Used with iccAVR.
 **
 **     Copyright 1999-2005 IAR Systems AB. All rights reserved.
 **
 **     File version: $Revision: 328482 $
 **
 **************************************************************/

#ifdef  __IAR_SYSTEMS_ICC__
#ifndef _SYSTEM_BUILD
#pragma system_include /* treat file as system include file for MISRA check */
#endif
#endif

#ifndef __INTRINSICS_H
#define __INTRINSICS_H

#ifndef __ICCAVR__
#error This file should only be compiled with iccAVR
#endif /* __ICCAVR__ */

__intrinsic void __no_operation(void);
__intrinsic void __enable_interrupt(void);
__intrinsic void __disable_interrupt(void);
__intrinsic void __sleep(void);
__intrinsic void __watchdog_reset(void);
#define __clear_watchdog_timer()  __watchdog_reset()

#pragma language=save
#pragma language=extended

#ifdef __TINY_AVR__
#else
__intrinsic unsigned char __load_program_memory(const unsigned char __flash *);
#ifdef __HAS_ELPM__
__intrinsic unsigned char __extended_load_program_memory(
                                             const unsigned char __farflash *);
#endif
#endif

#pragma language=restore

__intrinsic void __insert_opcode(unsigned short op);

#if __MEMORY_MODEL__ == 4
#if __CPU__ < 2
#define __STR_MATTR__ __flash
#else
#define __STR_MATTR__ __hugeflash
#endif
#else
#define __STR_MATTR__
#endif


__intrinsic void __require(void *);

__intrinsic void __delay_cycles(unsigned long);

__intrinsic unsigned char __save_interrupt(void);
#define __get_interrupt_state() __save_interrupt()

__intrinsic void          __restore_interrupt(unsigned char);
#define __set_interrupt_state(STATE) __restore_interrupt(STATE)
typedef unsigned char __istate_t;

__intrinsic unsigned char __swap_nibbles(unsigned char);

__intrinsic void          __indirect_jump_to(unsigned long);

#ifdef __HAS_ENHANCED_CORE__

#ifdef __HAS_MUL__
__intrinsic unsigned int  __multiply_unsigned(unsigned char, unsigned char);
__intrinsic signed int    __multiply_signed(signed char, signed char);
__intrinsic signed int    __multiply_signed_with_unsigned(signed char, unsigned char);

__intrinsic unsigned int  __fractional_multiply_unsigned(unsigned char, unsigned char);
__intrinsic signed int    __fractional_multiply_signed(signed char, signed char);
__intrinsic signed int    __fractional_multiply_signed_with_unsigned(signed char, signed char);
#endif

#pragma language=save
#pragma language=extended

#ifdef __XMEGA_CORE__
/* SPM */

/*
  __ResetNvmCmd, pseudo code.
  LDI  R16,0
  STS  NVM_CMD,R16
*/
__intrinsic void __ResetNvmCmd(void);

/*
  The following operations use LPM-based commands:
  Reading calibration bytes
  Reading user-accessible signature bytes
  Read Flash memory
*/

/*
  __AddrToZByteToNvmCmd_LPM, pseudo code.
  MOVW  R31:R30,addr
  STS   NVM_CMD,byte
  LPM
*/
__intrinsic unsigned char __AddrToZByteToNvmCmd_LPM(void __flash* addr, 
                                                    unsigned char byte);
#ifdef __HAS_ELPM__
/*
  __AddrToZ24ByteToNvmCmd_ELPM, pseudo code.
  MOVW  R31:R30,LWRD(addr)
  OUT   RAMPZ,(addr>>16)
  STS   NVM_CMD,byte
  ELPM
*/
__intrinsic unsigned char __AddrToZ24ByteToNvmCmd_ELPM(void __farflash* addr, 
                                                       unsigned char byte);
#endif


/*
  The following operations use SPM-based commands:
  Erasing user signature row
  Writing user signature row
  Erasing Flash memory
  Loading Flash page buffer
  Writing Flash memory
  Flushing Flash page buffer
*/

/*
  __DataToR0Byte0ToNvmCmdByte1ToCCP_SPM, pseudo code.
  MOV   R0,data
  STS   NVM_CMD,byte0
  OUT   CCP,byte1
  SPM
*/
__intrinsic void __DataToR0Byte0ToNvmCmdByte1ToCCP_SPM(unsigned char data, 
                                                       unsigned char byte0, 
                                                       unsigned char byte1);
/*
  __AddrToZByte0ToNvmCmdByte1ToCCP_SPM, pseudo code.
  MOVW  R31:R30,addr
  STS   NVM_CMD,byte0
  OUT   CCP,byte1
  SPM
*/
__intrinsic void __AddrToZByte0ToNvmCmdByte1ToCCP_SPM(void __flash* addr, 
                                                      unsigned char byte0, 
                                                      unsigned char byte1);
/*
  __AddrToZWordToR1R0Byte0ToNvmCmdByte1ToCCP_SPM, pseudo code.
  MOVW  R31:R30,addr
  MOVW  R1:R0,word
  STS   NVM_CMD,byte0
  OUT   CCP,byte1
  SPM
*/
__intrinsic void __AddrToZWordToR1R0Byte0ToNvmCmdByte1ToCCP_SPM(void __flash* addr, 
                                                                unsigned short word, 
                                                                unsigned char byte0, 
                                                                unsigned char byte1);
#ifdef __HAS_ELPM__
/*
  __AddrToZ24Byte0ToNvmCmdByte1ToCCP_SPM, pseudo code.
  MOVW  R31:R30,LWRD(addr)
  OUT   RAMPZ,(addr>>16)
  STS   NVM_CMD,byte0
  OUT   CCP,byte1
  SPM
*/
__intrinsic void __AddrToZ24Byte0ToNvmCmdByte1ToCCP_SPM(void __farflash* addr, 
                                                        unsigned char byte0, 
                                                        unsigned char byte1);
/*
  __AddrToZ24WordToR1R0Byte0ToNvmCmdByte1ToCCP_SPM, pseudo code.
  MOVW  R31:R30,LWRD(addr)
  OUT   RAMPZ,(addr>>16)
  MOVW  R1:R0,word
  STS   NVM_CMD,byte0
  OUT   CCP,byte1
  SPM
*/
__intrinsic void __AddrToZ24WordToR1R0Byte0ToNvmCmdByte1ToCCP_SPM(void __farflash* addr, 
                                                                  unsigned short word, 
                                                                  unsigned char byte0, 
                                                                  unsigned char byte1);
#endif

/*
  The following operations use NVM Action-based commands:
  Reading fuse bytes
  Writing lock bits
  Generating CRC for Application Section
  Generating CRC for Boot Section
*/

/*
  __intrinsic void __Byte0ToNvmCmdByte1ToCCPByte2ToNVMCtrlA, pseudo code.
  STS   NVM_CMD,byte0
  OUT   CCP,byte1
  STS   NVM_CTRLA,byte2
*/
__intrinsic void __Byte0ToNvmCmdByte1ToCCPByte2ToNVMCtrlA(unsigned char byte0, 
                                                          unsigned char byte1, 
                                                          unsigned char byte2);
/*
  __intrinsic void __Byte0ToCCPByte1ToNVMCtrlA, pseudo code.
  OUT   CCP,byte1
  STS   NVM_CTRLA,byte2
*/
__intrinsic void __Byte0ToCCPByte1ToNVMCtrlA(unsigned char byte0, 
                                             unsigned char byte1);


#define _SPM_LOCKBITS(Data)  \
  __DataToR0Byte0ToNvmCmdByte1ToCCP_SPM((Data),\
                                        NVM_CMD_WRITE_LOCK_BITS_gc,\
                                        CCP_SPM_gc)

#define _SPM_ERASE(Addr) \
  __AddrToZByte0ToNvmCmdByte1ToCCP_SPM((void __flash*)(Addr),\
                                       NVM_CMD_ERASE_APP_PAGE_gc,\
                                       CCP_SPM_gc)

#define _SPM_FILLTEMP(Addr,Word)  \
  __AddrToZWordToR1R0Byte0ToNvmCmdByte1ToCCP_SPM((void __flash*)(Addr),\
                                                 (Word),\
                                                 NVM_CMD_LOAD_FLASH_BUFFER_gc,\
                                                 CCP_SPM_gc)

#define _SPM_PAGEWRITE(Addr) \
  __AddrToZByte0ToNvmCmdByte1ToCCP_SPM((void __flash*)(Addr),\
                                       NVM_CMD_WRITE_APP_PAGE_gc,\
                                       CCP_SPM_gc)

#ifdef __HAS_ELPM__
#define _SPM_24_ERASE(Addr) \
  __AddrToZ24Byte0ToNvmCmdByte1ToCCP_SPM((void __farflash*)(Addr),\
                                         NVM_CMD_ERASE_APP_PAGE_gc,\
                                         CCP_SPM_gc)

#define _SPM_24_FILLTEMP(Addr,Word)  \
  __AddrToZ24WordToR1R0Byte0ToNvmCmdByte1ToCCP_SPM((void __farflash*)(Addr),\
                                                   (Word),\
                                                   NVM_CMD_LOAD_FLASH_BUFFER_gc,\
                                                   CCP_SPM_gc)

#define _SPM_24_PAGEWRITE(Addr) \
  __AddrToZ24Byte0ToNvmCmdByte1ToCCP_SPM((void __farflash*)(Addr),\
                                         NVM_CMD_WRITE_APP_PAGE_gc,\
                                         CCP_SPM_gc)
#endif

#else /*!__XMEGA_CORE__*/
/* SPM */

/*
  __DataToR0ByteToSPMCR_SPM, pseudo code.
  MOV   R0,data
  OUT   SPMCR,byte
  SPM
*/
__intrinsic void __DataToR0ByteToSPMCR_SPM(unsigned char data, 
                                           unsigned char byte);

/*
  __AddrToZByteToSPMCR_SPM, pseudo code.
  MOVW  R31:R30,addr
  OUT   SPMCR,byte
  SPM
*/
__intrinsic void __AddrToZByteToSPMCR_SPM(void __flash* addr, 
                                          unsigned char byte);

/*
  __AddrToZWordToR1R0ByteToSPMCR_SPM, pseudo code.
  MOVW  R31:R30,addr
  MOVW  R1:R0,word
  OUT   SPMCR,byte
  SPM
*/
__intrinsic void __AddrToZWordToR1R0ByteToSPMCR_SPM(void __flash* addr, 
                                                    unsigned short word, 
                                                    unsigned char byte);

#define _SPM_LOCKBITS(Data)  \
  __DataToR0ByteToSPMCR_SPM((Data), 0x09)

#define _SPM_ERASE(Addr) \
  __AddrToZByteToSPMCR_SPM((void __flash*)(Addr), 0x03)

#define _SPM_FILLTEMP(Addr,Word)  \
  __AddrToZWordToR1R0ByteToSPMCR_SPM((void __flash*)(Addr), (Word), 0x01)

#define _SPM_PAGEWRITE(Addr) \
  __AddrToZByteToSPMCR_SPM((void __flash*)(Addr), (0x05))


/*
  __AddrToZByteToSPMCR_LPM, pseudo code.
  MOVW  R31:R30,addr
  OUT   SPMCR,byte
  LPM
*/
__intrinsic unsigned char __AddrToZByteToSPMCR_LPM(void __flash* addr, 
                                                   unsigned char byte);

#define _SPM_GET_LOCKBITS()  \
  __AddrToZByteToSPMCR_LPM((void __flash*)0x0001, 0x09)

#define _SPM_GET_FUSEBITS()  \
  __AddrToZByteToSPMCR_LPM((void __flash*)0x0000, 0x09)


#ifdef __HAS_ELPM__
/*
  __AddrToZ24ByteToSPMCR_SPM, pseudo code.
  MOVW  R31:R30,LWRD(addr)
  OUT   RAMPZ,(addr>>16)
  OUT   SPMCR,byte
  SPM
*/
__intrinsic void __AddrToZ24ByteToSPMCR_SPM(void __farflash* addr, 
                                            unsigned char byte);
/*
  __AddrToZ24WordToR1R0ByteToSPMCR_SPM, pseudo code.
  MOVW  R31:R30,LWRD(addr)
  OUT   RAMPZ,(addr>>16)
  MOVW  R1:R0,word
  OUT   SPMCR,byte
  SPM
*/
__intrinsic void __AddrToZ24WordToR1R0ByteToSPMCR_SPM(void __farflash* addr, 
                                                      unsigned short word, 
                                                      unsigned char byte);
#define _SPM_24_ERASE(Addr) \
  __AddrToZ24ByteToSPMCR_SPM((void __farflash*)(Addr), 0x03)

#define _SPM_24_FILLTEMP(Addr,Data)  \
  __AddrToZ24WordToR1R0ByteToSPMCR_SPM((void __farflash*)(Addr), (Data), 0x01)

#define _SPM_24_PAGEWRITE(Addr) \
  __AddrToZ24ByteToSPMCR_SPM((void __farflash*)(Addr), (0x05))

/*
  __AddrToZ24ByteToSPMCR_ELPM, pseudo code.
  MOVW  R31:R30,LWRD(addr)
  OUT   RAMPZ,(addr>>16)
  OUT   SPMCR,byte
  ELPM
*/
__intrinsic unsigned char __AddrToZ24ByteToSPMCR_ELPM(void __farflash* addr, 
                                                      unsigned char byte);
#endif /*__HAS_ELPM__*/
#endif /*!__XMEGA_CORE__*/


#ifdef __XMEGA_CORE__
__intrinsic unsigned long long __DES_encryption(unsigned long long data,
                                                unsigned long long key);
__intrinsic unsigned long long __DES_decryption(unsigned long long data,
                                                unsigned long long key);


#ifdef __XMEGA_USB__
__intrinsic unsigned char __xch(unsigned char r, unsigned char* ptr);
__intrinsic unsigned char __las(unsigned char r, unsigned char* ptr);
__intrinsic unsigned char __lac(unsigned char r, unsigned char* ptr);
__intrinsic unsigned char __lat(unsigned char r, unsigned char* ptr);
#endif

__intrinsic unsigned short __io_read_xmega_16(unsigned char volatile* lowPtr,
                                              unsigned char volatile* highPtr);
__intrinsic void          __io_write_xmega_16(unsigned char volatile* lowPtr,
                                              unsigned char volatile* highPtr,
                                              unsigned short value);

#endif

#pragma language=restore

#endif /* __HAS_ENHANCED_CORE__ */

/* Include a file appropriate for the processor used, 
 * that defines EECR, EEAR and EEDR (e.g. io2312.h). */
#ifdef __HAS_EEPROM__
#define __EEPUT(ADR,VAL)  (*((unsigned char __eeprom *)ADR) = VAL)
#define __EEGET(VAR, ADR) (VAR = *((unsigned char __eeprom *)ADR))
#else /* !__HAS_EEPROM__ */
#define __EEPUT(ADR,VAL)  {while (EECR & 0x02) ; \
 EEAR = (ADR); EEDR = (VAL); EECR = 0x04; EECR = 0x02;}
 
#define __EEGET(VAR, ADR) {while (EECR & 0x02) ; \
        EEAR = (ADR); EECR = 0x01; (VAR) = EEDR;}
#endif /* __HAS_EEPROM__ */

/* PORT is a sfrb defined variable */
#define input(PORT) (PORT)
#define output(PORT,VAL) ((PORT)=(VAL))

#define input_block_dec(PORT,ADDRESS,COUNT)\
{ \
  unsigned char i;\
  unsigned char *addr=(ADDRESS);\
  for(i=0;i<(COUNT);i++)\
    *addr--=(PORT);\
}

#define input_block_inc(PORT,ADDRESS,COUNT)\
{ \
  unsigned char i;\
  unsigned char *addr=(ADDRESS);\
  for(i=0;i<(COUNT);i++)\
    *addr++=(PORT);\
}

#define output_block_dec(PORT,ADDRESS,COUNT)\
{ \
  unsigned char i;\
  unsigned char *addr=(ADDRESS);\
  for(i=0;i<(COUNT);i++)\
    (PORT)=*addr--;\
}

#define output_block_inc(PORT,ADDRESS,COUNT)\
{ \
  unsigned char i;\
  unsigned char *addr=(ADDRESS);\
  for(i=0;i<(COUNT);i++)\
    (PORT)=*addr++;\
}


/* Nice to have macros */

#define __out_word(BaseName, value)\
{\
  unsigned char _tH=(value) >>   8;\
  unsigned char _tL=(value) & 0xFF;\
  BaseName ## H = _tH;\
  BaseName ## L = _tL;\
}


#define __out_word_atomic(BaseName, value)\
{\
 unsigned char _t=__save_interrupt();\
 __disable_interrupt();\
 __out_word(BaseName,value);\
 __restore_interrupt(_t);\
}

#define __in_word(BaseName, value)\
{\
 (value) = (BaseName ## L);\
 (value) |= (unsigned short)BaseName ## H << 8;\
}


#define __in_word_atomic(BaseName, value)\
{\
 unsigned char _t=__save_interrupt();\
 __disable_interrupt();\
 __in_word(BaseName, value);\
 __restore_interrupt(_t);\
}
#endif /* __INTRINSICS_H */
