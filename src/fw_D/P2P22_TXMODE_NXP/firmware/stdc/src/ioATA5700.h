#ifndef IOATA5700_H_OVERALL
#define IOATA5700_H_OVERALL
/****** THIS IS A MACHINE GENERATED FILE - DO NOT EDIT ******************** */
/****** Created: 2014-01-21 18:32 ******* Source: ATA5700.xml ************* */
/**************************************************************************
 *
 * - File Name          : "ioATA5700.h"
 * - Title              : Register/Bit Definitions for the ATA5700
 * - Date               : 2014-01-21
 * - Version            : 2.25
 * - Support E-Mail     : avr@atmel.com
 * - Target MCU         : ATA5700
 *
 * - Compiler           : IAR Embedded Workbench - iccAVR and aAVR.
 *
 **************************************************************************
 */


#include "iomacro.h"

#if TID_GUARD(3)
#error This file should only be compiled with iccavr or aavr whith processor option -v3
#endif /* TID_GUARD(3) */

/* Include the SFR part if this file has not been included before,
* OR this file is included by the assembler (SFRs must be defined in
* each assembler module). */
#if !defined(__IOATA5700_H) || defined(__IAR_SYSTEMS_ASM__)

#pragma language=extended

/*==========================*/
/* Predefined SFR Addresses */
/*==========================*/


/* ***** SPECIFY DEVICE *************************************************** */
/*#define  PART_NAME       ATA5700 */
#define    SIGNATURE_000   0x1e
#define    SIGNATURE_001   0x95
#define    SIGNATURE_002   0x67



/* ***** I/O REGISTER DEFINITIONS ***************************************** */
/* NOTE:
 * Definitions marked "MEMORY MAPPED"are extended I/O ports
 * and cannot be used with IN/OUT instructions
 */
SFR_B(GPIOR0,	0x00)
SFR_B(PRR1,	0x01)
SFR_B(PRR2,	0x02)
SFR_B(PINB,	0x03)
SFR_B(DDRB,	0x04)
SFR_B(PORTB,	0x05)
SFR_B(PINC,	0x06)
SFR_B(DDRC,	0x07)
SFR_B(PORTC,	0x08)
SFR_B(PIND,	0x09)
SFR_B(DDRD,	0x0a)
SFR_B(PORTD,	0x0b)
SFR_B(TPCR2,	0x0c)
SFR_B(TPFR,	0x0d)
SFR_B(MCUCR,	0x0e)
SFR_B(FSCR,	0x0f)
SFR_B(T1CR,	0x11)
SFR_B(T2CR,	0x12)
SFR_B(T3CR,	0x13)
SFR_B(T4CR,	0x14)
SFR_B(T1IFR,	0x15)
SFR_B(T2IFR,	0x16)
SFR_B(T3IFR,	0x17)
SFR_B(T4IFR,	0x18)
SFR_B(T5IFR,	0x19)
SFR_B(PRR0,	0x1a)
SFR_B(PHFR,	0x1b)
SFR_B(LFFR,	0x1c)
SFR_B(AESCR,	0x1d)
SFR_B(AESSR,	0x1e)
SFR_B(EECR,	0x1f)
SFR_B(EEDR,	0x20)
SFR_B(EEARL,	0x21)
SFR_B(EEARH,	0x22)
SFR_B(EEPR0,	0x23)
SFR_B(EEPR,	0x23)
SFR_B(GPIOR1,	0x24)
SFR_B(GPIOR2,	0x25)
SFR_B(PCICR,	0x26)
SFR_B(EIMSK,	0x27)
SFR_B(EIFR,	0x28)
SFR_B(CRCDIR,	0x29)
SFR_B(VMSR,	0x2a)
SFR_B(MCUSR,	0x2b)
SFR_B(SPCR,	0x2c)
SFR_B(SPSR,	0x2d)
SFR_B(SPDR,	0x2e)
SFR_B(LFCR0,	0x2f)
SFR_B(LFCR1,	0x30)
SFR_B(DWDR,	0x31)
SFR_B(T0IFR,	0x32)
SFR_B(SPMCSR,	0x37)
SFR_B(SMCR,	0x38)
SFR_B(TPSR,	0x39)
SFR_B(LFCR2,	0x3a)
SFR_B(LFCR3,	0x3b)
SFR_B(SPL,	0x3d)
SFR_B(SPH,	0x3e)
SFR_B(SREG,	0x3f)
SFR_B(FSEN,	0x60)
SFR_B(FSFCR,	0x61)
SFR_B(GACDIVL,	0x62)
SFR_B(GACDIVH,	0x63)
SFR_B(FFREQ1L,	0x64)
SFR_B(FFREQ1M,	0x65)
SFR_B(FFREQ1H,	0x66)
SFR_B(FFREQ2L,	0x67)
SFR_B(FFREQ2M,	0x68)
SFR_B(FFREQ2H,	0x69)
SFR_B(BBTE2,	0x6a)
SFR_B(EICRA,	0x6b)
SFR_B(PCMSK0,	0x6c)
SFR_B(PCMSK1,	0x6d)
SFR_B(WDTCR,	0x6e)
SFR_B(T1CNT,	0x6f)
SFR_B(T1COR,	0x70)
SFR_B(T1MR,	0x71)
SFR_B(T1IMR,	0x72)
SFR_B(T2CNT,	0x73)
SFR_B(T2COR,	0x74)
SFR_B(T2MR,	0x75)
SFR_B(T2IMR,	0x76)
SFR_B(T3CNTL,	0x77)
SFR_B(T3CNTH,	0x78)
SFR_B(T3CORL,	0x79)
SFR_B(T3CORH,	0x7a)
SFR_B(T3ICRL,	0x7b)
SFR_B(T3ICRH,	0x7c)
SFR_B(T3MRA,	0x7d)
SFR_B(T3MRB,	0x7e)
SFR_B(T3IMR,	0x7f)
SFR_B(T4CNTL,	0x80)
SFR_B(T4CNTH,	0x81)
SFR_B(T4CORL,	0x82)
SFR_B(T4CORH,	0x83)
SFR_B(T4ICRL,	0x84)
SFR_B(T4ICRH,	0x85)
SFR_B(T4MRA,	0x86)
SFR_B(T4MRB,	0x87)
SFR_B(T4IMR,	0x88)
SFR_B(T5TEMP,	0x89)
SFR_B(T5OCRL,	0x8a)
SFR_B(T5OCRH,	0x8b)
SFR_B(T5CCR,	0x8c)
SFR_B(T5CNTL,	0x8d)
SFR_B(T5CNTH,	0x8e)
SFR_B(T5IMR,	0x8f)
SFR_B(GTCCR,	0x90)
SFR_B(LFCALR1,	0x96)
SFR_B(LFCALR2,	0x97)
SFR_B(LFCALR3,	0x98)
SFR_B(LFCALR4,	0x99)
SFR_B(LFCALR5,	0x9a)
SFR_B(LFCALR6,	0x9b)
SFR_B(LFCALR7,	0x9c)
SFR_B(LFCALR8,	0x9d)
SFR_B(LFCALR9,	0x9e)
SFR_B(LFCALR10,	0x9f)
SFR_B(LFCALR11,	0xa0)
SFR_B(LFCALR12,	0xa1)
SFR_B(LFCALR13,	0xa2)
SFR_B(LFCALR14,	0xa3)
SFR_B(LFCALR15,	0xa4)
SFR_B(LFCALR16,	0xa5)
SFR_B(LFCALR17,	0xa6)
SFR_B(LFCALR18,	0xa7)
SFR_B(LFCALR19,	0xa8)
SFR_B(LFCALR20,	0xa9)
SFR_B(LFCALR21,	0xaa)
SFR_B(LFCALR22,	0xab)
SFR_B(LFCALR23,	0xac)
SFR_B(LFCALR24,	0xad)
SFR_B(LFCALR25,	0xae)
SFR_B(LFCALR26,	0xaf)
SFR_B(LFCALR27,	0xb0)
SFR_B(LFCALR28,	0xb1)
SFR_B(LFCALR29,	0xb2)
SFR_B(LFCALR30,	0xb3)
SFR_B(LFCALR31,	0xb4)
SFR_B(LFCALR32,	0xb5)
SFR_B(LFCALR33,	0xb6)
SFR_B(LFCALR34,	0xb7)
SFR_B(LFCALR35,	0xb8)
SFR_B(LFCALR36,	0xb9)
SFR_B(LFCALR37,	0xba)
SFR_B(LFCALR38,	0xbb)
SFR_B(LFCALR39,	0xbc)
SFR_B(LFCALR40,	0xbd)
SFR_B(CLKOD,	0xc3)
SFR_B(CLKOCR,	0xc4)
SFR_B(XFUSE,	0xc5)
SFR_B(MRCCAL,	0xc6)
SFR_B(FRCCAL,	0xc7)
SFR_B(RCTCAL,	0xc8)
SFR_B(CMSR,	0xc9)
SFR_B(CMOCR,	0xca)
SFR_B(SUPFR,	0xcb)
SFR_B(SUPCR,	0xcc)
SFR_B(SUPCA1,	0xcd)
SFR_B(SUPCA2,	0xce)
SFR_B(SUPCA3,	0xcf)
SFR_B(SUPCA4,	0xd0)
SFR_B(CALRDY,	0xd1)
SFR_B(DFS,	0xd2)
SFR_B(DFTLL,	0xd3)
SFR_B(DFTLH,	0xd4)
SFR_B(DFL,	0xd5)
SFR_B(DFWP,	0xd6)
SFR_B(DFRP,	0xd7)
SFR_B(DFD,	0xd8)
SFR_B(DFI,	0xd9)
SFR_B(DFC,	0xda)
SFR_B(SFS,	0xdb)
SFR_B(SFL,	0xdc)
SFR_B(SFWP,	0xdd)
SFR_B(SFRP,	0xde)
SFR_B(SFD,	0xdf)
SFR_B(SFI,	0xe0)
SFR_B(SFC,	0xe1)
SFR_B(SSMCR,	0xe2)
SFR_B(SSMFBR,	0xe4)
SFR_B(SSMRR,	0xe5)
SFR_B(SSMSR,	0xe6)
SFR_B(SSMIFR,	0xe7)
SFR_B(SSMIMR,	0xe8)
SFR_B(MSMSTR,	0xe9)
SFR_B(SSMSTR,	0xea)
SFR_B(MSMCR1,	0xec)
SFR_B(MSMCR2,	0xed)
SFR_B(MSMCR3,	0xee)
SFR_B(MSMCR4,	0xef)
SFR_B(TRCIDL,	0xfc)
SFR_B(TRCIDH,	0xfd)
SFR_B(TRCDR,	0xff)
SFR_B_EXT_IO_R(0x100,	FESR)
SFR_B_EXT_IO_R(0x101,	FEEN1)
SFR_B_EXT_IO_R(0x102,	FEEN2)
SFR_B_EXT_IO_R(0x103,	FELNA)
SFR_B_EXT_IO_R(0x104,	FEAT)
SFR_B_EXT_IO_R(0x105,	FEPAC)
SFR_B_EXT_IO_R(0x106,	FEVCT)
SFR_B_EXT_IO_R(0x107,	FEBT)
SFR_B_EXT_IO_R(0x108,	FEMS)
SFR_B_EXT_IO_R(0x109,	FETN4)
SFR_B_EXT_IO_R(0x10a,	FECR)
SFR_B_EXT_IO_R(0x10b,	FEVCO)
SFR_B_EXT_IO_R(0x10c,	FEALR)
SFR_B_EXT_IO_R(0x10d,	FEANT)
SFR_B_EXT_IO_R(0x10e,	FEBIA)
SFR_B_EXT_IO_R(0x10f,	SPARE1)
SFR_B_EXT_IO_R(0x110,	SPARE2)
SFR_B_EXT_IO_R(0x111,	SPARE3)
SFR_B_EXT_IO_R(0x112,	SPARE4)
SFR_B_EXT_IO_R(0x11c,	FETE1)
SFR_B_EXT_IO_R(0x11d,	FETE2)
SFR_B_EXT_IO_R(0x11e,	FETE3)
SFR_B_EXT_IO_R(0x11f,	FETD)
SFR_B_EXT_IO_R(0x120,	TMFSM)
SFR_B_EXT_IO_R(0x121,	TMCRCL)
SFR_B_EXT_IO_R(0x122,	TMCRCH)
SFR_B_EXT_IO_R(0x123,	TMCSB)
SFR_B_EXT_IO_R(0x124,	TMCIL)
SFR_B_EXT_IO_R(0x125,	TMCIH)
SFR_B_EXT_IO_R(0x126,	TMCPL)
SFR_B_EXT_IO_R(0x127,	TMCPH)
SFR_B_EXT_IO_R(0x128,	TMSHR)
SFR_B_EXT_IO_R(0x129,	TMTLL)
SFR_B_EXT_IO_R(0x12a,	TMTLH)
SFR_B_EXT_IO_R(0x12b,	TMSSC)
SFR_B_EXT_IO_R(0x12c,	TMSR)
SFR_B_EXT_IO_R(0x12d,	TMCR2)
SFR_B_EXT_IO_R(0x12e,	TMCR1)
SFR_B_EXT_IO_R(0x130,	LFDSR1)
SFR_B_EXT_IO_R(0x131,	LFDSR2)
SFR_B_EXT_IO_R(0x132,	LFDSR3)
SFR_B_EXT_IO_R(0x133,	LFDSR4)
SFR_B_EXT_IO_R(0x134,	LFDSR5)
SFR_B_EXT_IO_R(0x135,	LFDSR6)
SFR_B_EXT_IO_R(0x136,	LFDSR7)
SFR_B_EXT_IO_R(0x137,	LFDSR8)
SFR_B_EXT_IO_R(0x138,	LFDSR9)
SFR_B_EXT_IO_R(0x139,	LFDSR10)
SFR_B_EXT_IO_R(0x13a,	LFDSR11)
SFR_B_EXT_IO_R(0x13b,	EEPR1)
SFR_B_EXT_IO_R(0x13c,	EEPR2)
SFR_B_EXT_IO_R(0x13d,	EEPR3)
SFR_B_EXT_IO_R(0x145,	CRCCR)
SFR_B_EXT_IO_R(0x146,	CRCDOR)
SFR_B_EXT_IO_R(0x151,	LFDSRR)
SFR_B_EXT_IO_R(0x152,	DBCR)
SFR_B_EXT_IO_R(0x153,	DBTC)
SFR_B_EXT_IO_R(0x154,	DBENB)
SFR_B_EXT_IO_R(0x155,	DBENC)
SFR_B_EXT_IO_R(0x156,	DBGSW)
SFR_B_EXT_IO_R(0x157,	SFFR)
SFR_B_EXT_IO_R(0x158,	SFIR)
SFR_B_EXT_IO_R(0x159,	EECR2)
SFR_B_EXT_IO_R(0x15a,	PGMST)
SFR_B_EXT_IO_R(0x15b,	EEST)
SFR_B_EXT_IO_R(0x160,	LFRSFR)
SFR_B_EXT_IO_R(0x161,	PCIFR)
SFR_B_EXT_IO_R(0x162,	T0CR)
SFR_B_EXT_IO_R(0x164,	DBEND)
SFR_B_EXT_IO_R(0x165,	TPCR1)
SFR_B_EXT_IO_R(0x166,	TPIMR)
SFR_B_EXT_IO_R(0x167,	TPDCR1)
SFR_B_EXT_IO_R(0x168,	TPDCR2)
SFR_B_EXT_IO_R(0x169,	TPDCR3)
SFR_B_EXT_IO_R(0x16a,	TPDCR4)
SFR_B_EXT_IO_R(0x16b,	TPDCR5)
SFR_B_EXT_IO_R(0x16c,	TPECR1)
SFR_B_EXT_IO_R(0x16d,	TPECR2)
SFR_B_EXT_IO_R(0x16e,	TPECR3)
SFR_B_EXT_IO_R(0x16f,	TPECR4)
SFR_B_EXT_IO_R(0x170,	TPECMR)
SFR_B_EXT_IO_R(0x171,	TPCR3)
SFR_B_EXT_IO_R(0x172,	TPCR4)
SFR_B_EXT_IO_R(0x173,	TPCR5)
SFR_B_EXT_IO_R(0x174,	LFRSMR)
SFR_B_EXT_IO_R(0x175,	TPCALR1)
SFR_B_EXT_IO_R(0x176,	TPCALR2)
SFR_B_EXT_IO_R(0x177,	TPCALR3)
SFR_B_EXT_IO_R(0x178,	TPCALR4)
SFR_B_EXT_IO_R(0x179,	TPCALR5)
SFR_B_EXT_IO_R(0x17a,	TPCALR6)
SFR_B_EXT_IO_R(0x17b,	TPCALR7)
SFR_B_EXT_IO_R(0x17c,	TPCALR8)
SFR_B_EXT_IO_R(0x17d,	TPCALR9)
SFR_B_EXT_IO_R(0x17e,	TPCALR10)
SFR_B_EXT_IO_R(0x17f,	AESDPR)
SFR_B_EXT_IO_R(0x180,	AESKR)
SFR_B_EXT_IO_R(0x181,	AESDR)
SFR_B_EXT_IO_R(0x182,	GPIOR3)
SFR_B_EXT_IO_R(0x183,	GPIOR4)
SFR_B_EXT_IO_R(0x184,	GPIOR5)
SFR_B_EXT_IO_R(0x185,	GPIOR6)
SFR_B_EXT_IO_R(0x186,	PHBCRR)
SFR_B_EXT_IO_R(0x187,	LFRSCR)
SFR_B_EXT_IO_R(0x188,	LFRSC1L)
SFR_B_EXT_IO_R(0x189,	LFRSC1H)
SFR_B_EXT_IO_R(0x18a,	LFRSC2L)
SFR_B_EXT_IO_R(0x18b,	LFRSC2H)
SFR_B_EXT_IO_R(0x18c,	LFRSC3L)
SFR_B_EXT_IO_R(0x18d,	LFRSC3H)
SFR_B_EXT_IO_R(0x18e,	LFCPR)
SFR_B_EXT_IO_R(0x18f,	LFIMR)
SFR_B_EXT_IO_R(0x190,	PHID00)
SFR_B_EXT_IO_R(0x191,	PHID01)
SFR_B_EXT_IO_R(0x192,	PHID02)
SFR_B_EXT_IO_R(0x193,	PHID03)
SFR_B_EXT_IO_R(0x194,	PHID0L)
SFR_B_EXT_IO_R(0x195,	PHID10)
SFR_B_EXT_IO_R(0x196,	PHID11)
SFR_B_EXT_IO_R(0x197,	PHID12)
SFR_B_EXT_IO_R(0x198,	PHID13)
SFR_B_EXT_IO_R(0x199,	PHID1L)
SFR_B_EXT_IO_R(0x19a,	PHIDFR)
SFR_B_EXT_IO_R(0x19b,	LFSYSY0)
SFR_B_EXT_IO_R(0x19c,	LFSYSY1)
SFR_B_EXT_IO_R(0x19d,	LFSYSY2)
SFR_B_EXT_IO_R(0x19e,	LFSYSY3)
SFR_B_EXT_IO_R(0x19f,	LFSYLE)
SFR_B_EXT_IO_R(0x1a0,	LFSTOP)
SFR_B_EXT_IO_R(0x1a1,	PHTCOR)
SFR_B_EXT_IO_R(0x1a2,	PHTCMR)
SFR_B_EXT_IO_R(0x1a4,	PHTBLR)
SFR_B_EXT_IO_R(0x1a5,	PHDFR)
SFR_B_EXT_IO_R(0x1a6,	PHTEMR)
SFR_B_EXT_IO_R(0x1a7,	LFQC3)
SFR_B_EXT_IO_R(0x1a8,	LFQC2)
SFR_B_EXT_IO_R(0x1a9,	LFQC1)
SFR_B_EXT_IO_R(0x1ad,	LFRSS1R)
SFR_B_EXT_IO_R(0x1ae,	LFRSS2R)
SFR_B_EXT_IO_R(0x1af,	LFRSISR)
SFR_B_EXT_IO_R(0x1d1,	PHFS)
SFR_B_EXT_IO_R(0x1d2,	PHFL)
SFR_B_EXT_IO_R(0x1d3,	PHFWP)
SFR_B_EXT_IO_R(0x1d4,	PHFRP)
SFR_B_EXT_IO_R(0x1d5,	PHFD)
SFR_B_EXT_IO_R(0x1d6,	PHFI)
SFR_B_EXT_IO_R(0x1d7,	PHFC)
SFR_B_EXT_IO_R(0x1d8,	PHIMR)
SFR_B_EXT_IO_R(0x1d9,	PHCRCR)
SFR_B_EXT_IO_R(0x1da,	PHCSTL)
SFR_B_EXT_IO_R(0x1db,	PHCSTH)
SFR_B_EXT_IO_R(0x1dc,	PHCRPL)
SFR_B_EXT_IO_R(0x1dd,	PHCRPH)
SFR_B_EXT_IO_R(0x1de,	PHCSRL)
SFR_B_EXT_IO_R(0x1df,	PHCSRH)
SFR_B_EXT_IO_R(0x1e0,	PHCKSR)
SFR_B_EXT_IO_R(0x1e1,	PHCKCR)
SFR_B_EXT_IO_R(0x1e3,	CMCR)
SFR_B_EXT_IO_R(0x1e4,	CMIMR)
SFR_B_EXT_IO_R(0x1e5,	CLPR)
SFR_B_EXT_IO_R(0x1e6,	VMCR)
SFR_B_EXT_IO_R(0x1e7,	DBONDR)
SFR_B_EXT_IO_R(0x1e8,	CALRDYLF)
SFR_B_EXT_IO_R(0x1e9,	TWBR)
SFR_B_EXT_IO_R(0x1ea,	TWCR)
SFR_B_EXT_IO_R(0x1eb,	TWSR)
SFR_B_EXT_IO_R(0x1ec,	TWDR)
SFR_B_EXT_IO_R(0x1ed,	TWAR)
SFR_B_EXT_IO_R(0x1ee,	TWAMR)
SFR_B_EXT_IO_R(0x1ef,	PDSCR)
SFR_B_EXT_IO_R(0x1f0,	TMOCR)
SFR_B_EXT_IO_R(0x1f1,	SRCCAL)
SFR_B_EXT_IO_R(0x1f2,	SRCTCAL)
SFR_B_EXT_IO_R(0x1f3,	SUPCA5)
SFR_B_EXT_IO_R(0x1f4,	SUPCA6)
SFR_B_EXT_IO_R(0x1f5,	SUPCA7)
SFR_B_EXT_IO_R(0x1f6,	SUPCA8)
SFR_B_EXT_IO_R(0x1f7,	SUPCA9)
SFR_B_EXT_IO_R(0x1f8,	SUPCA10)
SFR_B_EXT_IO_R(0x1f9,	TPCALR11)
SFR_B_EXT_IO_R(0x1fa,	TPCALR12)
SFR_B_EXT_IO_R(0x1fe,	PMTER)



#ifndef __IOATA5700_H
#define __IOATA5700_H

#ifdef __IAR_SYSTEMS_ASM__ 
#ifndef ENABLE_BIT_DEFINITIONS
#define  ENABLE_BIT_DEFINITIONS
#endif /* ENABLE_BIT_DEFINITIONS */
#endif /* __IAR_SYSTEMS_ASM__ */

#ifdef ENABLE_BIT_DEFINITIONS
/* ***** BIT DEFINITIONS ************************************************** */

/* ***** AES ************************** */
/* AESCR - AES Control Register */
#define    AESWK           0       /* AES Write Key */
#define    BM_AESWK	(uint8_t)(1U<<AESWK)
#define    AESWD           1       /* AES Write Data */
#define    BM_AESWD	(uint8_t)(1U<<AESWD)
#define    AESIM           2       /* AES Interrupt Mask */
#define    BM_AESIM	(uint8_t)(1U<<AESIM)
#define    AESD            3       /* AES Direction */
#define    BM_AESD	(uint8_t)(1U<<AESD)
#define    AESXOR          4       /* AES State XOR load */
#define    BM_AESXOR	(uint8_t)(1U<<AESXOR)
#define    AESRES          5       /* AES Reset */
#define    BM_AESRES	(uint8_t)(1U<<AESRES)
#define    AESLKM          6       /* AES Load Key Memory */
#define    BM_AESLKM	(uint8_t)(1U<<AESLKM)
#define    AESE            7       /* AES Enable */
#define    BM_AESE	(uint8_t)(1U<<AESE)

/* AESDPR - AES Data Pointer Register */
#define    AESDPR0         0       /* AES Data Pointer Register bit 0 */
#define    BM_AESDPR0	(uint8_t)(1U<<AESDPR0)
#define    AESDPR1         1       /* AES Data Pointer Register bit 1 */
#define    BM_AESDPR1	(uint8_t)(1U<<AESDPR1)
#define    AESDPR2         2       /* AES Data Pointer Register bit 2 */
#define    BM_AESDPR2	(uint8_t)(1U<<AESDPR2)
#define    AESDPR3         3       /* AES Data Pointer Register bit 3 */
#define    BM_AESDPR3	(uint8_t)(1U<<AESDPR3)

/* AESDR - AES Data Register */
#define    AESDR0          0       /* AES Data Register bit 0 */
#define    BM_AESDR0	(uint8_t)(1U<<AESDR0)
#define    AESDR1          1       /* AES Data Register bit 1 */
#define    BM_AESDR1	(uint8_t)(1U<<AESDR1)
#define    AESDR2          2       /* AES Data Register bit 2 */
#define    BM_AESDR2	(uint8_t)(1U<<AESDR2)
#define    AESDR3          3       /* AES Data Register bit 3 */
#define    BM_AESDR3	(uint8_t)(1U<<AESDR3)
#define    AESDR4          4       /* AES Data Register bit 4 */
#define    BM_AESDR4	(uint8_t)(1U<<AESDR4)
#define    AESDR5          5       /* AES Data Register bit 5 */
#define    BM_AESDR5	(uint8_t)(1U<<AESDR5)
#define    AESDR6          6       /* AES Data Register bit 6 */
#define    BM_AESDR6	(uint8_t)(1U<<AESDR6)
#define    AESDR7          7       /* AES Data Register bit 7 */
#define    BM_AESDR7	(uint8_t)(1U<<AESDR7)

/* AESKR - AES Key Register */
#define    AESKR0          0       /* AES Key Register bit 0 */
#define    BM_AESKR0	(uint8_t)(1U<<AESKR0)
#define    AESKR1          1       /* AES Key Register bit 1 */
#define    BM_AESKR1	(uint8_t)(1U<<AESKR1)
#define    AESKR2          2       /* AES Key Register bit 2 */
#define    BM_AESKR2	(uint8_t)(1U<<AESKR2)
#define    AESKR3          3       /* AES Key Register bit 3 */
#define    BM_AESKR3	(uint8_t)(1U<<AESKR3)
#define    AESKR4          4       /* AES Key Register bit 4 */
#define    BM_AESKR4	(uint8_t)(1U<<AESKR4)
#define    AESKR5          5       /* AES Key Register bit 5 */
#define    BM_AESKR5	(uint8_t)(1U<<AESKR5)
#define    AESKR6          6       /* AES Key Register bit 6 */
#define    BM_AESKR6	(uint8_t)(1U<<AESKR6)
#define    AESKR7          7       /* AES Key Register bit 7 */
#define    BM_AESKR7	(uint8_t)(1U<<AESKR7)

/* AESSR - AES Status Register */
#define    AESRF           0       /* AES Ready Flag */
#define    BM_AESRF	(uint8_t)(1U<<AESRF)
#define    AESERF          7       /* AES Error Flag */
#define    BM_AESERF	(uint8_t)(1U<<AESERF)


/* ***** CLK ************************** */
/* CLKOCR - Clock output control Register */
#define    CLKOS0          0       /* Clock output source bit 0 */
#define    BM_CLKOS0	(uint8_t)(1U<<CLKOS0)
#define    CLKOS1          1       /* Clock output source bit 1 */
#define    BM_CLKOS1	(uint8_t)(1U<<CLKOS1)
#define    CLKOEN          2       /* Clock output driver enable */
#define    BM_CLKOEN	(uint8_t)(1U<<CLKOEN)

/* CLKOD - Clock output divider settings Register */
#define    CLKOD0          0       /* Clock output divider bit 0 */
#define    BM_CLKOD0	(uint8_t)(1U<<CLKOD0)
#define    CLKOD1          1       /* Clock output divider bit 1 */
#define    BM_CLKOD1	(uint8_t)(1U<<CLKOD1)
#define    CLKOD2          2       /* Clock output divider bit 2 */
#define    BM_CLKOD2	(uint8_t)(1U<<CLKOD2)
#define    CLKOD3          3       /* Clock output divider bit 3 */
#define    BM_CLKOD3	(uint8_t)(1U<<CLKOD3)
#define    CLKOD4          4       /* Clock output divider bit 4 */
#define    BM_CLKOD4	(uint8_t)(1U<<CLKOD4)
#define    CLKOD5          5       /* Clock output divider bit 5 */
#define    BM_CLKOD5	(uint8_t)(1U<<CLKOD5)
#define    CLKOD6          6       /* Clock output divider bit 6 */
#define    BM_CLKOD6	(uint8_t)(1U<<CLKOD6)
#define    CLKOD7          7       /* Clock output divider bit 7 */
#define    BM_CLKOD7	(uint8_t)(1U<<CLKOD7)

/* CMOCR - Clock management override control register */
#define    FRCAO           0       /* FRC Always On */
#define    BM_FRCAO	(uint8_t)(1U<<FRCAO)
#define    MRCAO           1       /* MRC Always On */
#define    BM_MRCAO	(uint8_t)(1U<<MRCAO)
#define    FRCACT          2       /* FRC Active */
#define    BM_FRCACT	(uint8_t)(1U<<FRCACT)

/* CMSR - Clock management status Register */
#define    ECF             0       /* External clock fail */
#define    BM_ECF	(uint8_t)(1U<<ECF)

/* FRCCAL - Fast RC oscillator calibration Register */
#define    FRCCAL0         0       /* Fast RC oscillator calibration bit 0 */
#define    BM_FRCCAL0	(uint8_t)(1U<<FRCCAL0)
#define    FRCCAL1         1       /* Fast RC oscillator calibration bit 1 */
#define    BM_FRCCAL1	(uint8_t)(1U<<FRCCAL1)
#define    FRCCAL2         2       /* Fast RC oscillator calibration bit 2 */
#define    BM_FRCCAL2	(uint8_t)(1U<<FRCCAL2)
#define    FRCCAL3         3       /* Fast RC oscillator calibration bit 3 */
#define    BM_FRCCAL3	(uint8_t)(1U<<FRCCAL3)
#define    FRCCAL4         4       /* Fast RC oscillator calibration bit 4 */
#define    BM_FRCCAL4	(uint8_t)(1U<<FRCCAL4)

/* MRCCAL - Middle RC oscillator calibration Register */
#define    MRCCAL1         1       /* Middle RC oscillator calibration bit 1 */
#define    BM_MRCCAL1	(uint8_t)(1U<<MRCCAL1)
#define    MRCCAL2         2       /* Middle RC oscillator calibration bit 2 */
#define    BM_MRCCAL2	(uint8_t)(1U<<MRCCAL2)
#define    MRCCAL3         3       /* Middle RC oscillator calibration bit 3 */
#define    BM_MRCCAL3	(uint8_t)(1U<<MRCCAL3)
#define    MRCCAL4         4       /* Middle RC oscillator calibration bit 4 */
#define    BM_MRCCAL4	(uint8_t)(1U<<MRCCAL4)
#define    MRCCAL5         5       /* Middle RC oscillator calibration bit 5 */
#define    BM_MRCCAL5	(uint8_t)(1U<<MRCCAL5)
#define    MRCCAL6         6       /* Middle RC oscillator calibration bit 6 */
#define    BM_MRCCAL6	(uint8_t)(1U<<MRCCAL6)
#define    MRCCAL7         7       /* Middle RC oscillator calibration bit 7 */
#define    BM_MRCCAL7	(uint8_t)(1U<<MRCCAL7)

/* PRR0 - Power reduction Register 0 */
#define    PRSPI           0       /* Power Reduction Serial Peripheral Interface */
#define    BM_PRSPI	(uint8_t)(1U<<PRSPI)
#define    PRR0_R1         1       /*  */
#define    BM_PRR0_R1	(uint8_t)(1U<<PRR0_R1)
#define    PRTXDC          2       /* Power Reduction Transmit DSP Control */
#define    BM_PRTXDC	(uint8_t)(1U<<PRTXDC)
#define    PRCRC           3       /* Power Reduction CRC */
#define    BM_PRCRC	(uint8_t)(1U<<PRCRC)
#define    PRVM            4       /* Power Reduction Voltage Monitor */
#define    BM_PRVM	(uint8_t)(1U<<PRVM)
#define    PRCO            5       /* Power Reduction Clock Output */
#define    BM_PRCO	(uint8_t)(1U<<PRCO)
#define    PRCU            6       /* Power Reduction Crypto Unit */
#define    BM_PRCU	(uint8_t)(1U<<PRCU)
#define    PRTWI           7       /* Power Reduction Two Wire Interface */
#define    BM_PRTWI	(uint8_t)(1U<<PRTWI)

/* PRR1 - Power reduction Register 1 */
#define    PRT1            0       /* Power Reduction Timer 1 */
#define    BM_PRT1	(uint8_t)(1U<<PRT1)
#define    PRT2            1       /* Power Reduction Timer 2 */
#define    BM_PRT2	(uint8_t)(1U<<PRT2)
#define    PRT3            2       /* Power Reduction Timer 3 */
#define    BM_PRT3	(uint8_t)(1U<<PRT3)
#define    PRT4            3       /* Power Reduction Timer 4 */
#define    BM_PRT4	(uint8_t)(1U<<PRT4)
#define    PRT5            4       /* Power Reduction Timer 5 */
#define    BM_PRT5	(uint8_t)(1U<<PRT5)
#define    PRLFR           5       /* Power Reduction LF Receiver */
#define    BM_PRLFR	(uint8_t)(1U<<PRLFR)
#define    PRCI            6       /* Power Reduction Contactless Interface */
#define    BM_PRCI	(uint8_t)(1U<<PRCI)

/* PRR2 - Power reduction register 2 */
#define    PRR2_R0         0       /*  */
#define    BM_PRR2_R0	(uint8_t)(1U<<PRR2_R0)
#define    PRR2_R1         1       /*  */
#define    BM_PRR2_R1	(uint8_t)(1U<<PRR2_R1)
#define    PRSF            2       /* Power Reduction Preamble/RSSI FIFO */
#define    BM_PRSF	(uint8_t)(1U<<PRSF)
#define    PRDF            3       /* Power Reduction Data FIFO */
#define    BM_PRDF	(uint8_t)(1U<<PRDF)
#define    PRR2_R4         4       /*  */
#define    BM_PRR2_R4	(uint8_t)(1U<<PRR2_R4)
#define    PRR2_R5         5       /*  */
#define    BM_PRR2_R5	(uint8_t)(1U<<PRR2_R5)
#define    PRTM            6       /* Power Reduction Tx Modulator */
#define    BM_PRTM	(uint8_t)(1U<<PRTM)
#define    PRSSM           7       /* Power Reduction Sequencer State Machine */
#define    BM_PRSSM	(uint8_t)(1U<<PRSSM)

/* XFUSE -  */


/* ***** CPU ************************** */
/* CLPR - Clock Prescaler Register */
#define    CLKPS0          0       /* System Clock Prescaler Select bit 0 */
#define    BM_CLKPS0	(uint8_t)(1U<<CLKPS0)
#define    CLKPS1          1       /* System Clock Prescaler Select bit 1 */
#define    BM_CLKPS1	(uint8_t)(1U<<CLKPS1)
#define    CLKPS2          2       /* System Clock Prescaler Select bit 2 */
#define    BM_CLKPS2	(uint8_t)(1U<<CLKPS2)
#define    CLTPS0          3       /* Timer Clock Prescaler Select bit 0 */
#define    BM_CLTPS0	(uint8_t)(1U<<CLTPS0)
#define    CLTPS1          4       /* Timer Clock Prescaler Select bit 1 */
#define    BM_CLTPS1	(uint8_t)(1U<<CLTPS1)
#define    CLTPS2          5       /* Timer Clock Prescaler Select bit 2 */
#define    BM_CLTPS2	(uint8_t)(1U<<CLTPS2)
#define    CLPCE           7       /* Clock Prescaler Change Enable */
#define    BM_CLPCE	(uint8_t)(1U<<CLPCE)

/* CMCR - Clock Management Control Register */
#define    CMM0            0       /* Clock Management Mode bit 0 */
#define    BM_CMM0	(uint8_t)(1U<<CMM0)
#define    CMM1            1       /* Clock Management Mode bit 1 */
#define    BM_CMM1	(uint8_t)(1U<<CMM1)
#define    CMM2            2       /* Clock Management Mode bit 2 */
#define    BM_CMM2	(uint8_t)(1U<<CMM2)
#define    CCS             3       /* Core Clock Select */
#define    BM_CCS	(uint8_t)(1U<<CCS)
#define    CMONEN          6       /* Clock monitor enable */
#define    BM_CMONEN	(uint8_t)(1U<<CMONEN)
#define    CMCCE           7       /* Clock management control change enable */
#define    BM_CMCCE	(uint8_t)(1U<<CMCCE)

/* CMIMR - Clock interrupt mask Register */
#define    ECIE            0       /* External clock interrupt enable */
#define    BM_ECIE	(uint8_t)(1U<<ECIE)

/* DWDR - Debug Wire Data Register */
#define    DWDR0           0       /* Debug Wire Data Register bit 0 */
#define    BM_DWDR0	(uint8_t)(1U<<DWDR0)
#define    DWDR1           1       /* Debug Wire Data Register bit 1 */
#define    BM_DWDR1	(uint8_t)(1U<<DWDR1)
#define    DWDR2           2       /* Debug Wire Data Register bit 2 */
#define    BM_DWDR2	(uint8_t)(1U<<DWDR2)
#define    DWDR3           3       /* Debug Wire Data Register bit 3 */
#define    BM_DWDR3	(uint8_t)(1U<<DWDR3)
#define    DWDR4           4       /* Debug Wire Data Register bit 4 */
#define    BM_DWDR4	(uint8_t)(1U<<DWDR4)
#define    DWDR5           5       /* Debug Wire Data Register bit 5 */
#define    BM_DWDR5	(uint8_t)(1U<<DWDR5)
#define    DWDR6           6       /* Debug Wire Data Register bit 6 */
#define    BM_DWDR6	(uint8_t)(1U<<DWDR6)
#define    DWDR7           7       /* Debug Wire Data Register bit 7 */
#define    BM_DWDR7	(uint8_t)(1U<<DWDR7)

/* MCUCR - MCU control Register */
#define    IVCE            0       /* Interrupt Vector Change Enable */
#define    BM_IVCE	(uint8_t)(1U<<IVCE)
#define    IVSEL           1       /* Interrupt Vector Select */
#define    BM_IVSEL	(uint8_t)(1U<<IVSEL)
#define    SPIIO           2       /* SPI Interrupt Only */
#define    BM_SPIIO	(uint8_t)(1U<<SPIIO)
#define    ENPS            3       /* Enable Port Settings */
#define    BM_ENPS	(uint8_t)(1U<<ENPS)
#define    PUD             4       /* Pull-up Resistors Disable */
#define    BM_PUD	(uint8_t)(1U<<PUD)
#define    TRCCE           5       /* Trace Change Enable */
#define    BM_TRCCE	(uint8_t)(1U<<TRCCE)
#define    TRCEN           6       /* Trace Enable */
#define    BM_TRCEN	(uint8_t)(1U<<TRCEN)

/* MCUSR - MCU Status Register */
#define    PORF            0       /* Power-On Reset Flag */
#define    BM_PORF	(uint8_t)(1U<<PORF)
#define    EXTRF           1       /* External Reset Flag */
#define    BM_EXTRF	(uint8_t)(1U<<EXTRF)
#define    MCUSR_R2        2       /*  */
#define    BM_MCUSR_R2	(uint8_t)(1U<<MCUSR_R2)
#define    WDRF            3       /* Watchdog Reset Flag */
#define    BM_WDRF	(uint8_t)(1U<<WDRF)
#define    TPRF            5       /* Transponder Reset Flag */
#define    BM_TPRF	(uint8_t)(1U<<TPRF)
#define    MCUSR_R6        6       /*  */
#define    BM_MCUSR_R6	(uint8_t)(1U<<MCUSR_R6)

/* SMCR - Sleep mode control Register */
#define    SE              0       /* Sleep Enable */
#define    BM_SE	(uint8_t)(1U<<SE)
#define    SM0             1       /* Sleep Mode Select bit 0 */
#define    BM_SM0	(uint8_t)(1U<<SM0)
#define    SM1             2       /* Sleep Mode Select bit 1 */
#define    BM_SM1	(uint8_t)(1U<<SM1)
#define    SM2             3       /* Sleep Mode Select bit 2 */
#define    BM_SM2	(uint8_t)(1U<<SM2)

/* SPH - Stack Pointer High */
#define    SP8             0       /* Stack pointer bit 8 */
#define    BM_SP8	(uint8_t)(1U<<SP8)
#define    SP9             1       /* Stack pointer bit 9 */
#define    BM_SP9	(uint8_t)(1U<<SP9)
#define    SP10            2       /* Stack pointer bit 10 */
#define    BM_SP10	(uint8_t)(1U<<SP10)

/* SPL - Stack Pointer Low */
#define    SP0             0       /* Stack pointer bit 0 */
#define    BM_SP0	(uint8_t)(1U<<SP0)
#define    SP1             1       /* Stack pointer bit 1 */
#define    BM_SP1	(uint8_t)(1U<<SP1)
#define    SP2             2       /* Stack pointer bit 2 */
#define    BM_SP2	(uint8_t)(1U<<SP2)
#define    SP3             3       /* Stack pointer bit 3 */
#define    BM_SP3	(uint8_t)(1U<<SP3)
#define    SP4             4       /* Stack pointer bit 4 */
#define    BM_SP4	(uint8_t)(1U<<SP4)
#define    SP5             5       /* Stack pointer bit 5 */
#define    BM_SP5	(uint8_t)(1U<<SP5)
#define    SP6             6       /* Stack pointer bit 6 */
#define    BM_SP6	(uint8_t)(1U<<SP6)
#define    SP7             7       /* Stack pointer bit 7 */
#define    BM_SP7	(uint8_t)(1U<<SP7)

/* SPMCSR - Store Program Memory Control and Status Register */
#define    SELFPRGEN       0       /* Self Programming Enable */
#define    BM_SELFPRGEN	(uint8_t)(1U<<SELFPRGEN)
#define    PGERS           1       /* Page Erase */
#define    BM_PGERS	(uint8_t)(1U<<PGERS)
#define    PGWRT           2       /* Page Write */
#define    BM_PGWRT	(uint8_t)(1U<<PGWRT)
#define    BLBSET          3       /* Boot Lock Bit Set */
#define    BM_BLBSET	(uint8_t)(1U<<BLBSET)
#define    RWWSRE          4       /* Read-While-Write Section Read Enable */
#define    BM_RWWSRE	(uint8_t)(1U<<RWWSRE)
#define    SIGRD           5       /*  */
#define    BM_SIGRD	(uint8_t)(1U<<SIGRD)
#define    RWWSB           6       /* Read-While-Write Section Busy */
#define    BM_RWWSB	(uint8_t)(1U<<RWWSB)
#define    SPMIE           7       /* SPM Interrupt Enable */
#define    BM_SPMIE	(uint8_t)(1U<<SPMIE)

/* SREG - Status Register */
#define    SREG_C          0       /* Carry Flag */
#define    BM_SREG_C	(uint8_t)(1U<<SREG_C)
#define    SREG_Z          1       /* Zero Flag */
#define    BM_SREG_Z	(uint8_t)(1U<<SREG_Z)
#define    SREG_N          2       /* Negative Flag */
#define    BM_SREG_N	(uint8_t)(1U<<SREG_N)
#define    SREG_V          3       /* Two's Complement Overflow Flag */
#define    BM_SREG_V	(uint8_t)(1U<<SREG_V)
#define    SREG_S          4       /* Sign Bit */
#define    BM_SREG_S	(uint8_t)(1U<<SREG_S)
#define    SREG_H          5       /* Half Carry Flag */
#define    BM_SREG_H	(uint8_t)(1U<<SREG_H)
#define    SREG_T          6       /* Bit Copy Storage */
#define    BM_SREG_T	(uint8_t)(1U<<SREG_T)
#define    SREG_I          7       /* Global Interrupt Enable */
#define    BM_SREG_I	(uint8_t)(1U<<SREG_I)


/* ***** CRC ************************** */
/* CRCCR - CRC Control Register */
#define    CRCRS           0       /* CRC Data Register Reset */
#define    BM_CRCRS	(uint8_t)(1U<<CRCRS)
#define    REFLI           1       /* Reflect Data Input Byte */
#define    BM_REFLI	(uint8_t)(1U<<REFLI)
#define    REFLO           2       /* Reflect Data Output Byte */
#define    BM_REFLO	(uint8_t)(1U<<REFLO)

/* CRCDIR - CRC Data Input Register */
#define    CRCDIR0         0       /* CRC Data Input Register bit 0 */
#define    BM_CRCDIR0	(uint8_t)(1U<<CRCDIR0)
#define    CRCDIR1         1       /* CRC Data Input Register bit 1 */
#define    BM_CRCDIR1	(uint8_t)(1U<<CRCDIR1)
#define    CRCDIR2         2       /* CRC Data Input Register bit 2 */
#define    BM_CRCDIR2	(uint8_t)(1U<<CRCDIR2)
#define    CRCDIR3         3       /* CRC Data Input Register bit 3 */
#define    BM_CRCDIR3	(uint8_t)(1U<<CRCDIR3)
#define    CRCDIR4         4       /* CRC Data Input Register bit 4 */
#define    BM_CRCDIR4	(uint8_t)(1U<<CRCDIR4)
#define    CRCDIR5         5       /* CRC Data Input Register bit 5 */
#define    BM_CRCDIR5	(uint8_t)(1U<<CRCDIR5)
#define    CRCDIR6         6       /* CRC Data Input Register bit 6 */
#define    BM_CRCDIR6	(uint8_t)(1U<<CRCDIR6)
#define    CRCDIR7         7       /* CRC Data Input Register bit 7 */
#define    BM_CRCDIR7	(uint8_t)(1U<<CRCDIR7)

/* CRCDOR - CRC Data Output Register */
#define    CRCDOR0         0       /* CRC Data Output Register bit 0 */
#define    BM_CRCDOR0	(uint8_t)(1U<<CRCDOR0)
#define    CRCDOR1         1       /* CRC Data Output Register bit 1 */
#define    BM_CRCDOR1	(uint8_t)(1U<<CRCDOR1)
#define    CRCDOR2         2       /* CRC Data Output Register bit 2 */
#define    BM_CRCDOR2	(uint8_t)(1U<<CRCDOR2)
#define    CRCDOR3         3       /* CRC Data Output Register bit 3 */
#define    BM_CRCDOR3	(uint8_t)(1U<<CRCDOR3)
#define    CRCDOR4         4       /* CRC Data Output Register bit 4 */
#define    BM_CRCDOR4	(uint8_t)(1U<<CRCDOR4)
#define    CRCDOR5         5       /* CRC Data Output Register bit 5 */
#define    BM_CRCDOR5	(uint8_t)(1U<<CRCDOR5)
#define    CRCDOR6         6       /* CRC Data Output Register bit 6 */
#define    BM_CRCDOR6	(uint8_t)(1U<<CRCDOR6)
#define    CRCDOR7         7       /* CRC Data Output Register bit 7 */
#define    BM_CRCDOR7	(uint8_t)(1U<<CRCDOR7)


/* ***** DEBOUNCE ********************* */
/* DBCR - DeBounce Control Register */
#define    DBMD            0       /* Debounce Mode */
#define    BM_DBMD	(uint8_t)(1U<<DBMD)
#define    DBCS            1       /* Debounce Clock Select */
#define    BM_DBCS	(uint8_t)(1U<<DBCS)
#define    DBTMS           2       /* Debounce Timer Mask Select */
#define    BM_DBTMS	(uint8_t)(1U<<DBTMS)
#define    DBHA            3       /* Debounce Handshake Active */
#define    BM_DBHA	(uint8_t)(1U<<DBHA)

/* DBENB - DeBounce Enable Port B */
#define    DBENB0          0       /* DeBounce Enable Port B Pin 0 */
#define    BM_DBENB0	(uint8_t)(1U<<DBENB0)
#define    DBENB1          1       /* DeBounce Enable Port B Pin 1 */
#define    BM_DBENB1	(uint8_t)(1U<<DBENB1)
#define    DBENB2          2       /* DeBounce Enable Port B Pin 2 */
#define    BM_DBENB2	(uint8_t)(1U<<DBENB2)
#define    DBENB3          3       /* DeBounce Enable Port B Pin 3 */
#define    BM_DBENB3	(uint8_t)(1U<<DBENB3)
#define    DBENB4          4       /* DeBounce Enable Port B Pin 4 */
#define    BM_DBENB4	(uint8_t)(1U<<DBENB4)
#define    DBENB5          5       /* DeBounce Enable Port B Pin 5 */
#define    BM_DBENB5	(uint8_t)(1U<<DBENB5)
#define    DBENB6          6       /* DeBounce Enable Port B Pin 6 */
#define    BM_DBENB6	(uint8_t)(1U<<DBENB6)
#define    DBENB7          7       /* DeBounce Enable Port B Pin 7 */
#define    BM_DBENB7	(uint8_t)(1U<<DBENB7)

/* DBENC - DeBounce Enable Port C */
#define    DBENC0          0       /* DeBounce Enable Port C Pin 0 */
#define    BM_DBENC0	(uint8_t)(1U<<DBENC0)
#define    DBENC1          1       /* DeBounce Enable Port C Pin 1 */
#define    BM_DBENC1	(uint8_t)(1U<<DBENC1)
#define    DBENC2          2       /* DeBounce Enable Port C Pin 2 */
#define    BM_DBENC2	(uint8_t)(1U<<DBENC2)

/* DBEND - DeBounce Enable Port D */
#define    DBEND0          0       /* DeBounce Enable Port D Pin 0 */
#define    BM_DBEND0	(uint8_t)(1U<<DBEND0)
#define    DBEND1          1       /* DeBounce Enable Port D Pin 1 */
#define    BM_DBEND1	(uint8_t)(1U<<DBEND1)
#define    DBEND2          2       /* DeBounce Enable Port D Pin 2 */
#define    BM_DBEND2	(uint8_t)(1U<<DBEND2)
#define    DBEND3          3       /* DeBounce Enable Port D Pin 3 */
#define    BM_DBEND3	(uint8_t)(1U<<DBEND3)
#define    DBEND4          4       /* DeBounce Enable Port D Pin 4 */
#define    BM_DBEND4	(uint8_t)(1U<<DBEND4)
#define    DBEND5          5       /* DeBounce Enable Port D Pin 5 */
#define    BM_DBEND5	(uint8_t)(1U<<DBEND5)
#define    DBEND6          6       /* DeBounce Enable Port D Pin 6 */
#define    BM_DBEND6	(uint8_t)(1U<<DBEND6)
#define    DBEND7          7       /* DeBounce Enable Port D Pin 7 */
#define    BM_DBEND7	(uint8_t)(1U<<DBEND7)

/* DBTC - Debounce Timer Compare Register */
#define    DBTC0           0       /* Debounce Timer Compare bit 0 */
#define    BM_DBTC0	(uint8_t)(1U<<DBTC0)
#define    DBTC1           1       /* Debounce Timer Compare bit 1 */
#define    BM_DBTC1	(uint8_t)(1U<<DBTC1)
#define    DBTC2           2       /* Debounce Timer Compare bit 2 */
#define    BM_DBTC2	(uint8_t)(1U<<DBTC2)
#define    DBTC3           3       /* Debounce Timer Compare bit 3 */
#define    BM_DBTC3	(uint8_t)(1U<<DBTC3)
#define    DBTC4           4       /* Debounce Timer Compare bit 4 */
#define    BM_DBTC4	(uint8_t)(1U<<DBTC4)
#define    DBTC5           5       /* Debounce Timer Compare bit 5 */
#define    BM_DBTC5	(uint8_t)(1U<<DBTC5)
#define    DBTC6           6       /* Debounce Timer Compare bit 6 */
#define    BM_DBTC6	(uint8_t)(1U<<DBTC6)
#define    DBTC7           7       /* Debounce Timer Compare bit 7 */
#define    BM_DBTC7	(uint8_t)(1U<<DBTC7)


/* ***** DEBUG ************************ */
/* DBGSW - Debugging Support Switch */
#define    DBGGS0          0       /* Debugging Support Group Select bit 0 */
#define    BM_DBGGS0	(uint8_t)(1U<<DBGGS0)
#define    DBGGS1          1       /* Debugging Support Group Select bit 1 */
#define    BM_DBGGS1	(uint8_t)(1U<<DBGGS1)
#define    DBGGS2          2       /* Debugging Support Group Select bit 2 */
#define    BM_DBGGS2	(uint8_t)(1U<<DBGGS2)
#define    DBGGS3          3       /* Debugging Support Group Select bit 3 */
#define    BM_DBGGS3	(uint8_t)(1U<<DBGGS3)
#define    CPBFOS0         4       /* CPU Busy Flag Output Select bit 0 */
#define    BM_CPBFOS0	(uint8_t)(1U<<CPBFOS0)
#define    CPBFOS1         5       /* CPU Busy Flag Output Select bit 1 */
#define    BM_CPBFOS1	(uint8_t)(1U<<CPBFOS1)
#define    CPBF            6       /* CPU Busy Flag */
#define    BM_CPBF	(uint8_t)(1U<<CPBF)
#define    ATEST           7       /* Analog Test Pins Enable */
#define    BM_ATEST	(uint8_t)(1U<<ATEST)

/* DBONDR - Downbond Test Register */
#define    BBESD           0       /* Bond BBESD */
#define    BM_BBESD	(uint8_t)(1U<<BBESD)
#define    AGND_BB         1       /* Bond AGND BB */
#define    BM_AGND_BB	(uint8_t)(1U<<AGND_BB)
#define    ISO_GND         2       /* Bond ISO SUBS3 GND */
#define    BM_ISO_GND	(uint8_t)(1U<<ISO_GND)
#define    AGND_LF         3       /* Bond AGND LF */
#define    BM_AGND_LF	(uint8_t)(1U<<AGND_LF)
#define    BTEST4          4       /* Bond Test 4 */
#define    BM_BTEST4	(uint8_t)(1U<<BTEST4)
#define    BTEST5          5       /* Bond Test 5 */
#define    BM_BTEST5	(uint8_t)(1U<<BTEST5)
#define    BTEST6          6       /* Bond Test 6 */
#define    BM_BTEST6	(uint8_t)(1U<<BTEST6)

/* TRCDR - Trace Data Register */
#define    TRCDR0          0       /*  */
#define    BM_TRCDR0	(uint8_t)(1U<<TRCDR0)
#define    TRCDR1          1       /*  */
#define    BM_TRCDR1	(uint8_t)(1U<<TRCDR1)
#define    TRCDR2          2       /*  */
#define    BM_TRCDR2	(uint8_t)(1U<<TRCDR2)
#define    TRCDR3          3       /*  */
#define    BM_TRCDR3	(uint8_t)(1U<<TRCDR3)
#define    TRCDR4          4       /*  */
#define    BM_TRCDR4	(uint8_t)(1U<<TRCDR4)
#define    TRCDR5          5       /*  */
#define    BM_TRCDR5	(uint8_t)(1U<<TRCDR5)
#define    TRCDR6          6       /*  */
#define    BM_TRCDR6	(uint8_t)(1U<<TRCDR6)
#define    TRCDR7          7       /*  */
#define    BM_TRCDR7	(uint8_t)(1U<<TRCDR7)

/* TRCIDH - Trace ID Register High byte */
#define    TRCID8          0       /*  */
#define    BM_TRCID8	(uint8_t)(1U<<TRCID8)
#define    TRCID9          1       /*  */
#define    BM_TRCID9	(uint8_t)(1U<<TRCID9)
#define    TRCID10         2       /*  */
#define    BM_TRCID10	(uint8_t)(1U<<TRCID10)
#define    TRCID11         3       /*  */
#define    BM_TRCID11	(uint8_t)(1U<<TRCID11)
#define    TRCID12         4       /*  */
#define    BM_TRCID12	(uint8_t)(1U<<TRCID12)
#define    TRCID13         5       /*  */
#define    BM_TRCID13	(uint8_t)(1U<<TRCID13)
#define    TRCID14         6       /*  */
#define    BM_TRCID14	(uint8_t)(1U<<TRCID14)
#define    TRCID15         7       /*  */
#define    BM_TRCID15	(uint8_t)(1U<<TRCID15)

/* TRCIDL - Trace ID Register Low byte */
#define    TRCID0          0       /*  */
#define    BM_TRCID0	(uint8_t)(1U<<TRCID0)
#define    TRCID1          1       /*  */
#define    BM_TRCID1	(uint8_t)(1U<<TRCID1)
#define    TRCID2          2       /*  */
#define    BM_TRCID2	(uint8_t)(1U<<TRCID2)
#define    TRCID3          3       /*  */
#define    BM_TRCID3	(uint8_t)(1U<<TRCID3)
#define    TRCID4          4       /*  */
#define    BM_TRCID4	(uint8_t)(1U<<TRCID4)
#define    TRCID5          5       /*  */
#define    BM_TRCID5	(uint8_t)(1U<<TRCID5)
#define    TRCID6          6       /*  */
#define    BM_TRCID6	(uint8_t)(1U<<TRCID6)
#define    TRCID7          7       /*  */
#define    BM_TRCID7	(uint8_t)(1U<<TRCID7)


/* ***** DFIFO ************************ */
/* DFC - Data FIFO Configuration Register */
#define    DFFLC0          0       /* Data FIFO Fill-Level Configuration bit 0 */
#define    BM_DFFLC0	(uint8_t)(1U<<DFFLC0)
#define    DFFLC1          1       /* Data FIFO Fill-Level Configuration bit 1 */
#define    BM_DFFLC1	(uint8_t)(1U<<DFFLC1)
#define    DFFLC2          2       /* Data FIFO Fill-Level Configuration bit 2 */
#define    BM_DFFLC2	(uint8_t)(1U<<DFFLC2)
#define    DFFLC3          3       /* Data FIFO Fill-Level Configuration bit 3 */
#define    BM_DFFLC3	(uint8_t)(1U<<DFFLC3)
#define    DFFLC4          4       /* Data FIFO Fill-Level Configuration bit 4 */
#define    BM_DFFLC4	(uint8_t)(1U<<DFFLC4)
#define    DFFLC5          5       /* Data FIFO Fill-Level Configuration bit 5 */
#define    BM_DFFLC5	(uint8_t)(1U<<DFFLC5)
#define    DFDRA           7       /* Data FIFO Direct Read Access Operational Mode */
#define    BM_DFDRA	(uint8_t)(1U<<DFDRA)

/* DFI - Data FIFO Interrupt Mask Register */
#define    DFFLIM          0       /* Data FIFO Fill-level Interrupt Mask */
#define    BM_DFFLIM	(uint8_t)(1U<<DFFLIM)
#define    DFERIM          1       /* Data FIFO Error Interrupt Mask */
#define    BM_DFERIM	(uint8_t)(1U<<DFERIM)

/* DFL - Data FIFO Fill Level Register */
#define    DFFLS0          0       /* Data FIFO Fill Level Status bit 0 */
#define    BM_DFFLS0	(uint8_t)(1U<<DFFLS0)
#define    DFFLS1          1       /* Data FIFO Fill Level Status bit 1 */
#define    BM_DFFLS1	(uint8_t)(1U<<DFFLS1)
#define    DFFLS2          2       /* Data FIFO Fill Level Status bit 2 */
#define    BM_DFFLS2	(uint8_t)(1U<<DFFLS2)
#define    DFFLS3          3       /* Data FIFO Fill Level Status bit 3 */
#define    BM_DFFLS3	(uint8_t)(1U<<DFFLS3)
#define    DFFLS4          4       /* Data FIFO Fill Level Status bit 4 */
#define    BM_DFFLS4	(uint8_t)(1U<<DFFLS4)
#define    DFFLS5          5       /* Data FIFO Fill Level Status bit 5 */
#define    BM_DFFLS5	(uint8_t)(1U<<DFFLS5)
#define    DFCLR           7       /* Data FIFO Clear */
#define    BM_DFCLR	(uint8_t)(1U<<DFCLR)

/* DFRP - Data FIFO Read Pointer */
#define    DFRP0           0       /* Data FIFO Read Pointer bit 0 */
#define    BM_DFRP0	(uint8_t)(1U<<DFRP0)
#define    DFRP1           1       /* Data FIFO Read Pointer bit 1 */
#define    BM_DFRP1	(uint8_t)(1U<<DFRP1)
#define    DFRP2           2       /* Data FIFO Read Pointer bit 2 */
#define    BM_DFRP2	(uint8_t)(1U<<DFRP2)
#define    DFRP3           3       /* Data FIFO Read Pointer bit 3 */
#define    BM_DFRP3	(uint8_t)(1U<<DFRP3)
#define    DFRP4           4       /* Data FIFO Read Pointer bit 4 */
#define    BM_DFRP4	(uint8_t)(1U<<DFRP4)
#define    DFRP5           5       /* Data FIFO Read Pointer bit 5 */
#define    BM_DFRP5	(uint8_t)(1U<<DFRP5)

/* DFS - Data FIFO Status Register */
#define    DFFLRF          0       /* Data FIFO Fill-Level Reached Status Flag */
#define    BM_DFFLRF	(uint8_t)(1U<<DFFLRF)
#define    DFUFL           1       /* Data FIFO Underflow Flag */
#define    BM_DFUFL	(uint8_t)(1U<<DFUFL)
#define    DFOFL           2       /* Data FIFO Overflow Flag */
#define    BM_DFOFL	(uint8_t)(1U<<DFOFL)

/* DFTLH - Data FIFO Telegram Length High Byte */
#define    DFTL8           0       /* Data FIFO Telegram Length bit 8 */
#define    BM_DFTL8	(uint8_t)(1U<<DFTL8)
#define    DFTL9           1       /* Data FIFO Telegram Length bit 9 */
#define    BM_DFTL9	(uint8_t)(1U<<DFTL9)
#define    DFTL10          2       /* Data FIFO Telegram Length bit 10 */
#define    BM_DFTL10	(uint8_t)(1U<<DFTL10)
#define    DFTL11          3       /* Data FIFO Telegram Length bit 11 */
#define    BM_DFTL11	(uint8_t)(1U<<DFTL11)

/* DFTLL - Data FIFO Telegram Length Low Byte */
#define    DFTL0           0       /* Data FIFO Telegram Length bit 0 */
#define    BM_DFTL0	(uint8_t)(1U<<DFTL0)
#define    DFTL1           1       /* Data FIFO Telegram Length bit 1 */
#define    BM_DFTL1	(uint8_t)(1U<<DFTL1)
#define    DFTL2           2       /* Data FIFO Telegram Length bit 2 */
#define    BM_DFTL2	(uint8_t)(1U<<DFTL2)
#define    DFTL3           3       /* Data FIFO Telegram Length bit 3 */
#define    BM_DFTL3	(uint8_t)(1U<<DFTL3)
#define    DFTL4           4       /* Data FIFO Telegram Length bit 4 */
#define    BM_DFTL4	(uint8_t)(1U<<DFTL4)
#define    DFTL5           5       /* Data FIFO Telegram Length bit 5 */
#define    BM_DFTL5	(uint8_t)(1U<<DFTL5)
#define    DFTL6           6       /* Data FIFO Telegram Length bit 6 */
#define    BM_DFTL6	(uint8_t)(1U<<DFTL6)
#define    DFTL7           7       /* Data FIFO Telegram Length bit 7 */
#define    BM_DFTL7	(uint8_t)(1U<<DFTL7)

/* DFWP - Data FIFO Write Pointer */
#define    DFWP0           0       /* Data FIFO Write Pointer bit 0 */
#define    BM_DFWP0	(uint8_t)(1U<<DFWP0)
#define    DFWP1           1       /* Data FIFO Write Pointer bit 1 */
#define    BM_DFWP1	(uint8_t)(1U<<DFWP1)
#define    DFWP2           2       /* Data FIFO Write Pointer bit 2 */
#define    BM_DFWP2	(uint8_t)(1U<<DFWP2)
#define    DFWP3           3       /* Data FIFO Write Pointer bit 3 */
#define    BM_DFWP3	(uint8_t)(1U<<DFWP3)
#define    DFWP4           4       /* Data FIFO Write Pointer bit 4 */
#define    BM_DFWP4	(uint8_t)(1U<<DFWP4)
#define    DFWP5           5       /* Data FIFO Write Pointer bit 5 */
#define    BM_DFWP5	(uint8_t)(1U<<DFWP5)


/* ***** EEPROM *********************** */
/* EEARH - EEPROM Address Register High Byte */
#define    EEAR8           0       /* EEPROM Read/Write Address bit 8 */
#define    BM_EEAR8	(uint8_t)(1U<<EEAR8)
#define    EEAR9           1       /* EEPROM Read/Write Address bit 9 */
#define    BM_EEAR9	(uint8_t)(1U<<EEAR9)
#define    EEAR10          2       /* EEPROM Read/Write Address bit 10 */
#define    BM_EEAR10	(uint8_t)(1U<<EEAR10)
#define    EEAR11          3       /* EEPROM Read/Write Address bit 11 */
#define    BM_EEAR11	(uint8_t)(1U<<EEAR11)

/* EEARL - EEPROM Address Register Low Byte */
#define    EEAR0           0       /* EEPROM Read/Write Address bit 0 */
#define    BM_EEAR0	(uint8_t)(1U<<EEAR0)
#define    EEAR1           1       /* EEPROM Read/Write Address bit 1 */
#define    BM_EEAR1	(uint8_t)(1U<<EEAR1)
#define    EEAR2           2       /* EEPROM Read/Write Address bit 2 */
#define    BM_EEAR2	(uint8_t)(1U<<EEAR2)
#define    EEAR3           3       /* EEPROM Read/Write Address bit 3 */
#define    BM_EEAR3	(uint8_t)(1U<<EEAR3)
#define    EEAR4           4       /* EEPROM Read/Write Address bit 4 */
#define    BM_EEAR4	(uint8_t)(1U<<EEAR4)
#define    EEAR5           5       /* EEPROM Read/Write Address bit 5 */
#define    BM_EEAR5	(uint8_t)(1U<<EEAR5)
#define    EEAR6           6       /* EEPROM Read/Write Address bit 6 */
#define    BM_EEAR6	(uint8_t)(1U<<EEAR6)
#define    EEAR7           7       /* EEPROM Read/Write Address bit 7 */
#define    BM_EEAR7	(uint8_t)(1U<<EEAR7)

/* EECR - EEPROM Control Register */
#define    EERE            0       /* EEPROM Read Enable */
#define    BM_EERE	(uint8_t)(1U<<EERE)
#define    EEWE            1       /* EEPROM Write Enable */
#define    BM_EEWE	(uint8_t)(1U<<EEWE)
#define    EEMWE           2       /* EEPROM Master Write Enable */
#define    BM_EEMWE	(uint8_t)(1U<<EEMWE)
#define    EERIE           3       /* EEPROM Ready Interrupt Enable */
#define    BM_EERIE	(uint8_t)(1U<<EERIE)
#define    EEPM0           4       /* EEPROM Programming Mode bit 0 */
#define    BM_EEPM0	(uint8_t)(1U<<EEPM0)
#define    EEPM1           5       /* EEPROM Programming Mode bit 1 */
#define    BM_EEPM1	(uint8_t)(1U<<EEPM1)
#define    EEPAGE          6       /* EEPROM page access (multiple bytes access mode) */
#define    BM_EEPAGE	(uint8_t)(1U<<EEPAGE)
#define    NVMBSY          7       /* Non-volatile memory busy */
#define    BM_NVMBSY	(uint8_t)(1U<<NVMBSY)

/* EECR2 - EEPROM Control Register 2 */
#define    EEBRE           0       /* EEPROM Burst Read Enable */
#define    BM_EEBRE	(uint8_t)(1U<<EEBRE)
#define    E2CIM           1       /* EEPROM Error Code Correction Interrupt Mask */
#define    BM_E2CIM	(uint8_t)(1U<<E2CIM)
#define    E2AVF           5       /* EEPROM Error Code Correction Interrupt Mask */
#define    BM_E2AVF	(uint8_t)(1U<<E2AVF)
#define    E2FF            6       /* EEPROM error Fault Flag bit */
#define    BM_E2FF	(uint8_t)(1U<<E2FF)
#define    E2CF            7       /* EEPROM Error Correction Code Flag */
#define    BM_E2CF	(uint8_t)(1U<<E2CF)

/* EEPR - EEPROM Protection Register */
#define    EEAP0           0       /* EEPROM Access Protection Register bit 0 */
#define    BM_EEAP0	(uint8_t)(1U<<EEAP0)
#define    EEAP1           1       /* EEPROM Access Protection Register bit 1 */
#define    BM_EEAP1	(uint8_t)(1U<<EEAP1)
#define    EEAP2           2       /* EEPROM Access Protection Register bit 2 */
#define    BM_EEAP2	(uint8_t)(1U<<EEAP2)
#define    EEAP3           3       /* EEPROM Access Protection Register bit 3 */
#define    BM_EEAP3	(uint8_t)(1U<<EEAP3)


/* ***** FE *************************** */
/* FEALR - Front-End Antenna Level Detector Range */
#define    RNGE0           0       /* Range of the ANT_TUNE level detector bit 0 */
#define    BM_RNGE0	(uint8_t)(1U<<RNGE0)
#define    RNGE1           1       /* Range of the ANT_TUNE level detector bit 1 */
#define    BM_RNGE1	(uint8_t)(1U<<RNGE1)

/* FEANT - Front-End Antenna */
#define    LVLC0           0       /* antenna signal LeVeL bit 0 */
#define    BM_LVLC0	(uint8_t)(1U<<LVLC0)
#define    LVLC1           1       /* antenna signal LeVeL bit 1 */
#define    BM_LVLC1	(uint8_t)(1U<<LVLC1)
#define    LVLC2           2       /* antenna signal LeVeL bit 2 */
#define    BM_LVLC2	(uint8_t)(1U<<LVLC2)
#define    LVLC3           3       /* antenna signal LeVeL bit 3 */
#define    BM_LVLC3	(uint8_t)(1U<<LVLC3)

/* FEAT - Front-End Antenna Tuning */
#define    ANTN0           0       /* Antenna Tuning bit 0 */
#define    BM_ANTN0	(uint8_t)(1U<<ANTN0)
#define    ANTN1           1       /* Antenna Tuning bit 1 */
#define    BM_ANTN1	(uint8_t)(1U<<ANTN1)
#define    ANTN2           2       /* Antenna Tuning bit 2 */
#define    BM_ANTN2	(uint8_t)(1U<<ANTN2)
#define    ANTN3           3       /* Antenna Tuning bit 3 */
#define    BM_ANTN3	(uint8_t)(1U<<ANTN3)

/* FEBIA - Front-End IF Amplifier BIAS */
#define    BIAM0           0       /* Bias amplifier bit 0 */
#define    BM_BIAM0	(uint8_t)(1U<<BIAM0)
#define    BIAM1           1       /* Bias amplifier bit 1 */
#define    BM_BIAM1	(uint8_t)(1U<<BIAM1)
#define    BIAM2           2       /* Bias amplifier bit 2 */
#define    BM_BIAM2	(uint8_t)(1U<<BIAM2)
#define    BIAM3           3       /* Bias amplifier bit 3 */
#define    BM_BIAM3	(uint8_t)(1U<<BIAM3)
#define    FEBIA_R4        4       /*  */
#define    BM_FEBIA_R4	(uint8_t)(1U<<FEBIA_R4)
#define    FEBIA_R5        5       /*  */
#define    BM_FEBIA_R5	(uint8_t)(1U<<FEBIA_R5)
#define    FEBIA_R6        6       /*  */
#define    BM_FEBIA_R6	(uint8_t)(1U<<FEBIA_R6)
#define    IFAEN           7       /* IF Amplifier Enable */
#define    BM_IFAEN	(uint8_t)(1U<<IFAEN)

/* FEBT - Front-End RC Tuning Register */
#define    CTN20           0       /* Capacitor Tuning bit 0 */
#define    BM_CTN20	(uint8_t)(1U<<CTN20)
#define    CTN21           1       /* Capacitor Tuning bit 1 */
#define    BM_CTN21	(uint8_t)(1U<<CTN21)
#define    RTN20           2       /* Resistor Tuning bit 0 */
#define    BM_RTN20	(uint8_t)(1U<<RTN20)
#define    RTN21           3       /* Resistor Tuning bit 1 */
#define    BM_RTN21	(uint8_t)(1U<<RTN21)

/* FECR - Front-End Control Register */
#define    LBNHB           0       /* Select Low-Band Not High-Band */
#define    BM_LBNHB	(uint8_t)(1U<<LBNHB)
#define    S4N3            1       /* Select 433 Not 315MHz Band */
#define    BM_S4N3	(uint8_t)(1U<<S4N3)
#define    ANDP            2       /* Antenna Damping */
#define    BM_ANDP	(uint8_t)(1U<<ANDP)
#define    ADHS            3       /* ADC High Sample Rate */
#define    BM_ADHS	(uint8_t)(1U<<ADHS)
#define    PLCKG           4       /* PLL Lock Detect Gate */
#define    BM_PLCKG	(uint8_t)(1U<<PLCKG)
#define    ANPS            5       /* ASK Not DPSK Switch */
#define    BM_ANPS	(uint8_t)(1U<<ANPS)

/* FEEN1 - Front-End Enable Register 1 */
#define    PLEN            0       /* PLL enable */
#define    BM_PLEN	(uint8_t)(1U<<PLEN)
#define    PLCAL           1       /* PLL calibration mode */
#define    BM_PLCAL	(uint8_t)(1U<<PLCAL)
#define    XTOEN           2       /* Cristal Oscillator enable */
#define    BM_XTOEN	(uint8_t)(1U<<XTOEN)
#define    LNAEN           3       /* Low Noise Amplifier enable */
#define    BM_LNAEN	(uint8_t)(1U<<LNAEN)
#define    ADEN            4       /* Analog Digital Converter enable */
#define    BM_ADEN	(uint8_t)(1U<<ADEN)
#define    ADCLK           5       /* ADC Clock enable */
#define    BM_ADCLK	(uint8_t)(1U<<ADCLK)
#define    PLSP1           6       /* PLL Speedup 1 */
#define    BM_PLSP1	(uint8_t)(1U<<PLSP1)
#define    ATEN            7       /* Antenna Tuning Enable */
#define    BM_ATEN	(uint8_t)(1U<<ATEN)

/* FEEN2 - Front-End Enable Register 2 */
#define    SDRX            0       /* Single Pole Double Throw (SPDT) Switch RX */
#define    BM_SDRX	(uint8_t)(1U<<SDRX)
#define    SDTX            1       /* Single Pole Double Throw (SPDT) Switch TX */
#define    BM_SDTX	(uint8_t)(1U<<SDTX)
#define    PAEN            2       /* Power Amplifier enable */
#define    BM_PAEN	(uint8_t)(1U<<PAEN)
#define    TMPM            3       /* Temperature measurement */
#define    BM_TMPM	(uint8_t)(1U<<TMPM)
#define    PLPEN           4       /* PLL Post En IQ divider */
#define    BM_PLPEN	(uint8_t)(1U<<PLPEN)
#define    XTPEN           5       /* XTO VPump enable */
#define    BM_XTPEN	(uint8_t)(1U<<XTPEN)
#define    CPBIA           6       /* Cap Array Bias */
#define    BM_CPBIA	(uint8_t)(1U<<CPBIA)
#define    FEEN2_R7        7       /*  */
#define    BM_FEEN2_R7	(uint8_t)(1U<<FEEN2_R7)

/* FELNA - Front-End LNA Bias Register */
#define    LBH0            0       /* LNA Bias High band bit 0 */
#define    BM_LBH0	(uint8_t)(1U<<LBH0)
#define    LBH1            1       /* LNA Bias High band bit 1 */
#define    BM_LBH1	(uint8_t)(1U<<LBH1)
#define    LBH2            2       /* LNA Bias High band bit 2 */
#define    BM_LBH2	(uint8_t)(1U<<LBH2)
#define    LBH3            3       /* LNA Bias High band bit 3 */
#define    BM_LBH3	(uint8_t)(1U<<LBH3)
#define    LBL0            4       /* LNA Bias Low band bit 0 */
#define    BM_LBL0	(uint8_t)(1U<<LBL0)
#define    LBL1            5       /* LNA Bias Low band bit 1 */
#define    BM_LBL1	(uint8_t)(1U<<LBL1)
#define    LBL2            6       /* LNA Bias Low band bit 2 */
#define    BM_LBL2	(uint8_t)(1U<<LBL2)
#define    LBL3            7       /* LNA Bias Low band bit 3 */
#define    BM_LBL3	(uint8_t)(1U<<LBL3)

/* FEMS - Front-End Main and Swallow Control Register */
#define    PLLS0           0       /* PLL Swallow bit 0 */
#define    BM_PLLS0	(uint8_t)(1U<<PLLS0)
#define    PLLS1           1       /* PLL Swallow bit 1 */
#define    BM_PLLS1	(uint8_t)(1U<<PLLS1)
#define    PLLS2           2       /* PLL Swallow bit 2 */
#define    BM_PLLS2	(uint8_t)(1U<<PLLS2)
#define    PLLS3           3       /* PLL Swallow bit 3 */
#define    BM_PLLS3	(uint8_t)(1U<<PLLS3)
#define    PLLM0           4       /* PLL Mode bit 0 */
#define    BM_PLLM0	(uint8_t)(1U<<PLLM0)
#define    PLLM1           5       /* PLL Mode bit 1 */
#define    BM_PLLM1	(uint8_t)(1U<<PLLM1)
#define    PLLM2           6       /* PLL Mode bit 2 */
#define    BM_PLLM2	(uint8_t)(1U<<PLLM2)
#define    PLLM3           7       /* PLL Mode bit 3 */
#define    BM_PLLM3	(uint8_t)(1U<<PLLM3)

/* FEPAC - Front-End Power Amplifier Control Register */
#define    PACR0           0       /* Power Amplifier Control Register bit 0 */
#define    BM_PACR0	(uint8_t)(1U<<PACR0)
#define    PACR1           1       /* Power Amplifier Control Register bit 1 */
#define    BM_PACR1	(uint8_t)(1U<<PACR1)
#define    PACR2           2       /* Power Amplifier Control Register bit 2 */
#define    BM_PACR2	(uint8_t)(1U<<PACR2)
#define    PACR3           3       /* Power Amplifier Control Register bit 3 */
#define    BM_PACR3	(uint8_t)(1U<<PACR3)
#define    PACR4           4       /* Power Amplifier Control Register bit 4 */
#define    BM_PACR4	(uint8_t)(1U<<PACR4)
#define    PACR5           5       /* Power Amplifier Control Register bit 5 */
#define    BM_PACR5	(uint8_t)(1U<<PACR5)
#define    PACR6           6       /* Power Amplifier Control Register bit 6 */
#define    BM_PACR6	(uint8_t)(1U<<PACR6)
#define    PACR7           7       /* Power Amplifier Control Register bit 7 */
#define    BM_PACR7	(uint8_t)(1U<<PACR7)

/* FESR - Front-End Status Register */
#define    LBSAT           0       /* LNA Low band saturated */
#define    BM_LBSAT	(uint8_t)(1U<<LBSAT)
#define    HBSAT           1       /* LNA High band saturated */
#define    BM_HBSAT	(uint8_t)(1U<<HBSAT)
#define    XRDY            2       /* XTO ready */
#define    BM_XRDY	(uint8_t)(1U<<XRDY)
#define    PLCK            3       /* PLL locked */
#define    BM_PLCK	(uint8_t)(1U<<PLCK)
#define    ANTS            4       /* Antenna saturated */
#define    BM_ANTS	(uint8_t)(1U<<ANTS)

/* FETD - Front-End Test Data Register */
#define    FETM0           0       /*  */
#define    BM_FETM0	(uint8_t)(1U<<FETM0)
#define    FETM1           1       /*  */
#define    BM_FETM1	(uint8_t)(1U<<FETM1)
#define    FETM2           2       /*  */
#define    BM_FETM2	(uint8_t)(1U<<FETM2)
#define    FETM3           3       /*  */
#define    BM_FETM3	(uint8_t)(1U<<FETM3)
#define    FETM4           4       /*  */
#define    BM_FETM4	(uint8_t)(1U<<FETM4)
#define    FETM5           5       /*  */
#define    BM_FETM5	(uint8_t)(1U<<FETM5)
#define    FETM6           6       /*  */
#define    BM_FETM6	(uint8_t)(1U<<FETM6)
#define    FETM7           7       /*  */
#define    BM_FETM7	(uint8_t)(1U<<FETM7)

/* FETE1 - Front-End Test Enable Register 1 */
#define    ADCT            0       /*  */
#define    BM_ADCT	(uint8_t)(1U<<ADCT)
#define    XTOT            1       /*  */
#define    BM_XTOT	(uint8_t)(1U<<XTOT)
#define    LNLT            2       /*  */
#define    BM_LNLT	(uint8_t)(1U<<LNLT)
#define    LNHT            3       /*  */
#define    BM_LNHT	(uint8_t)(1U<<LNHT)
#define    PATE            4       /*  */
#define    BM_PATE	(uint8_t)(1U<<PATE)
#define    AMPT            5       /*  */
#define    BM_AMPT	(uint8_t)(1U<<AMPT)
#define    VCOT            6       /*  */
#define    BM_VCOT	(uint8_t)(1U<<VCOT)
#define    ANTN            7       /*  */
#define    BM_ANTN	(uint8_t)(1U<<ANTN)

/* FETE2 - Front-End Test Enable Register 2 */
#define    RCCT            0       /*  */
#define    BM_RCCT	(uint8_t)(1U<<RCCT)
#define    PPFT            1       /*  */
#define    BM_PPFT	(uint8_t)(1U<<PPFT)
#define    LFT             2       /*  */
#define    BM_LFT	(uint8_t)(1U<<LFT)
#define    CPT             3       /*  */
#define    BM_CPT	(uint8_t)(1U<<CPT)
#define    PFDT            4       /*  */
#define    BM_PFDT	(uint8_t)(1U<<PFDT)
#define    DADCT           5       /*  */
#define    BM_DADCT	(uint8_t)(1U<<DADCT)
#define    PRET            6       /*  */
#define    BM_PRET	(uint8_t)(1U<<PRET)
#define    SWALT           7       /*  */
#define    BM_SWALT	(uint8_t)(1U<<SWALT)

/* FETE3 - Front-End Test Enable Register 3 */
#define    BIOUT           0       /*  */
#define    BM_BIOUT	(uint8_t)(1U<<BIOUT)
#define    RMPTST          1       /*  */
#define    BM_RMPTST	(uint8_t)(1U<<RMPTST)
#define    FETE3_R2        2       /*  */
#define    BM_FETE3_R2	(uint8_t)(1U<<FETE3_R2)
#define    FETE3_R3        3       /*  */
#define    BM_FETE3_R3	(uint8_t)(1U<<FETE3_R3)
#define    FETE3_R4        4       /*  */
#define    BM_FETE3_R4	(uint8_t)(1U<<FETE3_R4)
#define    FETE3_R5        5       /*  */
#define    BM_FETE3_R5	(uint8_t)(1U<<FETE3_R5)
#define    FETE3_R6        6       /*  */
#define    BM_FETE3_R6	(uint8_t)(1U<<FETE3_R6)
#define    FETE3_R7        7       /*  */
#define    BM_FETE3_R7	(uint8_t)(1U<<FETE3_R7)

/* FETN4 - Front-End RC Tuning 4bit Register */
#define    CTN40           0       /* 4 bit Capacitor Tuning bit 0 */
#define    BM_CTN40	(uint8_t)(1U<<CTN40)
#define    CTN41           1       /* 4 bit Capacitor Tuning bit 1 */
#define    BM_CTN41	(uint8_t)(1U<<CTN41)
#define    CTN42           2       /* 4 bit Capacitor Tuning bit 2 */
#define    BM_CTN42	(uint8_t)(1U<<CTN42)
#define    CTN43           3       /* 4 bit Capacitor Tuning bit 3 */
#define    BM_CTN43	(uint8_t)(1U<<CTN43)
#define    RTN40           4       /* 4 bit Resistor Tuning bit 0 */
#define    BM_RTN40	(uint8_t)(1U<<RTN40)
#define    RTN41           5       /* 4 bit Resistor Tuning bit 1 */
#define    BM_RTN41	(uint8_t)(1U<<RTN41)
#define    RTN42           6       /* 4 bit Resistor Tuning bit 2 */
#define    BM_RTN42	(uint8_t)(1U<<RTN42)
#define    RTN43           7       /* 4 bit Resistor Tuning bit 3 */
#define    BM_RTN43	(uint8_t)(1U<<RTN43)

/* FEVCO - Front-End VCO and PLL control */
#define    CPCC0           0       /* Charge pump current control bit 0 */
#define    BM_CPCC0	(uint8_t)(1U<<CPCC0)
#define    CPCC1           1       /* Charge pump current control bit 1 */
#define    BM_CPCC1	(uint8_t)(1U<<CPCC1)
#define    CPCC2           2       /* Charge pump current control bit 2 */
#define    BM_CPCC2	(uint8_t)(1U<<CPCC2)
#define    CPCC3           3       /* Charge pump current control bit 3 */
#define    BM_CPCC3	(uint8_t)(1U<<CPCC3)
#define    VCOB0           4       /* VCO Bias bit 0 */
#define    BM_VCOB0	(uint8_t)(1U<<VCOB0)
#define    VCOB1           5       /* VCO Bias bit 1 */
#define    BM_VCOB1	(uint8_t)(1U<<VCOB1)
#define    VCOB2           6       /* VCO Bias bit 2 */
#define    BM_VCOB2	(uint8_t)(1U<<VCOB2)
#define    VCOB3           7       /* VCO Bias bit 3 */
#define    BM_VCOB3	(uint8_t)(1U<<VCOB3)

/* FEVCT - Front-End VCO Tuning Register */
#define    FEVCT0          0       /* Front-End VCO Tuning Register bit 0 */
#define    BM_FEVCT0	(uint8_t)(1U<<FEVCT0)
#define    FEVCT1          1       /* Front-End VCO Tuning Register bit 1 */
#define    BM_FEVCT1	(uint8_t)(1U<<FEVCT1)
#define    FEVCT2          2       /* Front-End VCO Tuning Register bit 2 */
#define    BM_FEVCT2	(uint8_t)(1U<<FEVCT2)
#define    FEVCT3          3       /* Front-End VCO Tuning Register bit 3 */
#define    BM_FEVCT3	(uint8_t)(1U<<FEVCT3)
#define    FEVCT_R4        4       /*  */
#define    BM_FEVCT_R4	(uint8_t)(1U<<FEVCT_R4)
#define    FEVCT_R5        5       /*  */
#define    BM_FEVCT_R5	(uint8_t)(1U<<FEVCT_R5)
#define    FEVCT_R6        6       /*  */
#define    BM_FEVCT_R6	(uint8_t)(1U<<FEVCT_R6)
#define    FEVCT_R7        7       /*  */
#define    BM_FEVCT_R7	(uint8_t)(1U<<FEVCT_R7)

/* SPARE1 - Front-End Spare Register 1 */
#define    SPARE10         0       /*  */
#define    BM_SPARE10	(uint8_t)(1U<<SPARE10)
#define    SPARE11         1       /*  */
#define    BM_SPARE11	(uint8_t)(1U<<SPARE11)
#define    SPARE12         2       /*  */
#define    BM_SPARE12	(uint8_t)(1U<<SPARE12)
#define    SPARE13         3       /*  */
#define    BM_SPARE13	(uint8_t)(1U<<SPARE13)
#define    SPARE14         4       /*  */
#define    BM_SPARE14	(uint8_t)(1U<<SPARE14)
#define    SPARE15         5       /*  */
#define    BM_SPARE15	(uint8_t)(1U<<SPARE15)
#define    SPARE16         6       /*  */
#define    BM_SPARE16	(uint8_t)(1U<<SPARE16)
#define    SPARE17         7       /*  */
#define    BM_SPARE17	(uint8_t)(1U<<SPARE17)

/* SPARE2 - Front-End Spare Register 2 */
#define    SPARE20         0       /*  */
#define    BM_SPARE20	(uint8_t)(1U<<SPARE20)
#define    SPARE21         1       /*  */
#define    BM_SPARE21	(uint8_t)(1U<<SPARE21)
#define    SPARE22         2       /*  */
#define    BM_SPARE22	(uint8_t)(1U<<SPARE22)
#define    SPARE23         3       /*  */
#define    BM_SPARE23	(uint8_t)(1U<<SPARE23)
#define    SPARE24         4       /*  */
#define    BM_SPARE24	(uint8_t)(1U<<SPARE24)
#define    SPARE25         5       /*  */
#define    BM_SPARE25	(uint8_t)(1U<<SPARE25)
#define    SPARE26         6       /*  */
#define    BM_SPARE26	(uint8_t)(1U<<SPARE26)
#define    SPARE27         7       /*  */
#define    BM_SPARE27	(uint8_t)(1U<<SPARE27)

/* SPARE3 - Front-End Spare Register 3 */
#define    SPARE30         0       /*  */
#define    BM_SPARE30	(uint8_t)(1U<<SPARE30)
#define    SPARE31         1       /*  */
#define    BM_SPARE31	(uint8_t)(1U<<SPARE31)
#define    SPARE32         2       /*  */
#define    BM_SPARE32	(uint8_t)(1U<<SPARE32)
#define    SPARE33         3       /*  */
#define    BM_SPARE33	(uint8_t)(1U<<SPARE33)
#define    SPARE34         4       /*  */
#define    BM_SPARE34	(uint8_t)(1U<<SPARE34)
#define    SPARE35         5       /*  */
#define    BM_SPARE35	(uint8_t)(1U<<SPARE35)
#define    SPARE36         6       /*  */
#define    BM_SPARE36	(uint8_t)(1U<<SPARE36)
#define    SPARE37         7       /*  */
#define    BM_SPARE37	(uint8_t)(1U<<SPARE37)

/* SPARE4 - Front-End Spare Register 4 */
#define    SPARE40         0       /*  */
#define    BM_SPARE40	(uint8_t)(1U<<SPARE40)
#define    SPARE41         1       /*  */
#define    BM_SPARE41	(uint8_t)(1U<<SPARE41)
#define    SPARE42         2       /*  */
#define    BM_SPARE42	(uint8_t)(1U<<SPARE42)
#define    SPARE43         3       /*  */
#define    BM_SPARE43	(uint8_t)(1U<<SPARE43)
#define    SPARE44         4       /*  */
#define    BM_SPARE44	(uint8_t)(1U<<SPARE44)
#define    SPARE45         5       /*  */
#define    BM_SPARE45	(uint8_t)(1U<<SPARE45)
#define    SPARE46         6       /*  */
#define    BM_SPARE46	(uint8_t)(1U<<SPARE46)
#define    SPARE47         7       /*  */
#define    BM_SPARE47	(uint8_t)(1U<<SPARE47)


/* ***** GPIOREGS ********************* */
/* GPIOR0 - General Purpose I/O Register 0 */
#define    GPIOR00         0       /*  */
#define    BM_GPIOR00	(uint8_t)(1U<<GPIOR00)
#define    GPIOR01         1       /*  */
#define    BM_GPIOR01	(uint8_t)(1U<<GPIOR01)
#define    GPIOR02         2       /*  */
#define    BM_GPIOR02	(uint8_t)(1U<<GPIOR02)
#define    GPIOR03         3       /*  */
#define    BM_GPIOR03	(uint8_t)(1U<<GPIOR03)
#define    GPIOR04         4       /*  */
#define    BM_GPIOR04	(uint8_t)(1U<<GPIOR04)
#define    GPIOR05         5       /*  */
#define    BM_GPIOR05	(uint8_t)(1U<<GPIOR05)
#define    GPIOR06         6       /*  */
#define    BM_GPIOR06	(uint8_t)(1U<<GPIOR06)
#define    GPIOR07         7       /*  */
#define    BM_GPIOR07	(uint8_t)(1U<<GPIOR07)

/* GPIOR1 - General Purpose I/O Register 1 */
#define    GPIOR10         0       /*  */
#define    BM_GPIOR10	(uint8_t)(1U<<GPIOR10)
#define    GPIOR11         1       /*  */
#define    BM_GPIOR11	(uint8_t)(1U<<GPIOR11)
#define    GPIOR12         2       /*  */
#define    BM_GPIOR12	(uint8_t)(1U<<GPIOR12)
#define    GPIOR13         3       /*  */
#define    BM_GPIOR13	(uint8_t)(1U<<GPIOR13)
#define    GPIOR14         4       /*  */
#define    BM_GPIOR14	(uint8_t)(1U<<GPIOR14)
#define    GPIOR15         5       /*  */
#define    BM_GPIOR15	(uint8_t)(1U<<GPIOR15)
#define    GPIOR16         6       /*  */
#define    BM_GPIOR16	(uint8_t)(1U<<GPIOR16)
#define    GPIOR17         7       /*  */
#define    BM_GPIOR17	(uint8_t)(1U<<GPIOR17)

/* GPIOR2 - General Purpose I/O Register 2 */
#define    GPIOR20         0       /*  */
#define    BM_GPIOR20	(uint8_t)(1U<<GPIOR20)
#define    GPIOR21         1       /*  */
#define    BM_GPIOR21	(uint8_t)(1U<<GPIOR21)
#define    GPIOR22         2       /*  */
#define    BM_GPIOR22	(uint8_t)(1U<<GPIOR22)
#define    GPIOR23         3       /*  */
#define    BM_GPIOR23	(uint8_t)(1U<<GPIOR23)
#define    GPIOR24         4       /*  */
#define    BM_GPIOR24	(uint8_t)(1U<<GPIOR24)
#define    GPIOR25         5       /*  */
#define    BM_GPIOR25	(uint8_t)(1U<<GPIOR25)
#define    GPIOR26         6       /*  */
#define    BM_GPIOR26	(uint8_t)(1U<<GPIOR26)
#define    GPIOR27         7       /*  */
#define    BM_GPIOR27	(uint8_t)(1U<<GPIOR27)

/* GPIOR3 - General Purpose I/O Register 3 */
#define    GPIOR30         0       /*  */
#define    BM_GPIOR30	(uint8_t)(1U<<GPIOR30)
#define    GPIOR31         1       /*  */
#define    BM_GPIOR31	(uint8_t)(1U<<GPIOR31)
#define    GPIOR32         2       /*  */
#define    BM_GPIOR32	(uint8_t)(1U<<GPIOR32)
#define    GPIOR33         3       /*  */
#define    BM_GPIOR33	(uint8_t)(1U<<GPIOR33)
#define    GPIOR34         4       /*  */
#define    BM_GPIOR34	(uint8_t)(1U<<GPIOR34)
#define    GPIOR35         5       /*  */
#define    BM_GPIOR35	(uint8_t)(1U<<GPIOR35)
#define    GPIOR36         6       /*  */
#define    BM_GPIOR36	(uint8_t)(1U<<GPIOR36)
#define    GPIOR37         7       /*  */
#define    BM_GPIOR37	(uint8_t)(1U<<GPIOR37)

/* GPIOR4 - General Purpose I/O Register 4 */
#define    GPIOR40         0       /*  */
#define    BM_GPIOR40	(uint8_t)(1U<<GPIOR40)
#define    GPIOR41         1       /*  */
#define    BM_GPIOR41	(uint8_t)(1U<<GPIOR41)
#define    GPIOR42         2       /*  */
#define    BM_GPIOR42	(uint8_t)(1U<<GPIOR42)
#define    GPIOR43         3       /*  */
#define    BM_GPIOR43	(uint8_t)(1U<<GPIOR43)
#define    GPIOR44         4       /*  */
#define    BM_GPIOR44	(uint8_t)(1U<<GPIOR44)
#define    GPIOR45         5       /*  */
#define    BM_GPIOR45	(uint8_t)(1U<<GPIOR45)
#define    GPIOR46         6       /*  */
#define    BM_GPIOR46	(uint8_t)(1U<<GPIOR46)
#define    GPIOR47         7       /*  */
#define    BM_GPIOR47	(uint8_t)(1U<<GPIOR47)

/* GPIOR5 - General Purpose I/O Register 5 */
#define    GPIOR50         0       /*  */
#define    BM_GPIOR50	(uint8_t)(1U<<GPIOR50)
#define    GPIOR51         1       /*  */
#define    BM_GPIOR51	(uint8_t)(1U<<GPIOR51)
#define    GPIOR52         2       /*  */
#define    BM_GPIOR52	(uint8_t)(1U<<GPIOR52)
#define    GPIOR53         3       /*  */
#define    BM_GPIOR53	(uint8_t)(1U<<GPIOR53)
#define    GPIOR54         4       /*  */
#define    BM_GPIOR54	(uint8_t)(1U<<GPIOR54)
#define    GPIOR55         5       /*  */
#define    BM_GPIOR55	(uint8_t)(1U<<GPIOR55)
#define    GPIOR56         6       /*  */
#define    BM_GPIOR56	(uint8_t)(1U<<GPIOR56)
#define    GPIOR57         7       /*  */
#define    BM_GPIOR57	(uint8_t)(1U<<GPIOR57)

/* GPIOR6 - General Purpose I/O Register 6 */
#define    GPIOR60         0       /*  */
#define    BM_GPIOR60	(uint8_t)(1U<<GPIOR60)
#define    GPIOR61         1       /*  */
#define    BM_GPIOR61	(uint8_t)(1U<<GPIOR61)
#define    GPIOR62         2       /*  */
#define    BM_GPIOR62	(uint8_t)(1U<<GPIOR62)
#define    GPIOR63         3       /*  */
#define    BM_GPIOR63	(uint8_t)(1U<<GPIOR63)
#define    GPIOR64         4       /*  */
#define    BM_GPIOR64	(uint8_t)(1U<<GPIOR64)
#define    GPIOR65         5       /*  */
#define    BM_GPIOR65	(uint8_t)(1U<<GPIOR65)
#define    GPIOR66         6       /*  */
#define    BM_GPIOR66	(uint8_t)(1U<<GPIOR66)
#define    GPIOR67         7       /*  */
#define    BM_GPIOR67	(uint8_t)(1U<<GPIOR67)


/* ***** INT ************************** */
/* EICRA - External Interrupt control Register */
#define    ISC00           0       /* Interrupt Sense Control 0 bit 0 */
#define    BM_ISC00	(uint8_t)(1U<<ISC00)
#define    ISC01           1       /* Interrupt Sense Control 0 bit 1 */
#define    BM_ISC01	(uint8_t)(1U<<ISC01)
#define    ISC10           2       /* Interrupt Sense Control 1 bit 0 */
#define    BM_ISC10	(uint8_t)(1U<<ISC10)
#define    ISC11           3       /* Interrupt Sense Control 1 bit 1 */
#define    BM_ISC11	(uint8_t)(1U<<ISC11)

/* EIFR - External Interrupt Flag Register */
#define    INTF0           0       /* External Interrupt Flag 0 */
#define    BM_INTF0	(uint8_t)(1U<<INTF0)
#define    INTF1           1       /* External Interrupt Flag 1 */
#define    BM_INTF1	(uint8_t)(1U<<INTF1)

/* EIMSK - External Interrupt Mask Register */
#define    INT0            0       /* External Interrupt Request 0 Enable */
#define    BM_INT0	(uint8_t)(1U<<INT0)
#define    INT1            1       /* External Interrupt Request 1 Enable */
#define    BM_INT1	(uint8_t)(1U<<INT1)

/* PCICR - Pin change Interrupt control Register */
#define    PCIE0           0       /* Pin Change Interrupt Enable 0 */
#define    BM_PCIE0	(uint8_t)(1U<<PCIE0)
#define    PCIE1           1       /* Pin Change Interrupt Enable 1 */
#define    BM_PCIE1	(uint8_t)(1U<<PCIE1)

/* PCIFR - Pin change Interrupt flag Register */
#define    PCIF0           0       /* Pin Change Interrupt Flag 0 */
#define    BM_PCIF0	(uint8_t)(1U<<PCIF0)
#define    PCIF1           1       /* Pin Change Interrupt Flag 1 */
#define    BM_PCIF1	(uint8_t)(1U<<PCIF1)

/* PCMSK0 - Pin change Mask Register 0 */
#define    PCINT0          0       /* Pin Change Enable Mask bit 0 */
#define    BM_PCINT0	(uint8_t)(1U<<PCINT0)
#define    PCINT1          1       /* Pin Change Enable Mask bit 1 */
#define    BM_PCINT1	(uint8_t)(1U<<PCINT1)
#define    PCINT2          2       /* Pin Change Enable Mask bit 2 */
#define    BM_PCINT2	(uint8_t)(1U<<PCINT2)
#define    PCINT3          3       /* Pin Change Enable Mask bit 3 */
#define    BM_PCINT3	(uint8_t)(1U<<PCINT3)
#define    PCINT4          4       /* Pin Change Enable Mask bit 4 */
#define    BM_PCINT4	(uint8_t)(1U<<PCINT4)
#define    PCINT5          5       /* Pin Change Enable Mask bit 5 */
#define    BM_PCINT5	(uint8_t)(1U<<PCINT5)
#define    PCINT6          6       /* Pin Change Enable Mask bit 6 */
#define    BM_PCINT6	(uint8_t)(1U<<PCINT6)
#define    PCINT7          7       /* Pin Change Enable Mask bit 7 */
#define    BM_PCINT7	(uint8_t)(1U<<PCINT7)

/* PCMSK1 - Pin change Mask Register 1 */
#define    PCINT8          0       /* Pin Change Enable Mask bit 8 */
#define    BM_PCINT8	(uint8_t)(1U<<PCINT8)
#define    PCINT9          1       /* Pin Change Enable Mask bit 9 */
#define    BM_PCINT9	(uint8_t)(1U<<PCINT9)
#define    PCINT10         2       /* Pin Change Enable Mask bit 10 */
#define    BM_PCINT10	(uint8_t)(1U<<PCINT10)
#define    PCINT11         3       /* Pin Change Enable Mask bit 11 */
#define    BM_PCINT11	(uint8_t)(1U<<PCINT11)
#define    PCINT12         4       /* Pin Change Enable Mask bit 12 */
#define    BM_PCINT12	(uint8_t)(1U<<PCINT12)
#define    PCINT13         5       /* Pin Change Enable Mask bit 13 */
#define    BM_PCINT13	(uint8_t)(1U<<PCINT13)
#define    PCINT14         6       /* Pin Change Enable Mask bit 14 */
#define    BM_PCINT14	(uint8_t)(1U<<PCINT14)
#define    PCINT15         7       /* Pin Change Enable Mask bit 15 */
#define    BM_PCINT15	(uint8_t)(1U<<PCINT15)


/* ***** LED ************************** */
/* PDSCR - Pad Driver Strength Control Register */
#define    PDSC0           0       /* Pad Driver Strength Control bit 0 */
#define    BM_PDSC0	(uint8_t)(1U<<PDSC0)
#define    PDSC1           1       /* Pad Driver Strength Control bit 1 */
#define    BM_PDSC1	(uint8_t)(1U<<PDSC1)
#define    PDSC2           2       /* Pad Driver Strength Control bit 2 */
#define    BM_PDSC2	(uint8_t)(1U<<PDSC2)
#define    PDSC3           3       /* Pad Driver Strength Control bit 3 */
#define    BM_PDSC3	(uint8_t)(1U<<PDSC3)
#define    PDSC4           4       /* Pad Driver Strength Control bit 4 */
#define    BM_PDSC4	(uint8_t)(1U<<PDSC4)
#define    RSSISEL         6       /* Testmode Enable for RSSI Measurement */
#define    BM_RSSISEL	(uint8_t)(1U<<RSSISEL)
#define    ATBSEL          7       /* Analog Testbus Select */
#define    BM_ATBSEL	(uint8_t)(1U<<ATBSEL)


/* ***** LF3D ************************* */
/* LFCPR - LF Receiver Calibration Protect Register */
#define    LFCALP          0       /* LF CaLibration Protection */
#define    BM_LFCALP	(uint8_t)(1U<<LFCALP)
#define    LFCALRY         1       /* LF Calibration Ready */
#define    BM_LFCALRY	(uint8_t)(1U<<LFCALRY)
#define    LFCPCE          7       /* LF CaLibration Protect Change Enable */
#define    BM_LFCPCE	(uint8_t)(1U<<LFCPCE)

/* LFCR0 - LF Receiver Decode Unit Control Register 0 */
#define    LFCE1           0       /* LF Receiver Channel 1 Enable */
#define    BM_LFCE1	(uint8_t)(1U<<LFCE1)
#define    LFCE2           1       /* LF Receiver Channel 2 Enable */
#define    BM_LFCE2	(uint8_t)(1U<<LFCE2)
#define    LFCE3           2       /* LF Receiver Channel 3 Enable */
#define    BM_LFCE3	(uint8_t)(1U<<LFCE3)
#define    LFBR0           3       /* LF Receiver Bit Rate bit 0 */
#define    BM_LFBR0	(uint8_t)(1U<<LFBR0)
#define    LFBR1           4       /* LF Receiver Bit Rate bit 1 */
#define    BM_LFBR1	(uint8_t)(1U<<LFBR1)
#define    LFMG            5       /* LF Receiver Maximum Gain Select */
#define    BM_LFMG	(uint8_t)(1U<<LFMG)
#define    LFRRT0          6       /* LF Receiver Reset Time select bit 0 */
#define    BM_LFRRT0	(uint8_t)(1U<<LFRRT0)
#define    LFRRT1          7       /* LF Receiver Reset Time select bit 1 */
#define    BM_LFRRT1	(uint8_t)(1U<<LFRRT1)

/* LFCR1 - LF Receiver Decode Unit Control Register 1 */
#define    LFFM0           0       /* LF Function Mode bit 0 */
#define    BM_LFFM0	(uint8_t)(1U<<LFFM0)
#define    LFFM1           1       /* LF Function Mode bit 1 */
#define    BM_LFFM1	(uint8_t)(1U<<LFFM1)
#define    ARMDE           2       /* Analog Reset Manchester Detector Enable */
#define    BM_ARMDE	(uint8_t)(1U<<ARMDE)
#define    LFCODE          3       /* Reserved for future use */
#define    BM_LFCODE	(uint8_t)(1U<<LFCODE)
#define    FLLEN           4       /* FLL Enable */
#define    BM_FLLEN	(uint8_t)(1U<<FLLEN)
#define    ADTHEN          5       /* Adapt Threshold Enable */
#define    BM_ADTHEN	(uint8_t)(1U<<ADTHEN)
#define    LFPEEN          6       /* LF Port Event Enable */
#define    BM_LFPEEN	(uint8_t)(1U<<LFPEEN)
#define    LFRE            7       /* LF Receiver Enable */
#define    BM_LFRE	(uint8_t)(1U<<LFRE)

/* LFCR2 - LF Receiver Decode Unit Control Register 2 */
#define    LFSEN0          0       /* LF Sensitivity Mode bit 0 */
#define    BM_LFSEN0	(uint8_t)(1U<<LFSEN0)
#define    LFSEN1          1       /* LF Sensitivity Mode bit 1 */
#define    BM_LFSEN1	(uint8_t)(1U<<LFSEN1)
#define    LFDAMP          2       /* LF coil Damping level select */
#define    BM_LFDAMP	(uint8_t)(1U<<LFDAMP)
#define    LFSBEN          4       /* LF Standby mode Enable */
#define    BM_LFSBEN	(uint8_t)(1U<<LFSBEN)
#define    LFVC0           5       /* LF Velocity Control bit 0 */
#define    BM_LFVC0	(uint8_t)(1U<<LFVC0)
#define    LFVC1           6       /* LF Velocity Control bit 1 */
#define    BM_LFVC1	(uint8_t)(1U<<LFVC1)
#define    LFVC2           7       /* LF Velocity Control bit 2 */
#define    BM_LFVC2	(uint8_t)(1U<<LFVC2)

/* LFCR3 - LF Receiver Decode Unit Control Register 3 */
#define    LFRCTEN         0       /* LF RC Trim Enable */
#define    BM_LFRCTEN	(uint8_t)(1U<<LFRCTEN)
#define    LFRCPCEN        1       /* LF RC Pump Continous Enable */
#define    BM_LFRCPCEN	(uint8_t)(1U<<LFRCPCEN)
#define    LFRCPM          2       /* LF RC Pump mode Enable */
#define    BM_LFRCPM	(uint8_t)(1U<<LFRCPM)

/* LFDSR1 - LF Receiver Decoder Setting Register 1 */
#define    LOTHA0          0       /* Low Threshold A bit 0 */
#define    BM_LOTHA0	(uint8_t)(1U<<LOTHA0)
#define    LOTHA1          1       /* Low Threshold A bit 1 */
#define    BM_LOTHA1	(uint8_t)(1U<<LOTHA1)
#define    HITHA0          2       /* High Threshold A bit 0 */
#define    BM_HITHA0	(uint8_t)(1U<<HITHA0)
#define    HITHA1          3       /* High Threshold A bit 1 */
#define    BM_HITHA1	(uint8_t)(1U<<HITHA1)
#define    CTTHA0          4       /* Count Threshold A bit 0 */
#define    BM_CTTHA0	(uint8_t)(1U<<CTTHA0)
#define    CTTHA1          5       /* Count Threshold A bit 1 */
#define    BM_CTTHA1	(uint8_t)(1U<<CTTHA1)

/* LFDSR11 - Low Frequency Decoder Setting Register 11 */
#define    TINITA0         0       /* Time Init A bit 0 */
#define    BM_TINITA0	(uint8_t)(1U<<TINITA0)
#define    TINITA1         1       /* Time Init A bit 1 */
#define    BM_TINITA1	(uint8_t)(1U<<TINITA1)
#define    TINITA2         2       /* Time Init A bit 2 */
#define    BM_TINITA2	(uint8_t)(1U<<TINITA2)
#define    TINITA3         3       /* Time Init A bit 3 */
#define    BM_TINITA3	(uint8_t)(1U<<TINITA3)
#define    TINITB0         4       /* Time Init B bit 0 */
#define    BM_TINITB0	(uint8_t)(1U<<TINITB0)
#define    TINITB1         5       /* Time Init B bit 1 */
#define    BM_TINITB1	(uint8_t)(1U<<TINITB1)
#define    TINITB2         6       /* Time Init B bit 2 */
#define    BM_TINITB2	(uint8_t)(1U<<TINITB2)
#define    TINITB3         7       /* Time Init B bit 3 */
#define    BM_TINITB3	(uint8_t)(1U<<TINITB3)

/* LFDSR2 - LF Receiver Decoder Setting Register 2 */
#define    LOTHB0          0       /* Low Threshold B bit 0 */
#define    BM_LOTHB0	(uint8_t)(1U<<LOTHB0)
#define    LOTHB1          1       /* Low Threshold B bit 1 */
#define    BM_LOTHB1	(uint8_t)(1U<<LOTHB1)
#define    HITHB0          2       /* High Threshold B bit 0 */
#define    BM_HITHB0	(uint8_t)(1U<<HITHB0)
#define    HITHB1          3       /* High Threshold B bit 1 */
#define    BM_HITHB1	(uint8_t)(1U<<HITHB1)
#define    CTTHB0          4       /* Count Threshold B bit 0 */
#define    BM_CTTHB0	(uint8_t)(1U<<CTTHB0)
#define    CTTHB1          5       /* Count Threshold B bit 1 */
#define    BM_CTTHB1	(uint8_t)(1U<<CTTHB1)

/* LFDSR3 - LF Receiver Decoder Setting Register 3 */
#define    PBDTH0          0       /* Preburst Detection Threshold bit 0 */
#define    BM_PBDTH0	(uint8_t)(1U<<PBDTH0)
#define    PBDTH1          1       /* Preburst Detection Threshold bit 1 */
#define    BM_PBDTH1	(uint8_t)(1U<<PBDTH1)
#define    QCTH0           3       /* Quality Check Threshold bit 0 */
#define    BM_QCTH0	(uint8_t)(1U<<QCTH0)
#define    QCTH1           4       /* Quality Check Threshold bit 1 */
#define    BM_QCTH1	(uint8_t)(1U<<QCTH1)
#define    QCTH2           5       /* Quality Check Threshold bit 2 */
#define    BM_QCTH2	(uint8_t)(1U<<QCTH2)

/* LFDSR4 - LF Receiver Decoder Setting Register 4 */
#define    SDTHA0          0       /* Sync Detection Threshold A bit 0 */
#define    BM_SDTHA0	(uint8_t)(1U<<SDTHA0)
#define    SDTHA1          1       /* Sync Detection Threshold A bit 1 */
#define    BM_SDTHA1	(uint8_t)(1U<<SDTHA1)
#define    SDTHA2          2       /* Sync Detection Threshold A bit 2 */
#define    BM_SDTHA2	(uint8_t)(1U<<SDTHA2)
#define    SCTHA0          3       /* Sync Count Threshold A bit 0 */
#define    BM_SCTHA0	(uint8_t)(1U<<SCTHA0)
#define    SCTHA1          4       /* Sync Count Threshold A bit 1 */
#define    BM_SCTHA1	(uint8_t)(1U<<SCTHA1)
#define    SCTHA2          5       /* Sync Count Threshold A bit 2 */
#define    BM_SCTHA2	(uint8_t)(1U<<SCTHA2)
#define    SRSTC0          6       /* Sync Restart Control bit 0 */
#define    BM_SRSTC0	(uint8_t)(1U<<SRSTC0)
#define    SRSTC1          7       /* Sync Restart Control bit 1 */
#define    BM_SRSTC1	(uint8_t)(1U<<SRSTC1)

/* LFFR - LF Receiver Decode Unit Flag Register */
#define    LFSYDF          0       /* LF Synchronization Detect Flag */
#define    BM_LFSYDF	(uint8_t)(1U<<LFSYDF)
#define    LFDEF           1       /* LF Decoder Error Flag */
#define    BM_LFDEF	(uint8_t)(1U<<LFDEF)
#define    LFEOF           2       /* LF End of Telegram Flag */
#define    BM_LFEOF	(uint8_t)(1U<<LFEOF)
#define    LFTOF           3       /* LF telegram Time Out Flag */
#define    BM_LFTOF	(uint8_t)(1U<<LFTOF)
#define    LFSD            6       /* LF Signal Detect */
#define    BM_LFSD	(uint8_t)(1U<<LFSD)
#define    LFES            7       /* LF Envelope Signal */
#define    BM_LFES	(uint8_t)(1U<<LFES)

/* LFIMR - LF Receiver Decode Unit Interrupt Mask Register */
#define    LFSYDIM         0       /* LF Synchronization Detected Interrupt Mask */
#define    BM_LFSYDIM	(uint8_t)(1U<<LFSYDIM)
#define    LFDEIM          1       /* LF Decoder Error Interrupt Mask */
#define    BM_LFDEIM	(uint8_t)(1U<<LFDEIM)
#define    LFEOIM          2       /* LF End Of telegram Interrupt Mask */
#define    BM_LFEOIM	(uint8_t)(1U<<LFEOIM)

/* LFQC1 - LF Receiver Decode Unit Channel 1 Quality Faktor Register */
#define    LFQS10          0       /* LF resistor select channel 1 bit 0 */
#define    BM_LFQS10	(uint8_t)(1U<<LFQS10)
#define    LFQS11          1       /* LF resistor select channel 1 bit 1 */
#define    BM_LFQS11	(uint8_t)(1U<<LFQS11)
#define    LFQS12          2       /* LF resistor select channel 1 bit 2 */
#define    BM_LFQS12	(uint8_t)(1U<<LFQS12)
#define    LFQS13          3       /* LF resistor select channel 1 bit 3 */
#define    BM_LFQS13	(uint8_t)(1U<<LFQS13)
#define    LFCS10          4       /* LF capacitor select channel 1 bit 0 */
#define    BM_LFCS10	(uint8_t)(1U<<LFCS10)
#define    LFCS11          5       /* LF capacitor select channel 1 bit 1 */
#define    BM_LFCS11	(uint8_t)(1U<<LFCS11)
#define    LFCS12          6       /* LF capacitor select channel 1 bit 2 */
#define    BM_LFCS12	(uint8_t)(1U<<LFCS12)
#define    LFCS13          7       /* LF capacitor select channel 1 bit 3 */
#define    BM_LFCS13	(uint8_t)(1U<<LFCS13)

/* LFQC2 - LF Receiver Decode Unit Channel 2 Quality Faktor Register */
#define    LFQS20          0       /* LF resistor select channel 2 bit 0 */
#define    BM_LFQS20	(uint8_t)(1U<<LFQS20)
#define    LFQS21          1       /* LF resistor select channel 2 bit 1 */
#define    BM_LFQS21	(uint8_t)(1U<<LFQS21)
#define    LFQS22          2       /* LF resistor select channel 2 bit 2 */
#define    BM_LFQS22	(uint8_t)(1U<<LFQS22)
#define    LFQS23          3       /* LF resistor select channel 2 bit 3 */
#define    BM_LFQS23	(uint8_t)(1U<<LFQS23)
#define    LFCS20          4       /* LF capacitor select channel 2 bit 0 */
#define    BM_LFCS20	(uint8_t)(1U<<LFCS20)
#define    LFCS21          5       /* LF capacitor select channel 2 bit 1 */
#define    BM_LFCS21	(uint8_t)(1U<<LFCS21)
#define    LFCS22          6       /* LF capacitor select channel 2 bit 2 */
#define    BM_LFCS22	(uint8_t)(1U<<LFCS22)
#define    LFCS23          7       /* LF capacitor select channel 2 bit 3 */
#define    BM_LFCS23	(uint8_t)(1U<<LFCS23)

/* LFQC3 - LF Receiver Decode Unit Channel 3 Quality Faktor Register */
#define    LFQS30          0       /* LF resistor select channel 3 bit 0 */
#define    BM_LFQS30	(uint8_t)(1U<<LFQS30)
#define    LFQS31          1       /* LF resistor select channel 3 bit 1 */
#define    BM_LFQS31	(uint8_t)(1U<<LFQS31)
#define    LFQS32          2       /* LF resistor select channel 3 bit 2 */
#define    BM_LFQS32	(uint8_t)(1U<<LFQS32)
#define    LFQS33          3       /* LF resistor select channel 3 bit 3 */
#define    BM_LFQS33	(uint8_t)(1U<<LFQS33)
#define    LFCS30          4       /* LF capacitor select channel 3 bit 0 */
#define    BM_LFCS30	(uint8_t)(1U<<LFCS30)
#define    LFCS31          5       /* LF capacitor select channel 3 bit 1 */
#define    BM_LFCS31	(uint8_t)(1U<<LFCS31)
#define    LFCS32          6       /* LF capacitor select channel 3 bit 2 */
#define    BM_LFCS32	(uint8_t)(1U<<LFCS32)
#define    LFCS33          7       /* LF capacitor select channel 3 bit 3 */
#define    BM_LFCS33	(uint8_t)(1U<<LFCS33)

/* LFSTOP - LF Receiver Decode Unit Stop Bit Register */
#define    LFSTSY0         0       /* LF Stop Bits Symbols bit 0 */
#define    BM_LFSTSY0	(uint8_t)(1U<<LFSTSY0)
#define    LFSTSY1         1       /* LF Stop Bits Symbols bit 1 */
#define    BM_LFSTSY1	(uint8_t)(1U<<LFSTSY1)
#define    LFSTSY2         2       /* LF Stop Bits Symbols bit 2 */
#define    BM_LFSTSY2	(uint8_t)(1U<<LFSTSY2)
#define    LFSTSY3         3       /* LF Stop Bits Symbols bit 3 */
#define    BM_LFSTSY3	(uint8_t)(1U<<LFSTSY3)
#define    LFSTL0          4       /* LF Stop Bits Length bit 0 */
#define    BM_LFSTL0	(uint8_t)(1U<<LFSTL0)
#define    LFSTL1          5       /* LF Stop Bits Length bit 1 */
#define    BM_LFSTL1	(uint8_t)(1U<<LFSTL1)
#define    LFSTL2          6       /* LF Stop Bits Length bit 2 */
#define    BM_LFSTL2	(uint8_t)(1U<<LFSTL2)

/* LFSYLE - LF Receiver Decode Unit Synchronization Length Register */
#define    LFSYLE0         0       /*  */
#define    BM_LFSYLE0	(uint8_t)(1U<<LFSYLE0)
#define    LFSYLE1         1       /*  */
#define    BM_LFSYLE1	(uint8_t)(1U<<LFSYLE1)
#define    LFSYLE2         2       /*  */
#define    BM_LFSYLE2	(uint8_t)(1U<<LFSYLE2)
#define    LFSYLE3         3       /*  */
#define    BM_LFSYLE3	(uint8_t)(1U<<LFSYLE3)
#define    LFSYLE4         4       /*  */
#define    BM_LFSYLE4	(uint8_t)(1U<<LFSYLE4)
#define    LFSYLE5         5       /*  */
#define    BM_LFSYLE5	(uint8_t)(1U<<LFSYLE5)


/* ***** LF_PROTOCOL_HANDLER ********** */
/* PHBCRR - Protocol Handler Bit Counter Read Register */
#define    PHBCR0          0       /* Protocol Handler Bit Counter Read bit 0 */
#define    BM_PHBCR0	(uint8_t)(1U<<PHBCR0)
#define    PHBCR1          1       /* Protocol Handler Bit Counter Read bit 1 */
#define    BM_PHBCR1	(uint8_t)(1U<<PHBCR1)
#define    PHBCR2          2       /* Protocol Handler Bit Counter Read bit 2 */
#define    BM_PHBCR2	(uint8_t)(1U<<PHBCR2)
#define    PHBCR3          3       /* Protocol Handler Bit Counter Read bit 3 */
#define    BM_PHBCR3	(uint8_t)(1U<<PHBCR3)
#define    PHBCR4          4       /* Protocol Handler Bit Counter Read bit 4 */
#define    BM_PHBCR4	(uint8_t)(1U<<PHBCR4)
#define    PHBCR5          5       /* Protocol Handler Bit Counter Read bit 5 */
#define    BM_PHBCR5	(uint8_t)(1U<<PHBCR5)
#define    PHBCR6          6       /* Protocol Handler Bit Counter Read bit 6 */
#define    BM_PHBCR6	(uint8_t)(1U<<PHBCR6)
#define    PHBCR7          7       /* Protocol Handler Bit Counter Read bit 7 */
#define    BM_PHBCR7	(uint8_t)(1U<<PHBCR7)

/* PHCKCR - PH FIFO Clock Gen. Control Register */
#define    FIFSCSW         0       /* FIFO Synchronous Clock Switch */
#define    BM_FIFSCSW	(uint8_t)(1U<<FIFSCSW)
#define    FRFIFO          5       /* Frame stored in FIFO */
#define    BM_FRFIFO	(uint8_t)(1U<<FRFIFO)
#define    CPM             6       /* Continuous Pattern Mode */
#define    BM_CPM	(uint8_t)(1U<<CPM)
#define    CSM             7       /* Continue or Single receive Mode */
#define    BM_CSM	(uint8_t)(1U<<CSM)

/* PHCKSR - PH FIFO Clock Gen. Status Register */
#define    FIFO_SW         0       /* FIFO Clock Switch */
#define    BM_FIFO_SW	(uint8_t)(1U<<FIFO_SW)

/* PHCRCR - Protocol Handler CRC Control Register */
#define    CRCFR           2       /* CRC ID Frame */
#define    BM_CRCFR	(uint8_t)(1U<<CRCFR)
#define    CRCMSB          3       /* CRC MSB */
#define    BM_CRCMSB	(uint8_t)(1U<<CRCMSB)
#define    CRCSE0          4       /* CRC Select bit 0 */
#define    BM_CRCSE0	(uint8_t)(1U<<CRCSE0)
#define    CRCSE1          5       /* CRC Select bit 1 */
#define    BM_CRCSE1	(uint8_t)(1U<<CRCSE1)
#define    STREN           6       /* Strobe Enable */
#define    BM_STREN	(uint8_t)(1U<<STREN)
#define    CRCEN           7       /* CRC Enable */
#define    BM_CRCEN	(uint8_t)(1U<<CRCEN)

/* PHCRPH - PH CRC Polynomial High Byte Register */
#define    PHCRP8          0       /*  */
#define    BM_PHCRP8	(uint8_t)(1U<<PHCRP8)
#define    PHCRP9          1       /*  */
#define    BM_PHCRP9	(uint8_t)(1U<<PHCRP9)
#define    PHCRP10         2       /*  */
#define    BM_PHCRP10	(uint8_t)(1U<<PHCRP10)
#define    PHCRP11         3       /*  */
#define    BM_PHCRP11	(uint8_t)(1U<<PHCRP11)
#define    PHCRP12         4       /*  */
#define    BM_PHCRP12	(uint8_t)(1U<<PHCRP12)
#define    PHCRP13         5       /*  */
#define    BM_PHCRP13	(uint8_t)(1U<<PHCRP13)
#define    PHCRP14         6       /*  */
#define    BM_PHCRP14	(uint8_t)(1U<<PHCRP14)
#define    PHCRP15         7       /*  */
#define    BM_PHCRP15	(uint8_t)(1U<<PHCRP15)

/* PHCRPL - PH CRC Polynomial Low Byte Register */
#define    PHCRP0          0       /*  */
#define    BM_PHCRP0	(uint8_t)(1U<<PHCRP0)
#define    PHCRP1          1       /*  */
#define    BM_PHCRP1	(uint8_t)(1U<<PHCRP1)
#define    PHCRP2          2       /*  */
#define    BM_PHCRP2	(uint8_t)(1U<<PHCRP2)
#define    PHCRP3          3       /*  */
#define    BM_PHCRP3	(uint8_t)(1U<<PHCRP3)
#define    PHCRP4          4       /*  */
#define    BM_PHCRP4	(uint8_t)(1U<<PHCRP4)
#define    PHCRP5          5       /*  */
#define    BM_PHCRP5	(uint8_t)(1U<<PHCRP5)
#define    PHCRP6          6       /*  */
#define    BM_PHCRP6	(uint8_t)(1U<<PHCRP6)
#define    PHCRP7          7       /*  */
#define    BM_PHCRP7	(uint8_t)(1U<<PHCRP7)

/* PHCSRH - PH CRC Checksum High Byte Register */
#define    PHCSR8          0       /*  */
#define    BM_PHCSR8	(uint8_t)(1U<<PHCSR8)
#define    PHCSR9          1       /*  */
#define    BM_PHCSR9	(uint8_t)(1U<<PHCSR9)
#define    PHCSR10         2       /*  */
#define    BM_PHCSR10	(uint8_t)(1U<<PHCSR10)
#define    PHCSR11         3       /*  */
#define    BM_PHCSR11	(uint8_t)(1U<<PHCSR11)
#define    PHCSR12         4       /*  */
#define    BM_PHCSR12	(uint8_t)(1U<<PHCSR12)
#define    PHCSR13         5       /*  */
#define    BM_PHCSR13	(uint8_t)(1U<<PHCSR13)
#define    PHCSR14         6       /*  */
#define    BM_PHCSR14	(uint8_t)(1U<<PHCSR14)
#define    PHCSR15         7       /*  */
#define    BM_PHCSR15	(uint8_t)(1U<<PHCSR15)

/* PHCSRL - PH CRC Checksum Low Byte Register */
#define    PHCSR0          0       /*  */
#define    BM_PHCSR0	(uint8_t)(1U<<PHCSR0)
#define    PHCSR1          1       /*  */
#define    BM_PHCSR1	(uint8_t)(1U<<PHCSR1)
#define    PHCSR2          2       /*  */
#define    BM_PHCSR2	(uint8_t)(1U<<PHCSR2)
#define    PHCSR3          3       /*  */
#define    BM_PHCSR3	(uint8_t)(1U<<PHCSR3)
#define    PHCSR4          4       /*  */
#define    BM_PHCSR4	(uint8_t)(1U<<PHCSR4)
#define    PHCSR5          5       /*  */
#define    BM_PHCSR5	(uint8_t)(1U<<PHCSR5)
#define    PHCSR6          6       /*  */
#define    BM_PHCSR6	(uint8_t)(1U<<PHCSR6)
#define    PHCSR7          7       /*  */
#define    BM_PHCSR7	(uint8_t)(1U<<PHCSR7)

/* PHCSTH - PH CRC Start Value High Byte Register */
#define    PHCST8          0       /*  */
#define    BM_PHCST8	(uint8_t)(1U<<PHCST8)
#define    PHCST9          1       /*  */
#define    BM_PHCST9	(uint8_t)(1U<<PHCST9)
#define    PHCST10         2       /*  */
#define    BM_PHCST10	(uint8_t)(1U<<PHCST10)
#define    PHCST11         3       /*  */
#define    BM_PHCST11	(uint8_t)(1U<<PHCST11)
#define    PHCST12         4       /*  */
#define    BM_PHCST12	(uint8_t)(1U<<PHCST12)
#define    PHCST13         5       /*  */
#define    BM_PHCST13	(uint8_t)(1U<<PHCST13)
#define    PHCST14         6       /*  */
#define    BM_PHCST14	(uint8_t)(1U<<PHCST14)
#define    PHCST15         7       /*  */
#define    BM_PHCST15	(uint8_t)(1U<<PHCST15)

/* PHCSTL - PH CRC Start Value Low Byte Register */
#define    PHCST0          0       /*  */
#define    BM_PHCST0	(uint8_t)(1U<<PHCST0)
#define    PHCST1          1       /*  */
#define    BM_PHCST1	(uint8_t)(1U<<PHCST1)
#define    PHCST2          2       /*  */
#define    BM_PHCST2	(uint8_t)(1U<<PHCST2)
#define    PHCST3          3       /*  */
#define    BM_PHCST3	(uint8_t)(1U<<PHCST3)
#define    PHCST4          4       /*  */
#define    BM_PHCST4	(uint8_t)(1U<<PHCST4)
#define    PHCST5          5       /*  */
#define    BM_PHCST5	(uint8_t)(1U<<PHCST5)
#define    PHCST6          6       /*  */
#define    BM_PHCST6	(uint8_t)(1U<<PHCST6)
#define    PHCST7          7       /*  */
#define    BM_PHCST7	(uint8_t)(1U<<PHCST7)

/* PHDFR - Protocol Handler Data Frame end Register */
#define    PHDF0           0       /* Protocol Handler Data Frame end bit 0 */
#define    BM_PHDF0	(uint8_t)(1U<<PHDF0)
#define    PHDF1           1       /* Protocol Handler Data Frame end bit 1 */
#define    BM_PHDF1	(uint8_t)(1U<<PHDF1)
#define    PHDF2           2       /* Protocol Handler Data Frame end bit 2 */
#define    BM_PHDF2	(uint8_t)(1U<<PHDF2)
#define    PHDF3           3       /* Protocol Handler Data Frame end bit 3 */
#define    BM_PHDF3	(uint8_t)(1U<<PHDF3)
#define    PHDF4           4       /* Protocol Handler Data Frame end bit 4 */
#define    BM_PHDF4	(uint8_t)(1U<<PHDF4)
#define    PHDF5           5       /* Protocol Handler Data Frame end bit 5 */
#define    BM_PHDF5	(uint8_t)(1U<<PHDF5)
#define    PHDF6           6       /* Protocol Handler Data Frame end bit 6 */
#define    BM_PHDF6	(uint8_t)(1U<<PHDF6)
#define    PHDF7           7       /* Protocol Handler Data Frame end bit 7 */
#define    BM_PHDF7	(uint8_t)(1U<<PHDF7)

/* PHFC - PH FIFO Configuration Register */
#define    FLC0            0       /* PH FIFO Fill-Level Configuration bit 0 */
#define    BM_FLC0	(uint8_t)(1U<<FLC0)
#define    FLC1            1       /* PH FIFO Fill-Level Configuration bit 1 */
#define    BM_FLC1	(uint8_t)(1U<<FLC1)
#define    FLC2            2       /* PH FIFO Fill-Level Configuration bit 2 */
#define    BM_FLC2	(uint8_t)(1U<<FLC2)
#define    FLC3            3       /* PH FIFO Fill-Level Configuration bit 3 */
#define    BM_FLC3	(uint8_t)(1U<<FLC3)
#define    FLC4            4       /* PH FIFO Fill-Level Configuration bit 4 */
#define    BM_FLC4	(uint8_t)(1U<<FLC4)
#define    FLC5            5       /* PH FIFO Fill-Level Configuration bit 5 */
#define    BM_FLC5	(uint8_t)(1U<<FLC5)
#define    FFMSB           6       /* PH FIFO MSB alignment bit */
#define    BM_FFMSB	(uint8_t)(1U<<FFMSB)
#define    DRA             7       /* PH FIFO Direct Read Access Operational Mode */
#define    BM_DRA	(uint8_t)(1U<<DRA)

/* PHFI - PH FIFO Interrupt Mask Register */
#define    FLIM            0       /* PH FIFO Fill-level Interrupt Mask */
#define    BM_FLIM	(uint8_t)(1U<<FLIM)
#define    ERIM            1       /* PH FIFO Error Interrupt Mask */
#define    BM_ERIM	(uint8_t)(1U<<ERIM)

/* PHFL - PH FIFO Fill Level Register */
#define    FLS0            0       /* PH FIFO Fill Level Status bit 0 */
#define    BM_FLS0	(uint8_t)(1U<<FLS0)
#define    FLS1            1       /* PH FIFO Fill Level Status bit 1 */
#define    BM_FLS1	(uint8_t)(1U<<FLS1)
#define    FLS2            2       /* PH FIFO Fill Level Status bit 2 */
#define    BM_FLS2	(uint8_t)(1U<<FLS2)
#define    FLS3            3       /* PH FIFO Fill Level Status bit 3 */
#define    BM_FLS3	(uint8_t)(1U<<FLS3)
#define    FLS4            4       /* PH FIFO Fill Level Status bit 4 */
#define    BM_FLS4	(uint8_t)(1U<<FLS4)
#define    FLS5            5       /* PH FIFO Fill Level Status bit 5 */
#define    BM_FLS5	(uint8_t)(1U<<FLS5)
#define    PHCLR           7       /* PH FIFO Clear */
#define    BM_PHCLR	(uint8_t)(1U<<PHCLR)

/* PHFR - Protocol Handler Flag Register */
#define    CRCEF           0       /* PH CRC Error Status Flag */
#define    BM_CRCEF	(uint8_t)(1U<<CRCEF)
#define    PHTBLF          1       /* Protocol Handler Telegram Bit Length Flag */
#define    BM_PHTBLF	(uint8_t)(1U<<PHTBLF)
#define    PHDFF           2       /* Protocol Handler Date Frame Flag */
#define    BM_PHDFF	(uint8_t)(1U<<PHDFF)
#define    PHIDFF          3       /* Protocol Handler ID Frame Flag */
#define    BM_PHIDFF	(uint8_t)(1U<<PHIDFF)
#define    PHID0F          4       /* Protocol Handler IDentifier 0 Flag */
#define    BM_PHID0F	(uint8_t)(1U<<PHID0F)
#define    PHID1F          5       /* Protocol Handler IDentifier 1 Flag */
#define    BM_PHID1F	(uint8_t)(1U<<PHID1F)

/* PHFRP - PH FIFO Read Pointer */
#define    FRP0            0       /* PH FIFO Read Pointer bit 0 */
#define    BM_FRP0	(uint8_t)(1U<<FRP0)
#define    FRP1            1       /* PH FIFO Read Pointer bit 1 */
#define    BM_FRP1	(uint8_t)(1U<<FRP1)
#define    FRP2            2       /* PH FIFO Read Pointer bit 2 */
#define    BM_FRP2	(uint8_t)(1U<<FRP2)
#define    FRP3            3       /* PH FIFO Read Pointer bit 3 */
#define    BM_FRP3	(uint8_t)(1U<<FRP3)
#define    FRP4            4       /* PH FIFO Read Pointer bit 4 */
#define    BM_FRP4	(uint8_t)(1U<<FRP4)
#define    FRP5            5       /* PH FIFO Read Pointer bit 5 */
#define    BM_FRP5	(uint8_t)(1U<<FRP5)

/* PHFS - Protocol Handler FIFO Status Register */
#define    FLRF            0       /* PH FIFO Fill-Level Reached Status Flag */
#define    BM_FLRF	(uint8_t)(1U<<FLRF)
#define    FUFL            1       /* PH FIFO Underflow Flag */
#define    BM_FUFL	(uint8_t)(1U<<FUFL)
#define    FOFL            2       /* PH FIFO Overflow Flag */
#define    BM_FOFL	(uint8_t)(1U<<FOFL)

/* PHFWP - PH FIFO Write Pointer */
#define    FWP0            0       /* PH FIFO Write Pointer bit 0 */
#define    BM_FWP0	(uint8_t)(1U<<FWP0)
#define    FWP1            1       /* PH FIFO Write Pointer bit 1 */
#define    BM_FWP1	(uint8_t)(1U<<FWP1)
#define    FWP2            2       /* PH FIFO Write Pointer bit 2 */
#define    BM_FWP2	(uint8_t)(1U<<FWP2)
#define    FWP3            3       /* PH FIFO Write Pointer bit 3 */
#define    BM_FWP3	(uint8_t)(1U<<FWP3)
#define    FWP4            4       /* PH FIFO Write Pointer bit 4 */
#define    BM_FWP4	(uint8_t)(1U<<FWP4)
#define    FWP5            5       /* PH FIFO Write Pointer bit 5 */
#define    BM_FWP5	(uint8_t)(1U<<FWP5)

/* PHID0L - PH Identifier 0 Length Register */
#define    ID0FS0          0       /* Identifier 0 Frame Select bit 0 */
#define    BM_ID0FS0	(uint8_t)(1U<<ID0FS0)
#define    ID0FS1          1       /* Identifier 0 Frame Select bit 1 */
#define    BM_ID0FS1	(uint8_t)(1U<<ID0FS1)
#define    ID0FS2          2       /* Identifier 0 Frame Select bit 2 */
#define    BM_ID0FS2	(uint8_t)(1U<<ID0FS2)
#define    ID0FS3          3       /* Identifier 0 Frame Select bit 3 */
#define    BM_ID0FS3	(uint8_t)(1U<<ID0FS3)
#define    ID0FS4          4       /* Identifier 0 Frame Select bit 4 */
#define    BM_ID0FS4	(uint8_t)(1U<<ID0FS4)
#define    ID0FS5          5       /* Identifier 0 Frame Select bit 5 */
#define    BM_ID0FS5	(uint8_t)(1U<<ID0FS5)

/* PHID1L - PH Identifier 1 Length Register */
#define    ID1FS0          0       /* Identifier 0 Frame Select bit 0 */
#define    BM_ID1FS0	(uint8_t)(1U<<ID1FS0)
#define    ID1FS1          1       /* Identifier 0 Frame Select bit 1 */
#define    BM_ID1FS1	(uint8_t)(1U<<ID1FS1)
#define    ID1FS2          2       /* Identifier 0 Frame Select bit 2 */
#define    BM_ID1FS2	(uint8_t)(1U<<ID1FS2)
#define    ID1FS3          3       /* Identifier 0 Frame Select bit 3 */
#define    BM_ID1FS3	(uint8_t)(1U<<ID1FS3)
#define    ID1FS4          4       /* Identifier 0 Frame Select bit 4 */
#define    BM_ID1FS4	(uint8_t)(1U<<ID1FS4)
#define    ID1FS5          5       /* Identifier 0 Frame Select bit 5 */
#define    BM_ID1FS5	(uint8_t)(1U<<ID1FS5)

/* PHIDFR - Protocol Handler ID Frame Register */
#define    RDFS0           0       /*  */
#define    BM_RDFS0	(uint8_t)(1U<<RDFS0)
#define    RDFS1           1       /*  */
#define    BM_RDFS1	(uint8_t)(1U<<RDFS1)
#define    RDFS2           2       /*  */
#define    BM_RDFS2	(uint8_t)(1U<<RDFS2)
#define    RDFS3           3       /*  */
#define    BM_RDFS3	(uint8_t)(1U<<RDFS3)
#define    RDFS4           4       /*  */
#define    BM_RDFS4	(uint8_t)(1U<<RDFS4)
#define    RDFS5           5       /*  */
#define    BM_RDFS5	(uint8_t)(1U<<RDFS5)
#define    RDFS6           6       /*  */
#define    BM_RDFS6	(uint8_t)(1U<<RDFS6)
#define    RDFS7           7       /*  */
#define    BM_RDFS7	(uint8_t)(1U<<RDFS7)

/* PHIMR - Protocol Handler Interrupt Mask Register */
#define    PHTBLIM         1       /* Protocol Handler Telegram Bit Length Interrupt Mask */
#define    BM_PHTBLIM	(uint8_t)(1U<<PHTBLIM)
#define    PHDFIM          2       /* Protocol Handler Date Frame Interrupt Mask */
#define    BM_PHDFIM	(uint8_t)(1U<<PHDFIM)
#define    PHIDFIM         3       /* Protocol Handler ID Frame Interrupt Mask */
#define    BM_PHIDFIM	(uint8_t)(1U<<PHIDFIM)
#define    PHID0IM         4       /* Protocol Handler IDentifier 0 Interrupt Mask */
#define    BM_PHID0IM	(uint8_t)(1U<<PHID0IM)
#define    PHID1IM         5       /* Protocol Handler IDentifier 1 Interrupt Mask */
#define    BM_PHID1IM	(uint8_t)(1U<<PHID1IM)

/* PHTBLR - Protocol Handler Telegram Bit Length Register */
#define    PHTBL0          0       /*  */
#define    BM_PHTBL0	(uint8_t)(1U<<PHTBL0)
#define    PHTBL1          1       /*  */
#define    BM_PHTBL1	(uint8_t)(1U<<PHTBL1)
#define    PHTBL2          2       /*  */
#define    BM_PHTBL2	(uint8_t)(1U<<PHTBL2)
#define    PHTBL3          3       /*  */
#define    BM_PHTBL3	(uint8_t)(1U<<PHTBL3)
#define    PHTBL4          4       /*  */
#define    BM_PHTBL4	(uint8_t)(1U<<PHTBL4)
#define    PHTBL5          5       /*  */
#define    BM_PHTBL5	(uint8_t)(1U<<PHTBL5)
#define    PHTBL6          6       /*  */
#define    BM_PHTBL6	(uint8_t)(1U<<PHTBL6)
#define    PHTBL7          7       /*  */
#define    BM_PHTBL7	(uint8_t)(1U<<PHTBL7)

/* PHTCMR - Protocol Handler Timer Control Mode Register */
#define    PHPS0           0       /* PH timer Prescaler Select bit 0 */
#define    BM_PHPS0	(uint8_t)(1U<<PHPS0)
#define    PHPS1           1       /* PH timer Prescaler Select bit 1 */
#define    BM_PHPS1	(uint8_t)(1U<<PHPS1)
#define    PHPS2           2       /* PH timer Prescaler Select bit 2 */
#define    BM_PHPS2	(uint8_t)(1U<<PHPS2)
#define    PHCRM           3       /* PH timer Compare Reset Mask */
#define    BM_PHCRM	(uint8_t)(1U<<PHCRM)
#define    PHCIM           4       /* PH timer Compare Interrupt Mask */
#define    BM_PHCIM	(uint8_t)(1U<<PHCIM)
#define    PHRES           5       /* PH timer Reset */
#define    BM_PHRES	(uint8_t)(1U<<PHRES)
#define    PHSM            6       /* PH timer Start Mode */
#define    BM_PHSM	(uint8_t)(1U<<PHSM)
#define    PHTE            7       /* PH Timer Enable */
#define    BM_PHTE	(uint8_t)(1U<<PHTE)

/* PHTCOR - Protocol Handler Timer Compare Register */
#define    PHTCOR0         0       /*  */
#define    BM_PHTCOR0	(uint8_t)(1U<<PHTCOR0)
#define    PHTCOR1         1       /*  */
#define    BM_PHTCOR1	(uint8_t)(1U<<PHTCOR1)
#define    PHTCOR2         2       /*  */
#define    BM_PHTCOR2	(uint8_t)(1U<<PHTCOR2)
#define    PHTCOR3         3       /*  */
#define    BM_PHTCOR3	(uint8_t)(1U<<PHTCOR3)
#define    PHTCOR4         4       /*  */
#define    BM_PHTCOR4	(uint8_t)(1U<<PHTCOR4)
#define    PHTCOR5         5       /*  */
#define    BM_PHTCOR5	(uint8_t)(1U<<PHTCOR5)
#define    PHTCOR6         6       /*  */
#define    BM_PHTCOR6	(uint8_t)(1U<<PHTCOR6)
#define    PHTCOR7         7       /*  */
#define    BM_PHTCOR7	(uint8_t)(1U<<PHTCOR7)

/* PHTEMR - Protocol Handler Timer Event Mask Register */
#define    ID0EM           0       /* ID0 Event Mask */
#define    BM_ID0EM	(uint8_t)(1U<<ID0EM)
#define    ID1EM           1       /* ID1 Event Mask */
#define    BM_ID1EM	(uint8_t)(1U<<ID1EM)
#define    IDFEM           2       /* Identifier Frame Event Mask */
#define    BM_IDFEM	(uint8_t)(1U<<IDFEM)
#define    DFEM            3       /* Data Frame end reached Event Mask */
#define    BM_DFEM	(uint8_t)(1U<<DFEM)
#define    TBLEM           4       /* Telegram Bit Length reached Event Mask */
#define    BM_TBLEM	(uint8_t)(1U<<TBLEM)
#define    FLEM            5       /* Fill Level reached Event Mask */
#define    BM_FLEM	(uint8_t)(1U<<FLEM)
#define    EOFEM           6       /* End OF telegram Event Mask */
#define    BM_EOFEM	(uint8_t)(1U<<EOFEM)
#define    PHCOF           7       /* PH timer Compare Flag */
#define    BM_PHCOF	(uint8_t)(1U<<PHCOF)


/* ***** LF_RSSI ********************** */
/* LFRSC1H - LF RSSI Value Channel 1 High Byte Register */
#define    LFRSC18         0       /* LF RSSI Value Channel 1 bit 8 */
#define    BM_LFRSC18	(uint8_t)(1U<<LFRSC18)
#define    LFRSC19         1       /* LF RSSI Value Channel 1 bit 9 */
#define    BM_LFRSC19	(uint8_t)(1U<<LFRSC19)
#define    LFRSC110        2       /* LF RSSI Value Channel 1 bit 10 */
#define    BM_LFRSC110	(uint8_t)(1U<<LFRSC110)
#define    LFRSC111        3       /* LF RSSI Value Channel 1 bit 11 */
#define    BM_LFRSC111	(uint8_t)(1U<<LFRSC111)
#define    LFRSC112        4       /* LF RSSI Value Channel 1 bit 12 */
#define    BM_LFRSC112	(uint8_t)(1U<<LFRSC112)
#define    LFRSC113        5       /* LF RSSI Value Channel 1 bit 13 */
#define    BM_LFRSC113	(uint8_t)(1U<<LFRSC113)
#define    LFRSC114        6       /* LF RSSI Value Channel 1 bit 14 */
#define    BM_LFRSC114	(uint8_t)(1U<<LFRSC114)
#define    LFRSC115        7       /* LF RSSI Value Channel 1 bit 15 */
#define    BM_LFRSC115	(uint8_t)(1U<<LFRSC115)

/* LFRSC1L - LF RSSI Value Channel 1 Low Byte Register */
#define    LFRSC10         0       /* LF RSSI Value Channel 1 bit 0 */
#define    BM_LFRSC10	(uint8_t)(1U<<LFRSC10)
#define    LFRSC11         1       /* LF RSSI Value Channel 1 bit 1 */
#define    BM_LFRSC11	(uint8_t)(1U<<LFRSC11)
#define    LFRSC12         2       /* LF RSSI Value Channel 1 bit 2 */
#define    BM_LFRSC12	(uint8_t)(1U<<LFRSC12)
#define    LFRSC13         3       /* LF RSSI Value Channel 1 bit 3 */
#define    BM_LFRSC13	(uint8_t)(1U<<LFRSC13)
#define    LFRSC14         4       /* LF RSSI Value Channel 1 bit 4 */
#define    BM_LFRSC14	(uint8_t)(1U<<LFRSC14)
#define    LFRSC15         5       /* LF RSSI Value Channel 1 bit 5 */
#define    BM_LFRSC15	(uint8_t)(1U<<LFRSC15)
#define    LFRSC16         6       /* LF RSSI Value Channel 1 bit 6 */
#define    BM_LFRSC16	(uint8_t)(1U<<LFRSC16)
#define    LFRSC17         7       /* LF RSSI Value Channel 1 bit 7 */
#define    BM_LFRSC17	(uint8_t)(1U<<LFRSC17)

/* LFRSC2H - LF RSSI Value Channel 2 High Byte Register */
#define    LFRSC28         0       /* LF RSSI Value Channel 2 bit 8 */
#define    BM_LFRSC28	(uint8_t)(1U<<LFRSC28)
#define    LFRSC29         1       /* LF RSSI Value Channel 2 bit 9 */
#define    BM_LFRSC29	(uint8_t)(1U<<LFRSC29)
#define    LFRSC210        2       /* LF RSSI Value Channel 2 bit 10 */
#define    BM_LFRSC210	(uint8_t)(1U<<LFRSC210)
#define    LFRSC211        3       /* LF RSSI Value Channel 2 bit 11 */
#define    BM_LFRSC211	(uint8_t)(1U<<LFRSC211)
#define    LFRSC212        4       /* LF RSSI Value Channel 2 bit 12 */
#define    BM_LFRSC212	(uint8_t)(1U<<LFRSC212)
#define    LFRSC213        5       /* LF RSSI Value Channel 2 bit 13 */
#define    BM_LFRSC213	(uint8_t)(1U<<LFRSC213)
#define    LFRSC214        6       /* LF RSSI Value Channel 2 bit 14 */
#define    BM_LFRSC214	(uint8_t)(1U<<LFRSC214)
#define    LFRSC215        7       /* LF RSSI Value Channel 2 bit 15 */
#define    BM_LFRSC215	(uint8_t)(1U<<LFRSC215)

/* LFRSC2L - LF RSSI Value Channel 2 Low Byte Register */
#define    LFRSC20         0       /* LF RSSI Value Channel 2 bit 0 */
#define    BM_LFRSC20	(uint8_t)(1U<<LFRSC20)
#define    LFRSC21         1       /* LF RSSI Value Channel 2 bit 1 */
#define    BM_LFRSC21	(uint8_t)(1U<<LFRSC21)
#define    LFRSC22         2       /* LF RSSI Value Channel 2 bit 2 */
#define    BM_LFRSC22	(uint8_t)(1U<<LFRSC22)
#define    LFRSC23         3       /* LF RSSI Value Channel 2 bit 3 */
#define    BM_LFRSC23	(uint8_t)(1U<<LFRSC23)
#define    LFRSC24         4       /* LF RSSI Value Channel 2 bit 4 */
#define    BM_LFRSC24	(uint8_t)(1U<<LFRSC24)
#define    LFRSC25         5       /* LF RSSI Value Channel 2 bit 5 */
#define    BM_LFRSC25	(uint8_t)(1U<<LFRSC25)
#define    LFRSC26         6       /* LF RSSI Value Channel 2 bit 6 */
#define    BM_LFRSC26	(uint8_t)(1U<<LFRSC26)
#define    LFRSC27         7       /* LF RSSI Value Channel 2 bit 7 */
#define    BM_LFRSC27	(uint8_t)(1U<<LFRSC27)

/* LFRSC3H - LF RSSI Value Channel 3 High Byte Register */
#define    LFRSC38         0       /* LF RSSI Value Channel 3 bit 8 */
#define    BM_LFRSC38	(uint8_t)(1U<<LFRSC38)
#define    LFRSC39         1       /* LF RSSI Value Channel 3 bit 9 */
#define    BM_LFRSC39	(uint8_t)(1U<<LFRSC39)
#define    LFRSC310        2       /* LF RSSI Value Channel 3 bit 10 */
#define    BM_LFRSC310	(uint8_t)(1U<<LFRSC310)
#define    LFRSC311        3       /* LF RSSI Value Channel 3 bit 11 */
#define    BM_LFRSC311	(uint8_t)(1U<<LFRSC311)
#define    LFRSC312        4       /* LF RSSI Value Channel 3 bit 12 */
#define    BM_LFRSC312	(uint8_t)(1U<<LFRSC312)
#define    LFRSC313        5       /* LF RSSI Value Channel 3 bit 13 */
#define    BM_LFRSC313	(uint8_t)(1U<<LFRSC313)
#define    LFRSC314        6       /* LF RSSI Value Channel 3 bit 14 */
#define    BM_LFRSC314	(uint8_t)(1U<<LFRSC314)
#define    LFRSC315        7       /* LF RSSI Value Channel 3 bit 15 */
#define    BM_LFRSC315	(uint8_t)(1U<<LFRSC315)

/* LFRSC3L - LF RSSI Value Channel 3 Low Byte Register */
#define    LFRSC30         0       /* LF RSSI Value Channel 3 bit 0 */
#define    BM_LFRSC30	(uint8_t)(1U<<LFRSC30)
#define    LFRSC31         1       /* LF RSSI Value Channel 3 bit 1 */
#define    BM_LFRSC31	(uint8_t)(1U<<LFRSC31)
#define    LFRSC32         2       /* LF RSSI Value Channel 3 bit 2 */
#define    BM_LFRSC32	(uint8_t)(1U<<LFRSC32)
#define    LFRSC33         3       /* LF RSSI Value Channel 3 bit 3 */
#define    BM_LFRSC33	(uint8_t)(1U<<LFRSC33)
#define    LFRSC34         4       /* LF RSSI Value Channel 3 bit 4 */
#define    BM_LFRSC34	(uint8_t)(1U<<LFRSC34)
#define    LFRSC35         5       /* LF RSSI Value Channel 3 bit 5 */
#define    BM_LFRSC35	(uint8_t)(1U<<LFRSC35)
#define    LFRSC36         6       /* LF RSSI Value Channel 3 bit 6 */
#define    BM_LFRSC36	(uint8_t)(1U<<LFRSC36)
#define    LFRSC37         7       /* LF RSSI Value Channel 3 bit 7 */
#define    BM_LFRSC37	(uint8_t)(1U<<LFRSC37)

/* LFRSCR - LF RSSI Control Register */
#define    LFRSA0          0       /* LF RSSI Average Number bit 0 */
#define    BM_LFRSA0	(uint8_t)(1U<<LFRSA0)
#define    LFRSA1          1       /* LF RSSI Average Number bit 1 */
#define    BM_LFRSA1	(uint8_t)(1U<<LFRSA1)
#define    LFRSA2          2       /* LF RSSI Average Number bit 2 */
#define    BM_LFRSA2	(uint8_t)(1U<<LFRSA2)
#define    LFRSMS          3       /* LF RSSI Measurement Start */
#define    BM_LFRSMS	(uint8_t)(1U<<LFRSMS)
#define    LFRSIM          4       /* LF RSSI Interrupt Mask */
#define    BM_LFRSIM	(uint8_t)(1U<<LFRSIM)
#define    LFRSRS          5       /* LF RSSI Reset Select */
#define    BM_LFRSRS	(uint8_t)(1U<<LFRSRS)

/* LFRSFR - LF RSSI FLAG Register */
#define    LFRSMF          0       /* LF RSSI Measure Flag */
#define    BM_LFRSMF	(uint8_t)(1U<<LFRSMF)
#define    LFRSTO1         1       /* LF RSSI Time Out channel 1 */
#define    BM_LFRSTO1	(uint8_t)(1U<<LFRSTO1)
#define    LFRSTO2         2       /* LF RSSI Time Out channel 2 */
#define    BM_LFRSTO2	(uint8_t)(1U<<LFRSTO2)
#define    LFRSTO3         3       /* LF RSSI Time Out channel 3 */
#define    BM_LFRSTO3	(uint8_t)(1U<<LFRSTO3)

/* LFRSISR - LF RSSI Initial Step Register */
#define    LFRSIS0         0       /* LF RSSI initial value bit 0 */
#define    BM_LFRSIS0	(uint8_t)(1U<<LFRSIS0)
#define    LFRSIS1         1       /* LF RSSI initial value bit 1 */
#define    BM_LFRSIS1	(uint8_t)(1U<<LFRSIS1)
#define    LFRSIS2         2       /* LF RSSI initial value bit 2 */
#define    BM_LFRSIS2	(uint8_t)(1U<<LFRSIS2)
#define    LFRSIS3         3       /* LF RSSI initial value bit 3 */
#define    BM_LFRSIS3	(uint8_t)(1U<<LFRSIS3)
#define    LFRSIS4         4       /* LF RSSI initial value bit 4 */
#define    BM_LFRSIS4	(uint8_t)(1U<<LFRSIS4)
#define    LFRSIS5         5       /* LF RSSI initial value bit 5 */
#define    BM_LFRSIS5	(uint8_t)(1U<<LFRSIS5)
#define    RESIS           6       /* Reset the LFRSIS Register Address */
#define    BM_RESIS	(uint8_t)(1U<<RESIS)
#define    LFRSNIS         7       /* LF RSSI No Initial Step */
#define    BM_LFRSNIS	(uint8_t)(1U<<LFRSNIS)

/* LFRSMR - LF RSSI Mode Register */
#define    LFRSM0          0       /* LF RSSI Mode bit 0 */
#define    BM_LFRSM0	(uint8_t)(1U<<LFRSM0)
#define    LFRSM1          1       /* LF RSSI Mode bit 1 */
#define    BM_LFRSM1	(uint8_t)(1U<<LFRSM1)
#define    LFRSM2          2       /* LF RSSI Mode bit 2 */
#define    BM_LFRSM2	(uint8_t)(1U<<LFRSM2)
#define    LFRSM3          3       /* LF RSSI Mode bit 3 */
#define    BM_LFRSM3	(uint8_t)(1U<<LFRSM3)
#define    LFRSCM          4       /* LF RSSI Current Mode select */
#define    BM_LFRSCM	(uint8_t)(1U<<LFRSCM)
#define    LFRSFD          5       /* LF RSSI Full average Data select */
#define    BM_LFRSFD	(uint8_t)(1U<<LFRSFD)
#define    LFRSD0          6       /* LF RSSI T2 delay select bit 0 */
#define    BM_LFRSD0	(uint8_t)(1U<<LFRSD0)
#define    LFRSD1          7       /* LF RSSI T2 delay select bit 1 */
#define    BM_LFRSD1	(uint8_t)(1U<<LFRSD1)

/* LFRSS1R - LF RSSI Setting 1 Register */
#define    RSD_OK_TI0      0       /* RSD OK delay bit 0 */
#define    BM_RSD_OK_TI0	(uint8_t)(1U<<RSD_OK_TI0)
#define    RSD_OK_TI1      1       /* RSD OK delay bit 1 */
#define    BM_RSD_OK_TI1	(uint8_t)(1U<<RSD_OK_TI1)
#define    CK_TI0          2       /* RSSI step direction decision time bit 0 */
#define    BM_CK_TI0	(uint8_t)(1U<<CK_TI0)
#define    CK_TI1          3       /* RSSI step direction decision time bit 1 */
#define    BM_CK_TI1	(uint8_t)(1U<<CK_TI1)
#define    RS_STEP0        4       /* Initial RSSI step size bit 0 */
#define    BM_RS_STEP0	(uint8_t)(1U<<RS_STEP0)
#define    RS_STEP1        5       /* Initial RSSI step size bit 1 */
#define    BM_RS_STEP1	(uint8_t)(1U<<RS_STEP1)
#define    DEL10           6       /* Delay 1 Select bit 0 */
#define    BM_DEL10	(uint8_t)(1U<<DEL10)
#define    DEL11           7       /* Delay 1 Select bit 1 */
#define    BM_DEL11	(uint8_t)(1U<<DEL11)

/* LFRSS2R - LF RSSI Setting 2 Register */
#define    DEL20           0       /* Delay 2 Select bit 0 */
#define    BM_DEL20	(uint8_t)(1U<<DEL20)
#define    DEL21           1       /* Delay 2 Select bit 1 */
#define    BM_DEL21	(uint8_t)(1U<<DEL21)
#define    DEL30           2       /* Delay 3 Select bit 0 */
#define    BM_DEL30	(uint8_t)(1U<<DEL30)
#define    DEL31           3       /* Delay 3 Select bit 1 */
#define    BM_DEL31	(uint8_t)(1U<<DEL31)
#define    DEL40           4       /* Delay 4 Select bit 0 */
#define    BM_DEL40	(uint8_t)(1U<<DEL40)
#define    DEL41           5       /* Delay 4 Select bit 1 */
#define    BM_DEL41	(uint8_t)(1U<<DEL41)
#define    LFRSNF          7       /* LF RSSI No Finish */
#define    BM_LFRSNF	(uint8_t)(1U<<LFRSNF)


/* ***** MEM ************************** */
/* EEST - EEPROM Status Register */
#define    EESYN0          0       /* EEPROM Syndrome bit 0 */
#define    BM_EESYN0	(uint8_t)(1U<<EESYN0)
#define    EESYN1          1       /* EEPROM Syndrome bit 1 */
#define    BM_EESYN1	(uint8_t)(1U<<EESYN1)
#define    EESYN2          2       /* EEPROM Syndrome bit 2 */
#define    BM_EESYN2	(uint8_t)(1U<<EESYN2)
#define    EESYN3          3       /* EEPROM Syndrome bit 3 */
#define    BM_EESYN3	(uint8_t)(1U<<EESYN3)

/* PGMST - Program Memory Status Register */
#define    PGMSYN0         0       /* Program Memory Syndrome bit 0 */
#define    BM_PGMSYN0	(uint8_t)(1U<<PGMSYN0)
#define    PGMSYN1         1       /* Program Memory Syndrome bit 1 */
#define    BM_PGMSYN1	(uint8_t)(1U<<PGMSYN1)
#define    PGMSYN2         2       /* Program Memory Syndrome bit 2 */
#define    BM_PGMSYN2	(uint8_t)(1U<<PGMSYN2)
#define    PGMSYN3         3       /* Program Memory Syndrome bit 3 */
#define    BM_PGMSYN3	(uint8_t)(1U<<PGMSYN3)
#define    PGMSYN4         4       /* Program Memory Syndrome bit 4 */
#define    BM_PGMSYN4	(uint8_t)(1U<<PGMSYN4)


/* ***** PORTB ************************ */
/* DDRB - Port B Data Direction Register */
#define    DDRB0           0       /* Port B Data Direction Register bit 0 */
#define    BM_DDRB0	(uint8_t)(1U<<DDRB0)
#define    DDRB1           1       /* Port B Data Direction Register bit 1 */
#define    BM_DDRB1	(uint8_t)(1U<<DDRB1)
#define    DDRB2           2       /* Port B Data Direction Register bit 2 */
#define    BM_DDRB2	(uint8_t)(1U<<DDRB2)
#define    DDRB3           3       /* Port B Data Direction Register bit 3 */
#define    BM_DDRB3	(uint8_t)(1U<<DDRB3)
#define    DDRB4           4       /* Port B Data Direction Register bit 4 */
#define    BM_DDRB4	(uint8_t)(1U<<DDRB4)
#define    DDRB5           5       /* Port B Data Direction Register bit 5 */
#define    BM_DDRB5	(uint8_t)(1U<<DDRB5)
#define    DDRB6           6       /* Port B Data Direction Register bit 6 */
#define    BM_DDRB6	(uint8_t)(1U<<DDRB6)
#define    DDRB7           7       /* Port B Data Direction Register bit 7 */
#define    BM_DDRB7	(uint8_t)(1U<<DDRB7)

/* PINB - Port B Input Pins */
#define    PINB0           0       /* Port B Input Pins bit 0 */
#define    BM_PINB0	(uint8_t)(1U<<PINB0)
#define    PINB1           1       /* Port B Input Pins bit 1 */
#define    BM_PINB1	(uint8_t)(1U<<PINB1)
#define    PINB2           2       /* Port B Input Pins bit 2 */
#define    BM_PINB2	(uint8_t)(1U<<PINB2)
#define    PINB3           3       /* Port B Input Pins bit 3 */
#define    BM_PINB3	(uint8_t)(1U<<PINB3)
#define    PINB4           4       /* Port B Input Pins bit 4 */
#define    BM_PINB4	(uint8_t)(1U<<PINB4)
#define    PINB5           5       /* Port B Input Pins bit 5 */
#define    BM_PINB5	(uint8_t)(1U<<PINB5)
#define    PINB6           6       /* Port B Input Pins bit 6 */
#define    BM_PINB6	(uint8_t)(1U<<PINB6)
#define    PINB7           7       /* Port B Input Pins bit 7 */
#define    BM_PINB7	(uint8_t)(1U<<PINB7)

/* PORTB - Port B Data Register */
#define    PORTB0          0       /* Port B Data Register bit 0 */
#define    BM_PORTB0	(uint8_t)(1U<<PORTB0)
#define    PB0             0       /* For compatibility */
#define    BM_PB0	(uint8_t)(1U<<PB0)
#define    PORTB1          1       /* Port B Data Register bit 1 */
#define    BM_PORTB1	(uint8_t)(1U<<PORTB1)
#define    PB1             1       /* For compatibility */
#define    BM_PB1	(uint8_t)(1U<<PB1)
#define    PORTB2          2       /* Port B Data Register bit 2 */
#define    BM_PORTB2	(uint8_t)(1U<<PORTB2)
#define    PB2             2       /* For compatibility */
#define    BM_PB2	(uint8_t)(1U<<PB2)
#define    PORTB3          3       /* Port B Data Register bit 3 */
#define    BM_PORTB3	(uint8_t)(1U<<PORTB3)
#define    PB3             3       /* For compatibility */
#define    BM_PB3	(uint8_t)(1U<<PB3)
#define    PORTB4          4       /* Port B Data Register bit 4 */
#define    BM_PORTB4	(uint8_t)(1U<<PORTB4)
#define    PB4             4       /* For compatibility */
#define    BM_PB4	(uint8_t)(1U<<PB4)
#define    PORTB5          5       /* Port B Data Register bit 5 */
#define    BM_PORTB5	(uint8_t)(1U<<PORTB5)
#define    PB5             5       /* For compatibility */
#define    BM_PB5	(uint8_t)(1U<<PB5)
#define    PORTB6          6       /* Port B Data Register bit 6 */
#define    BM_PORTB6	(uint8_t)(1U<<PORTB6)
#define    PB6             6       /* For compatibility */
#define    BM_PB6	(uint8_t)(1U<<PB6)
#define    PORTB7          7       /* Port B Data Register bit 7 */
#define    BM_PORTB7	(uint8_t)(1U<<PORTB7)
#define    PB7             7       /* For compatibility */
#define    BM_PB7	(uint8_t)(1U<<PB7)


/* ***** PORTC ************************ */
/* DDRC - Port C Data Direction Register */
#define    DDRC0           0       /* Port C Data Direction Register bit 0 */
#define    BM_DDRC0	(uint8_t)(1U<<DDRC0)
#define    DDRC1           1       /* Port C Data Direction Register bit 1 */
#define    BM_DDRC1	(uint8_t)(1U<<DDRC1)
#define    DDRC2           2       /* Port C Data Direction Register bit 2 */
#define    BM_DDRC2	(uint8_t)(1U<<DDRC2)

/* PINC - Port C Input Pins */
#define    PINC0           0       /* Port C Input Pins bit 0 */
#define    BM_PINC0	(uint8_t)(1U<<PINC0)
#define    PINC1           1       /* Port C Input Pins bit 1 */
#define    BM_PINC1	(uint8_t)(1U<<PINC1)
#define    PINC2           2       /* Port C Input Pins bit 2 */
#define    BM_PINC2	(uint8_t)(1U<<PINC2)

/* PORTC - Port C Data Register */
#define    PORTC0          0       /* Port C Data Register bit 0 */
#define    BM_PORTC0	(uint8_t)(1U<<PORTC0)
#define    PC0             0       /* For compatibility */
#define    BM_PC0	(uint8_t)(1U<<PC0)
#define    PORTC1          1       /* Port C Data Register bit 1 */
#define    BM_PORTC1	(uint8_t)(1U<<PORTC1)
#define    PC1             1       /* For compatibility */
#define    BM_PC1	(uint8_t)(1U<<PC1)
#define    PORTC2          2       /* Port C Data Register bit 2 */
#define    BM_PORTC2	(uint8_t)(1U<<PORTC2)
#define    PC2             2       /* For compatibility */
#define    BM_PC2	(uint8_t)(1U<<PC2)


/* ***** PORTD ************************ */
/* DDRD - Port D Data Direction Register */
#define    DDRD0           0       /* Port D Data Direction Register bit 0 */
#define    BM_DDRD0	(uint8_t)(1U<<DDRD0)
#define    DDRD1           1       /* Port D Data Direction Register bit 1 */
#define    BM_DDRD1	(uint8_t)(1U<<DDRD1)
#define    DDRD2           2       /* Port D Data Direction Register bit 2 */
#define    BM_DDRD2	(uint8_t)(1U<<DDRD2)
#define    DDRD3           3       /* Port D Data Direction Register bit 3 */
#define    BM_DDRD3	(uint8_t)(1U<<DDRD3)
#define    DDRD4           4       /* Port D Data Direction Register bit 4 */
#define    BM_DDRD4	(uint8_t)(1U<<DDRD4)
#define    DDRD5           5       /* Port D Data Direction Register bit 5 */
#define    BM_DDRD5	(uint8_t)(1U<<DDRD5)
#define    DDRD6           6       /* Port D Data Direction Register bit 6 */
#define    BM_DDRD6	(uint8_t)(1U<<DDRD6)
#define    DDRD7           7       /* Port D Data Direction Register bit 7 */
#define    BM_DDRD7	(uint8_t)(1U<<DDRD7)

/* PIND - Port D Input Pins */
#define    PIND0           0       /* Port D Input Pins bit 0 */
#define    BM_PIND0	(uint8_t)(1U<<PIND0)
#define    PIND1           1       /* Port D Input Pins bit 1 */
#define    BM_PIND1	(uint8_t)(1U<<PIND1)
#define    PIND2           2       /* Port D Input Pins bit 2 */
#define    BM_PIND2	(uint8_t)(1U<<PIND2)
#define    PIND3           3       /* Port D Input Pins bit 3 */
#define    BM_PIND3	(uint8_t)(1U<<PIND3)
#define    PIND4           4       /* Port D Input Pins bit 4 */
#define    BM_PIND4	(uint8_t)(1U<<PIND4)
#define    PIND5           5       /* Port D Input Pins bit 5 */
#define    BM_PIND5	(uint8_t)(1U<<PIND5)
#define    PIND6           6       /* Port D Input Pins bit 6 */
#define    BM_PIND6	(uint8_t)(1U<<PIND6)
#define    PIND7           7       /* Port D Input Pins bit 7 */
#define    BM_PIND7	(uint8_t)(1U<<PIND7)

/* PORTD - Port D Data Register */
#define    PORTD0          0       /* Port D Data Register bit 0 */
#define    BM_PORTD0	(uint8_t)(1U<<PORTD0)
#define    PD0             0       /* For compatibility */
#define    BM_PD0	(uint8_t)(1U<<PD0)
#define    PORTD1          1       /* Port D Data Register bit 1 */
#define    BM_PORTD1	(uint8_t)(1U<<PORTD1)
#define    PD1             1       /* For compatibility */
#define    BM_PD1	(uint8_t)(1U<<PD1)
#define    PORTD2          2       /* Port D Data Register bit 2 */
#define    BM_PORTD2	(uint8_t)(1U<<PORTD2)
#define    PD2             2       /* For compatibility */
#define    BM_PD2	(uint8_t)(1U<<PD2)
#define    PORTD3          3       /* Port D Data Register bit 3 */
#define    BM_PORTD3	(uint8_t)(1U<<PORTD3)
#define    PD3             3       /* For compatibility */
#define    BM_PD3	(uint8_t)(1U<<PD3)
#define    PORTD4          4       /* Port D Data Register bit 4 */
#define    BM_PORTD4	(uint8_t)(1U<<PORTD4)
#define    PD4             4       /* For compatibility */
#define    BM_PD4	(uint8_t)(1U<<PD4)
#define    PORTD5          5       /* Port D Data Register bit 5 */
#define    BM_PORTD5	(uint8_t)(1U<<PORTD5)
#define    PD5             5       /* For compatibility */
#define    BM_PD5	(uint8_t)(1U<<PD5)
#define    PORTD6          6       /* Port D Data Register bit 6 */
#define    BM_PORTD6	(uint8_t)(1U<<PORTD6)
#define    PD6             6       /* For compatibility */
#define    BM_PD6	(uint8_t)(1U<<PD6)
#define    PORTD7          7       /* Port D Data Register bit 7 */
#define    BM_PORTD7	(uint8_t)(1U<<PORTD7)
#define    PD7             7       /* For compatibility */
#define    BM_PD7	(uint8_t)(1U<<PD7)


/* ***** SFIFO ************************ */
/* SFC - Support FIFO Configuration Register */
#define    SFFLC0          0       /* Support FIFO Fill-Level Configuration bit 0 */
#define    BM_SFFLC0	(uint8_t)(1U<<SFFLC0)
#define    SFFLC1          1       /* Support FIFO Fill-Level Configuration bit 1 */
#define    BM_SFFLC1	(uint8_t)(1U<<SFFLC1)
#define    SFFLC2          2       /* Support FIFO Fill-Level Configuration bit 2 */
#define    BM_SFFLC2	(uint8_t)(1U<<SFFLC2)
#define    SFFLC3          3       /* Support FIFO Fill-Level Configuration bit 3 */
#define    BM_SFFLC3	(uint8_t)(1U<<SFFLC3)
#define    SFFLC4          4       /* Support FIFO Fill-Level Configuration bit 4 */
#define    BM_SFFLC4	(uint8_t)(1U<<SFFLC4)
#define    SFDRA           7       /* Support FIFO Direct Read Access Operational Mode */
#define    BM_SFDRA	(uint8_t)(1U<<SFDRA)

/* SFI - Support FIFO Interrupt Mask Register */
#define    SFFLIM          0       /* Support FIFO Fill-level Interrupt Mask */
#define    BM_SFFLIM	(uint8_t)(1U<<SFFLIM)
#define    SFERIM          1       /* Support FIFO Error Interrupt Mask */
#define    BM_SFERIM	(uint8_t)(1U<<SFERIM)

/* SFL - Support FIFO Fill Level Register */
#define    SFFLS0          0       /* Support FIFO Fill Level Status bit 0 */
#define    BM_SFFLS0	(uint8_t)(1U<<SFFLS0)
#define    SFFLS1          1       /* Support FIFO Fill Level Status bit 1 */
#define    BM_SFFLS1	(uint8_t)(1U<<SFFLS1)
#define    SFFLS2          2       /* Support FIFO Fill Level Status bit 2 */
#define    BM_SFFLS2	(uint8_t)(1U<<SFFLS2)
#define    SFFLS3          3       /* Support FIFO Fill Level Status bit 3 */
#define    BM_SFFLS3	(uint8_t)(1U<<SFFLS3)
#define    SFFLS4          4       /* Support FIFO Fill Level Status bit 4 */
#define    BM_SFFLS4	(uint8_t)(1U<<SFFLS4)
#define    SFCLR           7       /* Support FIFO Clear */
#define    BM_SFCLR	(uint8_t)(1U<<SFCLR)

/* SFRP - Support FIFO Read Pointer */
#define    SFRP0           0       /* Support FIFO Read Pointer bit 0 */
#define    BM_SFRP0	(uint8_t)(1U<<SFRP0)
#define    SFRP1           1       /* Support FIFO Read Pointer bit 1 */
#define    BM_SFRP1	(uint8_t)(1U<<SFRP1)
#define    SFRP2           2       /* Support FIFO Read Pointer bit 2 */
#define    BM_SFRP2	(uint8_t)(1U<<SFRP2)
#define    SFRP3           3       /* Support FIFO Read Pointer bit 3 */
#define    BM_SFRP3	(uint8_t)(1U<<SFRP3)
#define    SFRP4           4       /* Support FIFO Read Pointer bit 4 */
#define    BM_SFRP4	(uint8_t)(1U<<SFRP4)

/* SFS - Support FIFO Status Register */
#define    SFFLRF          0       /* Support FIFO Fill-Level Reached Status Flag */
#define    BM_SFFLRF	(uint8_t)(1U<<SFFLRF)
#define    SFUFL           1       /* Support FIFO Underflow Flag */
#define    BM_SFUFL	(uint8_t)(1U<<SFUFL)
#define    SFOFL           2       /* Support FIFO Overflow Flag */
#define    BM_SFOFL	(uint8_t)(1U<<SFOFL)

/* SFWP - Support FIFO Write Pointer */
#define    SFWP0           0       /* Support FIFO Write Pointer bit 0 */
#define    BM_SFWP0	(uint8_t)(1U<<SFWP0)
#define    SFWP1           1       /* Support FIFO Write Pointer bit 1 */
#define    BM_SFWP1	(uint8_t)(1U<<SFWP1)
#define    SFWP2           2       /* Support FIFO Write Pointer bit 2 */
#define    BM_SFWP2	(uint8_t)(1U<<SFWP2)
#define    SFWP3           3       /* Support FIFO Write Pointer bit 3 */
#define    BM_SFWP3	(uint8_t)(1U<<SFWP3)
#define    SFWP4           4       /* Support FIFO Write Pointer bit 4 */
#define    BM_SFWP4	(uint8_t)(1U<<SFWP4)


/* ***** SPI ************************** */
/* SFFR - SPI FIFO Fill Status Register */
#define    RFL0            0       /* Receive Buffer Fill Level bit 0 */
#define    BM_RFL0	(uint8_t)(1U<<RFL0)
#define    RFL1            1       /* Receive Buffer Fill Level bit 1 */
#define    BM_RFL1	(uint8_t)(1U<<RFL1)
#define    RFL2            2       /* Receive Buffer Fill Level bit 2 */
#define    BM_RFL2	(uint8_t)(1U<<RFL2)
#define    RFC             3       /* Rx Buffer Clear */
#define    BM_RFC	(uint8_t)(1U<<RFC)
#define    TFL0            4       /* Transmit Buffer Fill Level bit 0 */
#define    BM_TFL0	(uint8_t)(1U<<TFL0)
#define    TFL1            5       /* Transmit Buffer Fill Level bit 1 */
#define    BM_TFL1	(uint8_t)(1U<<TFL1)
#define    TFL2            6       /* Transmit Buffer Fill Level bit 2 */
#define    BM_TFL2	(uint8_t)(1U<<TFL2)
#define    TFC             7       /* SPI Tx Buffer Clear */
#define    BM_TFC	(uint8_t)(1U<<TFC)

/* SFIR - SPI FIFO Interrupt Register */
#define    RIL0            0       /* Receive Buffer Interrupt Level bit 0 */
#define    BM_RIL0	(uint8_t)(1U<<RIL0)
#define    RIL1            1       /* Receive Buffer Interrupt Level bit 1 */
#define    BM_RIL1	(uint8_t)(1U<<RIL1)
#define    RIL2            2       /* Receive Buffer Interrupt Level bit 2 */
#define    BM_RIL2	(uint8_t)(1U<<RIL2)
#define    SRIE            3       /* Rx Buffer Interrupt Enable */
#define    BM_SRIE	(uint8_t)(1U<<SRIE)
#define    TIL0            4       /* Transmit Buffer Interrupt Level bit 0 */
#define    BM_TIL0	(uint8_t)(1U<<TIL0)
#define    TIL1            5       /* Transmit Buffer Interrupt Level bit 1 */
#define    BM_TIL1	(uint8_t)(1U<<TIL1)
#define    TIL2            6       /* Transmit Buffer Interrupt Level bit 2 */
#define    BM_TIL2	(uint8_t)(1U<<TIL2)
#define    STIE            7       /* SPI Rx Buffer Interrupt Enable */
#define    BM_STIE	(uint8_t)(1U<<STIE)

/* SPCR - SPI control Register */
#define    SPR0            0       /* SPI Clock Rate Select 0 */
#define    BM_SPR0	(uint8_t)(1U<<SPR0)
#define    SPR1            1       /* SPI Clock Rate Select 1 */
#define    BM_SPR1	(uint8_t)(1U<<SPR1)
#define    CPHA            2       /* Clock Phase */
#define    BM_CPHA	(uint8_t)(1U<<CPHA)
#define    CPOL            3       /* Clock Polarity */
#define    BM_CPOL	(uint8_t)(1U<<CPOL)
#define    MSTR            4       /* Master/Slave Select */
#define    BM_MSTR	(uint8_t)(1U<<MSTR)
#define    DORD            5       /* Data Order */
#define    BM_DORD	(uint8_t)(1U<<DORD)
#define    SPE             6       /* SPI Enable */
#define    BM_SPE	(uint8_t)(1U<<SPE)
#define    SPIE            7       /* Spe Interrupt Enable */
#define    BM_SPIE	(uint8_t)(1U<<SPIE)

/* SPSR - SPI Status Register */
#define    SPI2X           0       /* Double SPI Speed Bit */
#define    BM_SPI2X	(uint8_t)(1U<<SPI2X)
#define    RXIF            4       /* Rx Buffer Interrupt Flag */
#define    BM_RXIF	(uint8_t)(1U<<RXIF)
#define    TXIF            5       /* Tx Buffer Interrupt Flag */
#define    BM_TXIF	(uint8_t)(1U<<TXIF)
#define    SPIF            7       /* SPI Interrupt Flag */
#define    BM_SPIF	(uint8_t)(1U<<SPIF)


/* ***** SSM ************************** */
/* MSMCR1 - Master State Machine Control Register 1 */
#define    MSMSM00         0       /* Master State Machine SubState Machine Select 0 bit 0 */
#define    BM_MSMSM00	(uint8_t)(1U<<MSMSM00)
#define    MSMSM01         1       /* Master State Machine SubState Machine Select 0 bit 1 */
#define    BM_MSMSM01	(uint8_t)(1U<<MSMSM01)
#define    MSMSM02         2       /* Master State Machine SubState Machine Select 0 bit 2 */
#define    BM_MSMSM02	(uint8_t)(1U<<MSMSM02)
#define    MSMSM03         3       /* Master State Machine SubState Machine Select 0 bit 3 */
#define    BM_MSMSM03	(uint8_t)(1U<<MSMSM03)
#define    MSMSM10         4       /* Master State Machine SubState Machine Select 1 bit 0 */
#define    BM_MSMSM10	(uint8_t)(1U<<MSMSM10)
#define    MSMSM11         5       /* Master State Machine SubState Machine Select 1 bit 1 */
#define    BM_MSMSM11	(uint8_t)(1U<<MSMSM11)
#define    MSMSM12         6       /* Master State Machine SubState Machine Select 1 bit 2 */
#define    BM_MSMSM12	(uint8_t)(1U<<MSMSM12)
#define    MSMSM13         7       /* Master State Machine SubState Machine Select 1 bit 3 */
#define    BM_MSMSM13	(uint8_t)(1U<<MSMSM13)

/* MSMCR2 - Master State Machine Control Register 2 */
#define    MSMSM20         0       /* Master State Machine SubState Machine Select 2 bit 0 */
#define    BM_MSMSM20	(uint8_t)(1U<<MSMSM20)
#define    MSMSM21         1       /* Master State Machine SubState Machine Select 2 bit 1 */
#define    BM_MSMSM21	(uint8_t)(1U<<MSMSM21)
#define    MSMSM22         2       /* Master State Machine SubState Machine Select 2 bit 2 */
#define    BM_MSMSM22	(uint8_t)(1U<<MSMSM22)
#define    MSMSM23         3       /* Master State Machine SubState Machine Select 2 bit 3 */
#define    BM_MSMSM23	(uint8_t)(1U<<MSMSM23)
#define    MSMSM30         4       /* Master State Machine SubState Machine Select 3 bit 0 */
#define    BM_MSMSM30	(uint8_t)(1U<<MSMSM30)
#define    MSMSM31         5       /* Master State Machine SubState Machine Select 3 bit 1 */
#define    BM_MSMSM31	(uint8_t)(1U<<MSMSM31)
#define    MSMSM32         6       /* Master State Machine SubState Machine Select 3 bit 2 */
#define    BM_MSMSM32	(uint8_t)(1U<<MSMSM32)
#define    MSMSM33         7       /* Master State Machine SubState Machine Select 3 bit 3 */
#define    BM_MSMSM33	(uint8_t)(1U<<MSMSM33)

/* MSMCR3 - Master State Machine Control Register 3 */
#define    MSMSM40         0       /* Master State Machine SubState Machine Select 4 bit 0 */
#define    BM_MSMSM40	(uint8_t)(1U<<MSMSM40)
#define    MSMSM41         1       /* Master State Machine SubState Machine Select 4 bit 1 */
#define    BM_MSMSM41	(uint8_t)(1U<<MSMSM41)
#define    MSMSM42         2       /* Master State Machine SubState Machine Select 4 bit 2 */
#define    BM_MSMSM42	(uint8_t)(1U<<MSMSM42)
#define    MSMSM43         3       /* Master State Machine SubState Machine Select 4 bit 3 */
#define    BM_MSMSM43	(uint8_t)(1U<<MSMSM43)
#define    MSMSM50         4       /* Master State Machine SubState Machine Select 5 bit 0 */
#define    BM_MSMSM50	(uint8_t)(1U<<MSMSM50)
#define    MSMSM51         5       /* Master State Machine SubState Machine Select 5 bit 1 */
#define    BM_MSMSM51	(uint8_t)(1U<<MSMSM51)
#define    MSMSM52         6       /* Master State Machine SubState Machine Select 5 bit 2 */
#define    BM_MSMSM52	(uint8_t)(1U<<MSMSM52)
#define    MSMSM53         7       /* Master State Machine SubState Machine Select 5 bit 3 */
#define    BM_MSMSM53	(uint8_t)(1U<<MSMSM53)

/* MSMCR4 - Master State Machine Control Register 4 */
#define    MSMSM60         0       /* Master State Machine SubState Machine Select 6 bit 0 */
#define    BM_MSMSM60	(uint8_t)(1U<<MSMSM60)
#define    MSMSM61         1       /* Master State Machine SubState Machine Select 6 bit 1 */
#define    BM_MSMSM61	(uint8_t)(1U<<MSMSM61)
#define    MSMSM62         2       /* Master State Machine SubState Machine Select 6 bit 2 */
#define    BM_MSMSM62	(uint8_t)(1U<<MSMSM62)
#define    MSMSM63         3       /* Master State Machine SubState Machine Select 6 bit 3 */
#define    BM_MSMSM63	(uint8_t)(1U<<MSMSM63)
#define    MSMSM70         4       /* Master State Machine SubState Machine Select 7 bit 0 */
#define    BM_MSMSM70	(uint8_t)(1U<<MSMSM70)
#define    MSMSM71         5       /* Master State Machine SubState Machine Select 7 bit 1 */
#define    BM_MSMSM71	(uint8_t)(1U<<MSMSM71)
#define    MSMSM72         6       /* Master State Machine SubState Machine Select 7 bit 2 */
#define    BM_MSMSM72	(uint8_t)(1U<<MSMSM72)
#define    MSMSM73         7       /* Master State Machine SubState Machine Select 7 bit 3 */
#define    BM_MSMSM73	(uint8_t)(1U<<MSMSM73)

/* MSMSTR - Master State Machine state register */
#define    SSMMST0         0       /* Sequencer State Machine Master State bit 0 */
#define    BM_SSMMST0	(uint8_t)(1U<<SSMMST0)
#define    SSMMST1         1       /* Sequencer State Machine Master State bit 1 */
#define    BM_SSMMST1	(uint8_t)(1U<<SSMMST1)
#define    SSMMST2         2       /* Sequencer State Machine Master State bit 2 */
#define    BM_SSMMST2	(uint8_t)(1U<<SSMMST2)
#define    SSMMST3         3       /* Sequencer State Machine Master State bit 3 */
#define    BM_SSMMST3	(uint8_t)(1U<<SSMMST3)
#define    SSMMST4         4       /* Sequencer State Machine Master State bit 4 */
#define    BM_SSMMST4	(uint8_t)(1U<<SSMMST4)

/* SSMCR - SSM Control Register */
#define    SSMTGE          2       /* Sequencer State Machine Tx Gauss Enable */
#define    BM_SSMTGE	(uint8_t)(1U<<SSMTGE)
#define    SSMTPE          3       /* Sequencer State Machine Tx Preemphasis Enable */
#define    BM_SSMTPE	(uint8_t)(1U<<SSMTPE)
#define    SSMPVE          4       /* Sequencer State Machine PV Enable */
#define    BM_SSMPVE	(uint8_t)(1U<<SSMPVE)
#define    SSMTAE          5       /* Sequencer State Machine Tx Ask-Shaping Enable */
#define    BM_SSMTAE	(uint8_t)(1U<<SSMTAE)

/* SSMFBR - SSM Filter Bandwidth Register */
#define    SSMPLDT         5       /* Sequencer State Machine PLL Lock Delay Time */
#define    BM_SSMPLDT	(uint8_t)(1U<<SSMPLDT)

/* SSMIFR - SSM Interrupt Flag Register */
#define    SSMIF           0       /* Sequencer State Machine Interrupt Flag */
#define    BM_SSMIF	(uint8_t)(1U<<SSMIF)

/* SSMIMR - SSM interrupt mask register */
#define    SSMIM           0       /* Sequencer State Machine Interrupt Mask */
#define    BM_SSMIM	(uint8_t)(1U<<SSMIM)

/* SSMRR - SSM Run Register */
#define    SSMR            0       /* Sequencer State Machine Run */
#define    BM_SSMR	(uint8_t)(1U<<SSMR)
#define    SSMST           1       /* Sequencer State Machine Stop */
#define    BM_SSMST	(uint8_t)(1U<<SSMST)

/* SSMSR - SSM Status Register */
#define    SSMESM0         0       /* Sequencer State Machine Error State Machine bit 0 */
#define    BM_SSMESM0	(uint8_t)(1U<<SSMESM0)
#define    SSMESM1         1       /* Sequencer State Machine Error State Machine bit 1 */
#define    BM_SSMESM1	(uint8_t)(1U<<SSMESM1)
#define    SSMESM2         2       /* Sequencer State Machine Error State Machine bit 2 */
#define    BM_SSMESM2	(uint8_t)(1U<<SSMESM2)
#define    SSMESM3         3       /* Sequencer State Machine Error State Machine bit 3 */
#define    BM_SSMESM3	(uint8_t)(1U<<SSMESM3)
#define    SSMERR          7       /* Sequencer State Machine Error */
#define    BM_SSMERR	(uint8_t)(1U<<SSMERR)

/* SSMSTR - SSM State Register */
#define    SSMSTA0         0       /* Sequencer State Machine State A bit 0 */
#define    BM_SSMSTA0	(uint8_t)(1U<<SSMSTA0)
#define    SSMSTA1         1       /* Sequencer State Machine State A bit 1 */
#define    BM_SSMSTA1	(uint8_t)(1U<<SSMSTA1)
#define    SSMSTA2         2       /* Sequencer State Machine State A bit 2 */
#define    BM_SSMSTA2	(uint8_t)(1U<<SSMSTA2)
#define    SSMSTA3         3       /* Sequencer State Machine State A bit 3 */
#define    BM_SSMSTA3	(uint8_t)(1U<<SSMSTA3)
#define    SSMSTA4         4       /* Sequencer State Machine State A bit 4 */
#define    BM_SSMSTA4	(uint8_t)(1U<<SSMSTA4)
#define    SSMSTA5         5       /* Sequencer State Machine State A bit 5 */
#define    BM_SSMSTA5	(uint8_t)(1U<<SSMSTA5)


/* ***** SUP ************************** */
/* CALRDY - Calibration ready signature */
#define    CALRDY0         0       /* Calibration ready signature bit 0 */
#define    BM_CALRDY0	(uint8_t)(1U<<CALRDY0)
#define    CALRDY1         1       /* Calibration ready signature bit 1 */
#define    BM_CALRDY1	(uint8_t)(1U<<CALRDY1)
#define    CALRDY2         2       /* Calibration ready signature bit 2 */
#define    BM_CALRDY2	(uint8_t)(1U<<CALRDY2)
#define    CALRDY3         3       /* Calibration ready signature bit 3 */
#define    BM_CALRDY3	(uint8_t)(1U<<CALRDY3)
#define    CALRDY4         4       /* Calibration ready signature bit 4 */
#define    BM_CALRDY4	(uint8_t)(1U<<CALRDY4)
#define    CALRDY5         5       /* Calibration ready signature bit 5 */
#define    BM_CALRDY5	(uint8_t)(1U<<CALRDY5)
#define    CALRDY6         6       /* Calibration ready signature bit 6 */
#define    BM_CALRDY6	(uint8_t)(1U<<CALRDY6)
#define    CALRDY7         7       /* Calibration ready signature bit 7 */
#define    BM_CALRDY7	(uint8_t)(1U<<CALRDY7)

/* CALRDYLF - Calibration ready signature LFVCC */
#define    CALRDYLF0       0       /* Calibration ready signature LFVCC bit 0 */
#define    BM_CALRDYLF0	(uint8_t)(1U<<CALRDYLF0)
#define    CALRDYLF1       1       /* Calibration ready signature LFVCC bit 1 */
#define    BM_CALRDYLF1	(uint8_t)(1U<<CALRDYLF1)
#define    CALRDYLF2       2       /* Calibration ready signature LFVCC bit 2 */
#define    BM_CALRDYLF2	(uint8_t)(1U<<CALRDYLF2)
#define    CALRDYLF3       3       /* Calibration ready signature LFVCC bit 3 */
#define    BM_CALRDYLF3	(uint8_t)(1U<<CALRDYLF3)
#define    CALRDYLF4       4       /* Calibration ready signature LFVCC bit 4 */
#define    BM_CALRDYLF4	(uint8_t)(1U<<CALRDYLF4)
#define    CALRDYLF5       5       /* Calibration ready signature LFVCC bit 5 */
#define    BM_CALRDYLF5	(uint8_t)(1U<<CALRDYLF5)
#define    CALRDYLF6       6       /* Calibration ready signature LFVCC bit 6 */
#define    BM_CALRDYLF6	(uint8_t)(1U<<CALRDYLF6)
#define    CALRDYLF7       7       /* Calibration ready signature LFVCC bit 7 */
#define    BM_CALRDYLF7	(uint8_t)(1U<<CALRDYLF7)

/* PMTER - Power Management Test Enable Register */
#define    PMTE0           0       /* Power Management Test Enable bit 0 */
#define    BM_PMTE0	(uint8_t)(1U<<PMTE0)
#define    PMTE1           1       /* Power Management Test Enable bit 1 */
#define    BM_PMTE1	(uint8_t)(1U<<PMTE1)
#define    PMTE2           2       /* Power Management Test Enable bit 2 */
#define    BM_PMTE2	(uint8_t)(1U<<PMTE2)
#define    PMTE3           3       /* Power Management Test Enable bit 3 */
#define    BM_PMTE3	(uint8_t)(1U<<PMTE3)
#define    PMTE4           4       /* Power Management Test Enable bit 4 */
#define    BM_PMTE4	(uint8_t)(1U<<PMTE4)
#define    PMTE5           5       /* Power Management Test Enable bit 5 */
#define    BM_PMTE5	(uint8_t)(1U<<PMTE5)
#define    PMTE6           6       /* Power Management Test Enable bit 6 */
#define    BM_PMTE6	(uint8_t)(1U<<PMTE6)
#define    PMTE7           7       /* Power Management Test Enable bit 7 */
#define    BM_PMTE7	(uint8_t)(1U<<PMTE7)

/* RCTCAL - RC oscillator Temperature Compensation register */
#define    FRCTC           0       /* FRC Oscillator Temperature Compensation bit */
#define    BM_FRCTC	(uint8_t)(1U<<FRCTC)
#define    MRCTC0          1       /* Medium frequency RC Oscillator Temperature Compensation bit 0 */
#define    BM_MRCTC0	(uint8_t)(1U<<MRCTC0)
#define    MRCTC1          2       /* Medium frequency RC Oscillator Temperature Compensation bit 1 */
#define    BM_MRCTC1	(uint8_t)(1U<<MRCTC1)
#define    MRCTC2          3       /* Medium frequency RC Oscillator Temperature Compensation bit 2 */
#define    BM_MRCTC2	(uint8_t)(1U<<MRCTC2)

/* SUPCA1 - Supply calibration register 1 */
#define    PV22            2       /* Power Amplifier Voltage 2.2V */
#define    BM_PV22	(uint8_t)(1U<<PV22)
#define    PVDIC           3       /* Power Amplifier Regulator Double Internal Current */
#define    BM_PVDIC	(uint8_t)(1U<<PVDIC)
#define    PVCAL0          4       /* Power Amplifier Regulator Calibration bit 0 */
#define    BM_PVCAL0	(uint8_t)(1U<<PVCAL0)
#define    PVCAL1          5       /* Power Amplifier Regulator Calibration bit 1 */
#define    BM_PVCAL1	(uint8_t)(1U<<PVCAL1)
#define    PVCAL2          6       /* Power Amplifier Regulator Calibration bit 2 */
#define    BM_PVCAL2	(uint8_t)(1U<<PVCAL2)
#define    PVCAL3          7       /* Power Amplifier Regulator Calibration bit 3 */
#define    BM_PVCAL3	(uint8_t)(1U<<PVCAL3)

/* SUPCA10 - Supply calibration register 10 */
#define    POWMAN0         0       /* Power Management Spare bits bit 0 */
#define    BM_POWMAN0	(uint8_t)(1U<<POWMAN0)
#define    POWMAN1         1       /* Power Management Spare bits bit 1 */
#define    BM_POWMAN1	(uint8_t)(1U<<POWMAN1)
#define    POWMAN2         2       /* Power Management Spare bits bit 2 */
#define    BM_POWMAN2	(uint8_t)(1U<<POWMAN2)
#define    POWMAN3         3       /* Power Management Spare bits bit 3 */
#define    BM_POWMAN3	(uint8_t)(1U<<POWMAN3)
#define    POWMAN4         4       /* Power Management Spare bits bit 4 */
#define    BM_POWMAN4	(uint8_t)(1U<<POWMAN4)

/* SUPCA2 - Supply calibration register 2 */
#define    BGCAL0          0       /* Band Gap Calibration bit 0 */
#define    BM_BGCAL0	(uint8_t)(1U<<BGCAL0)
#define    BGCAL1          1       /* Band Gap Calibration bit 1 */
#define    BM_BGCAL1	(uint8_t)(1U<<BGCAL1)
#define    BGCAL2          2       /* Band Gap Calibration bit 2 */
#define    BM_BGCAL2	(uint8_t)(1U<<BGCAL2)
#define    BGCAL3          3       /* Band Gap Calibration bit 3 */
#define    BM_BGCAL3	(uint8_t)(1U<<BGCAL3)

/* SUPCA3 - Supply calibration register 3 */
#define    ACAL0           0       /* AVCC Regulator Output Voltage Calibration bit 0 */
#define    BM_ACAL0	(uint8_t)(1U<<ACAL0)
#define    ACAL1           1       /* AVCC Regulator Output Voltage Calibration bit 1 */
#define    BM_ACAL1	(uint8_t)(1U<<ACAL1)
#define    ACAL2           2       /* AVCC Regulator Output Voltage Calibration bit 2 */
#define    BM_ACAL2	(uint8_t)(1U<<ACAL2)
#define    ACAL3           3       /* AVCC Regulator Output Voltage Calibration bit 3 */
#define    BM_ACAL3	(uint8_t)(1U<<ACAL3)
#define    ACAL4           4       /* AVCC Regulator Output Voltage Calibration bit 4 */
#define    BM_ACAL4	(uint8_t)(1U<<ACAL4)
#define    ACAL5           5       /* AVCC Regulator Output Voltage Calibration bit 5 */
#define    BM_ACAL5	(uint8_t)(1U<<ACAL5)
#define    ACAL6           6       /* AVCC Regulator Output Voltage Calibration bit 6 */
#define    BM_ACAL6	(uint8_t)(1U<<ACAL6)
#define    ACAL7           7       /* AVCC Regulator Output Voltage Calibration bit 7 */
#define    BM_ACAL7	(uint8_t)(1U<<ACAL7)

/* SUPCA4 - Supply calibration register 4 */
#define    ICONST0         0       /* ICONST Constant current of bandgap bit 0 */
#define    BM_ICONST0	(uint8_t)(1U<<ICONST0)
#define    ICONST1         1       /* ICONST Constant current of bandgap bit 1 */
#define    BM_ICONST1	(uint8_t)(1U<<ICONST1)
#define    ICONST2         2       /* ICONST Constant current of bandgap bit 2 */
#define    BM_ICONST2	(uint8_t)(1U<<ICONST2)
#define    ICONST3         3       /* ICONST Constant current of bandgap bit 3 */
#define    BM_ICONST3	(uint8_t)(1U<<ICONST3)
#define    ICONST4         4       /* ICONST Constant current of bandgap bit 4 */
#define    BM_ICONST4	(uint8_t)(1U<<ICONST4)
#define    ICONST5         5       /* ICONST Constant current of bandgap bit 5 */
#define    BM_ICONST5	(uint8_t)(1U<<ICONST5)

/* SUPCA5 - Supply calibration register 5 */
#define    IPTAT0          0       /* IPTAT current of bandgap bit 0 */
#define    BM_IPTAT0	(uint8_t)(1U<<IPTAT0)
#define    IPTAT1          1       /* IPTAT current of bandgap bit 1 */
#define    BM_IPTAT1	(uint8_t)(1U<<IPTAT1)
#define    IPTAT2          2       /* IPTAT current of bandgap bit 2 */
#define    BM_IPTAT2	(uint8_t)(1U<<IPTAT2)
#define    IPTAT3          3       /* IPTAT current of bandgap bit 3 */
#define    BM_IPTAT3	(uint8_t)(1U<<IPTAT3)
#define    IPTAT4          4       /* IPTAT current of bandgap bit 4 */
#define    BM_IPTAT4	(uint8_t)(1U<<IPTAT4)
#define    IPTAT5          5       /* IPTAT current of bandgap bit 5 */
#define    BM_IPTAT5	(uint8_t)(1U<<IPTAT5)

/* SUPCA6 - Supply calibration register 6 */
#define    VBGTR0          0       /* Threshold voltage of bandgap bit 0 */
#define    BM_VBGTR0	(uint8_t)(1U<<VBGTR0)
#define    VBGTR1          1       /* Threshold voltage of bandgap bit 1 */
#define    BM_VBGTR1	(uint8_t)(1U<<VBGTR1)
#define    VBGTR2          2       /* Threshold voltage of bandgap bit 2 */
#define    BM_VBGTR2	(uint8_t)(1U<<VBGTR2)
#define    VBGTR3          3       /* Threshold voltage of bandgap bit 3 */
#define    BM_VBGTR3	(uint8_t)(1U<<VBGTR3)
#define    VBGTR4          4       /* Threshold voltage of bandgap bit 4 */
#define    BM_VBGTR4	(uint8_t)(1U<<VBGTR4)
#define    VBGTR5          5       /* Threshold voltage of bandgap bit 5 */
#define    BM_VBGTR5	(uint8_t)(1U<<VBGTR5)
#define    VBGTR6          6       /* Threshold voltage of bandgap bit 6 */
#define    BM_VBGTR6	(uint8_t)(1U<<VBGTR6)
#define    VBGTR7          7       /* Threshold voltage of bandgap bit 7 */
#define    BM_VBGTR7	(uint8_t)(1U<<VBGTR7)

/* SUPCA7 - Supply calibration register 7 */
#define    VCCCAL0         0       /* LFVCC and DVCC Output Voltage Calibration bit 0 */
#define    BM_VCCCAL0	(uint8_t)(1U<<VCCCAL0)
#define    VCCCAL1         1       /* LFVCC and DVCC Output Voltage Calibration bit 1 */
#define    BM_VCCCAL1	(uint8_t)(1U<<VCCCAL1)
#define    VCCCAL2         2       /* LFVCC and DVCC Output Voltage Calibration bit 2 */
#define    BM_VCCCAL2	(uint8_t)(1U<<VCCCAL2)
#define    LFVCCBD0        3       /* VREF voltage for LFVCC Brown-out-Detector bit 0 */
#define    BM_LFVCCBD0	(uint8_t)(1U<<LFVCCBD0)
#define    LFVCCBD1        4       /* VREF voltage for LFVCC Brown-out-Detector bit 1 */
#define    BM_LFVCCBD1	(uint8_t)(1U<<LFVCCBD1)
#define    LFVCCBD2        5       /* VREF voltage for LFVCC Brown-out-Detector bit 2 */
#define    BM_LFVCCBD2	(uint8_t)(1U<<LFVCCBD2)

/* SUPCA8 - Supply calibration register 8 */
#define    VSWBD0          0       /* VSIG voltage for LFVCC Brown-out-Detector bit 0 */
#define    BM_VSWBD0	(uint8_t)(1U<<VSWBD0)
#define    VSWBD1          1       /* VSIG voltage for LFVCC Brown-out-Detector bit 1 */
#define    BM_VSWBD1	(uint8_t)(1U<<VSWBD1)
#define    VSWBD2          2       /* VSIG voltage for LFVCC Brown-out-Detector bit 2 */
#define    BM_VSWBD2	(uint8_t)(1U<<VSWBD2)
#define    DVCCBD0         3       /* VREF voltage for DVCC Brown-out-Detector bit 0 */
#define    BM_DVCCBD0	(uint8_t)(1U<<DVCCBD0)
#define    DVCCBD1         4       /* VREF voltage for DVCC Brown-out-Detector bit 1 */
#define    BM_DVCCBD1	(uint8_t)(1U<<DVCCBD1)
#define    DVCCBD2         5       /* VREF voltage for DVCC Brown-out-Detector bit 2 */
#define    BM_DVCCBD2	(uint8_t)(1U<<DVCCBD2)

/* SUPCA9 - Supply calibration register 9 */
#define    VMEM0           0       /* VMEM ldo voltage bit 0 */
#define    BM_VMEM0	(uint8_t)(1U<<VMEM0)
#define    VMEM1           1       /* VMEM ldo voltage bit 1 */
#define    BM_VMEM1	(uint8_t)(1U<<VMEM1)
#define    VMEM2           2       /* VMEM ldo voltage bit 2 */
#define    BM_VMEM2	(uint8_t)(1U<<VMEM2)
#define    VMEM3           3       /* VMEM ldo voltage bit 3 */
#define    BM_VMEM3	(uint8_t)(1U<<VMEM3)
#define    VMEM4           4       /* VMEM ldo voltage bit 4 */
#define    BM_VMEM4	(uint8_t)(1U<<VMEM4)
#define    VMEM5           5       /* VMEM ldo voltage bit 5 */
#define    BM_VMEM5	(uint8_t)(1U<<VMEM5)
#define    VMEM6           6       /* VMEM ldo voltage bit 6 */
#define    BM_VMEM6	(uint8_t)(1U<<VMEM6)
#define    VMEM7           7       /* VMEM ldo voltage bit 7 */
#define    BM_VMEM7	(uint8_t)(1U<<VMEM7)

/* SUPCR - Supply Control Register */
#define    AVCCRM          0       /* AVCC Reset Interrupt Mask */
#define    BM_AVCCRM	(uint8_t)(1U<<AVCCRM)
#define    AVCCLM          1       /* AVCC Low Interrupt Mask */
#define    BM_AVCCLM	(uint8_t)(1U<<AVCCLM)
#define    PVEN            2       /* Power amplifier Voltage supply Enable */
#define    BM_PVEN	(uint8_t)(1U<<PVEN)
#define    AVDIC           3       /* AVCC Double Internal Current */
#define    BM_AVDIC	(uint8_t)(1U<<AVDIC)
#define    AVEN            4       /* AVCC Enable */
#define    BM_AVEN	(uint8_t)(1U<<AVEN)
#define    DVHEN           5       /* DVCC High Current Mode Enable */
#define    BM_DVHEN	(uint8_t)(1U<<DVHEN)
#define    VMRESM          6       /* VMEM Reset Mask */
#define    BM_VMRESM	(uint8_t)(1U<<VMRESM)
#define    VMEMEN          7       /* Memory Voltage Regulator Enable */
#define    BM_VMEMEN	(uint8_t)(1U<<VMEMEN)

/* SUPFR - Supply Interrupt Flag Register */
#define    AVCCRF          0       /* AVCC reset interrupt flag */
#define    BM_AVCCRF	(uint8_t)(1U<<AVCCRF)
#define    AVCCLF          1       /* AVCC low interrupt flag */
#define    BM_AVCCLF	(uint8_t)(1U<<AVCCLF)

/* VMCR - Voltage Monitor Control Register */
#define    VMLS0           0       /* Voltage Monitor Level Select bit 0 */
#define    BM_VMLS0	(uint8_t)(1U<<VMLS0)
#define    VMLS1           1       /* Voltage Monitor Level Select bit 1 */
#define    BM_VMLS1	(uint8_t)(1U<<VMLS1)
#define    VMLS2           2       /* Voltage Monitor Level Select bit 2 */
#define    BM_VMLS2	(uint8_t)(1U<<VMLS2)
#define    VMLS3           3       /* Voltage Monitor Level Select bit 3 */
#define    BM_VMLS3	(uint8_t)(1U<<VMLS3)
#define    VMIM            4       /* Voltage Monitor Interrupt Mask */
#define    BM_VMIM	(uint8_t)(1U<<VMIM)
#define    VMPS0           5       /* Voltage Monitor Power Supply Select 0 */
#define    BM_VMPS0	(uint8_t)(1U<<VMPS0)
#define    VMPS1           6       /* Voltage Monitor Power Supply Select 1 */
#define    BM_VMPS1	(uint8_t)(1U<<VMPS1)
#define    VMRS            7       /* Voltage Monitor Range Select */
#define    BM_VMRS	(uint8_t)(1U<<VMRS)

/* VMSR - Voltage Monitor Status Register */
#define    VMF             0       /* Voltage Monitor Flag */
#define    BM_VMF	(uint8_t)(1U<<VMF)


/* ***** TIMER0_WDT ******************* */
/* T0CR - Timer0 Control Register */
#define    T0PS0           0       /* Timer0 Prescaler Select bit 0 */
#define    BM_T0PS0	(uint8_t)(1U<<T0PS0)
#define    T0PS1           1       /* Timer0 Prescaler Select bit 1 */
#define    BM_T0PS1	(uint8_t)(1U<<T0PS1)
#define    T0PS2           2       /* Timer0 Prescaler Select bit 2 */
#define    BM_T0PS2	(uint8_t)(1U<<T0PS2)
#define    T0IE            3       /* Timer0 Interrupt Enable */
#define    BM_T0IE	(uint8_t)(1U<<T0IE)
#define    T0PR            4       /* Timer0 Prescaler Reset */
#define    BM_T0PR	(uint8_t)(1U<<T0PR)

/* T0IFR - Timer0 Interrupt Flag Register */
#define    T0F             0       /* Timer0 Flag */
#define    BM_T0F	(uint8_t)(1U<<T0F)

/* WDTCR - Watchdog Timer0 control Register */
#define    WDPS0           0       /* Watchdog Prescaler Select bit 0 */
#define    BM_WDPS0	(uint8_t)(1U<<WDPS0)
#define    WDPS1           1       /* Watchdog Prescaler Select bit 1 */
#define    BM_WDPS1	(uint8_t)(1U<<WDPS1)
#define    WDPS2           2       /* Watchdog Prescaler Select bit 2 */
#define    BM_WDPS2	(uint8_t)(1U<<WDPS2)
#define    WDE             3       /* Watchdog System Reset Enable */
#define    BM_WDE	(uint8_t)(1U<<WDE)
#define    WDCE            4       /* Watchdog Change Enable */
#define    BM_WDCE	(uint8_t)(1U<<WDCE)


/* ***** TIMER1 *********************** */
/* T1CNT - Timer1 Counter Register */
#define    T1CNT0          0       /* Timer1 Counter bit 0 */
#define    BM_T1CNT0	(uint8_t)(1U<<T1CNT0)
#define    T1CNT1          1       /* Timer1 Counter bit 1 */
#define    BM_T1CNT1	(uint8_t)(1U<<T1CNT1)
#define    T1CNT2          2       /* Timer1 Counter bit 2 */
#define    BM_T1CNT2	(uint8_t)(1U<<T1CNT2)
#define    T1CNT3          3       /* Timer1 Counter bit 3 */
#define    BM_T1CNT3	(uint8_t)(1U<<T1CNT3)
#define    T1CNT4          4       /* Timer1 Counter bit 4 */
#define    BM_T1CNT4	(uint8_t)(1U<<T1CNT4)
#define    T1CNT5          5       /* Timer1 Counter bit 5 */
#define    BM_T1CNT5	(uint8_t)(1U<<T1CNT5)
#define    T1CNT6          6       /* Timer1 Counter bit 6 */
#define    BM_T1CNT6	(uint8_t)(1U<<T1CNT6)
#define    T1CNT7          7       /* Timer1 Counter bit 7 */
#define    BM_T1CNT7	(uint8_t)(1U<<T1CNT7)

/* T1COR - Timer1 Compare Register */
#define    T1COR0          0       /* Timer1 Compare bit 0 */
#define    BM_T1COR0	(uint8_t)(1U<<T1COR0)
#define    T1COR1          1       /* Timer1 Compare bit 1 */
#define    BM_T1COR1	(uint8_t)(1U<<T1COR1)
#define    T1COR2          2       /* Timer1 Compare bit 2 */
#define    BM_T1COR2	(uint8_t)(1U<<T1COR2)
#define    T1COR3          3       /* Timer1 Compare bit 3 */
#define    BM_T1COR3	(uint8_t)(1U<<T1COR3)
#define    T1COR4          4       /* Timer1 Compare bit 4 */
#define    BM_T1COR4	(uint8_t)(1U<<T1COR4)
#define    T1COR5          5       /* Timer1 Compare bit 5 */
#define    BM_T1COR5	(uint8_t)(1U<<T1COR5)
#define    T1COR6          6       /* Timer1 Compare bit 6 */
#define    BM_T1COR6	(uint8_t)(1U<<T1COR6)
#define    T1COR7          7       /* Timer1 Compare bit 7 */
#define    BM_T1COR7	(uint8_t)(1U<<T1COR7)

/* T1CR - Timer1 control Register */
#define    T1OTM           0       /* Timer1 Overflow Toggle Mask */
#define    BM_T1OTM	(uint8_t)(1U<<T1OTM)
#define    T1CTM           1       /* Timer1 Compare Toggle Mask */
#define    BM_T1CTM	(uint8_t)(1U<<T1CTM)
#define    T1CRM           2       /* Timer1 Compare Reset Mask */
#define    BM_T1CRM	(uint8_t)(1U<<T1CRM)
#define    T1TOP           4       /* Timer1 Toggle Output Preset */
#define    BM_T1TOP	(uint8_t)(1U<<T1TOP)
#define    T1RES           5       /* Timer1 Reset */
#define    BM_T1RES	(uint8_t)(1U<<T1RES)
#define    T1TOS           6       /* Timer1 Toggle with Start */
#define    BM_T1TOS	(uint8_t)(1U<<T1TOS)
#define    T1ENA           7       /* Timer1 Enable */
#define    BM_T1ENA	(uint8_t)(1U<<T1ENA)

/* T1IFR - Timer1 Interrupt Flag Register */
#define    T1OFF           0       /* Timer1 Overflow Flag */
#define    BM_T1OFF	(uint8_t)(1U<<T1OFF)
#define    T1COF           1       /* Timer1 Compare Flag */
#define    BM_T1COF	(uint8_t)(1U<<T1COF)

/* T1IMR - Timer1 Interrupt Mask Register */
#define    T1OIM           0       /* Timer1 Overflow Interrupt Mask */
#define    BM_T1OIM	(uint8_t)(1U<<T1OIM)
#define    T1CIM           1       /* Timer1 Compare Interrupt Mask */
#define    BM_T1CIM	(uint8_t)(1U<<T1CIM)

/* T1MR - Timer1 Mode Register */
#define    T1CS0           0       /* Timer1 Clock Select bit 0 */
#define    BM_T1CS0	(uint8_t)(1U<<T1CS0)
#define    T1CS1           1       /* Timer1 Clock Select bit 1 */
#define    BM_T1CS1	(uint8_t)(1U<<T1CS1)
#define    T1PS0           2       /* Timer1 Prescaler Select bit 0 */
#define    BM_T1PS0	(uint8_t)(1U<<T1PS0)
#define    T1PS1           3       /* Timer1 Prescaler Select bit 1 */
#define    BM_T1PS1	(uint8_t)(1U<<T1PS1)
#define    T1PS2           4       /* Timer1 Prescaler Select bit 2 */
#define    BM_T1PS2	(uint8_t)(1U<<T1PS2)
#define    T1PS3           5       /* Timer1 Prescaler Select bit 3 */
#define    BM_T1PS3	(uint8_t)(1U<<T1PS3)
#define    T1DC0           6       /* Timer1 Duty Cycle bit 0 */
#define    BM_T1DC0	(uint8_t)(1U<<T1DC0)
#define    T1DC1           7       /* Timer1 Duty Cycle bit 1 */
#define    BM_T1DC1	(uint8_t)(1U<<T1DC1)


/* ***** TIMER2 *********************** */
/* T2CNT - Timer2 Counter Register */
#define    T2CNT0          0       /* Timer2 Counter bit 0 */
#define    BM_T2CNT0	(uint8_t)(1U<<T2CNT0)
#define    T2CNT1          1       /* Timer2 Counter bit 1 */
#define    BM_T2CNT1	(uint8_t)(1U<<T2CNT1)
#define    T2CNT2          2       /* Timer2 Counter bit 2 */
#define    BM_T2CNT2	(uint8_t)(1U<<T2CNT2)
#define    T2CNT3          3       /* Timer2 Counter bit 3 */
#define    BM_T2CNT3	(uint8_t)(1U<<T2CNT3)
#define    T2CNT4          4       /* Timer2 Counter bit 4 */
#define    BM_T2CNT4	(uint8_t)(1U<<T2CNT4)
#define    T2CNT5          5       /* Timer2 Counter bit 5 */
#define    BM_T2CNT5	(uint8_t)(1U<<T2CNT5)
#define    T2CNT6          6       /* Timer2 Counter bit 6 */
#define    BM_T2CNT6	(uint8_t)(1U<<T2CNT6)
#define    T2CNT7          7       /* Timer2 Counter bit 7 */
#define    BM_T2CNT7	(uint8_t)(1U<<T2CNT7)

/* T2COR - Timer2 Compare Register */
#define    T2COR0          0       /* Timer2 Compare bit 0 */
#define    BM_T2COR0	(uint8_t)(1U<<T2COR0)
#define    T2COR1          1       /* Timer2 Compare bit 1 */
#define    BM_T2COR1	(uint8_t)(1U<<T2COR1)
#define    T2COR2          2       /* Timer2 Compare bit 2 */
#define    BM_T2COR2	(uint8_t)(1U<<T2COR2)
#define    T2COR3          3       /* Timer2 Compare bit 3 */
#define    BM_T2COR3	(uint8_t)(1U<<T2COR3)
#define    T2COR4          4       /* Timer2 Compare bit 4 */
#define    BM_T2COR4	(uint8_t)(1U<<T2COR4)
#define    T2COR5          5       /* Timer2 Compare bit 5 */
#define    BM_T2COR5	(uint8_t)(1U<<T2COR5)
#define    T2COR6          6       /* Timer2 Compare bit 6 */
#define    BM_T2COR6	(uint8_t)(1U<<T2COR6)
#define    T2COR7          7       /* Timer2 Compare bit 7 */
#define    BM_T2COR7	(uint8_t)(1U<<T2COR7)

/* T2CR - Timer2 Control Register */
#define    T2OTM           0       /* Timer2 Overflow Toggle Mask */
#define    BM_T2OTM	(uint8_t)(1U<<T2OTM)
#define    T2CTM           1       /* Timer2 Compare Toggle Mask */
#define    BM_T2CTM	(uint8_t)(1U<<T2CTM)
#define    T2CRM           2       /* Timer2 Compare Reset Mask */
#define    BM_T2CRM	(uint8_t)(1U<<T2CRM)
#define    T2TOP           4       /* Timer2 Toggle Output Preset */
#define    BM_T2TOP	(uint8_t)(1U<<T2TOP)
#define    T2RES           5       /* Timer2 Reset */
#define    BM_T2RES	(uint8_t)(1U<<T2RES)
#define    T2TOS           6       /* Timer2 Toggle with Start */
#define    BM_T2TOS	(uint8_t)(1U<<T2TOS)
#define    T2ENA           7       /* Timer2 Enable */
#define    BM_T2ENA	(uint8_t)(1U<<T2ENA)

/* T2IFR - Timer2 Interrupt Flag Register */
#define    T2OFF           0       /* Timer2 Overflow Flag */
#define    BM_T2OFF	(uint8_t)(1U<<T2OFF)
#define    T2COF           1       /* Timer2 Compare Flag */
#define    BM_T2COF	(uint8_t)(1U<<T2COF)

/* T2IMR - Timer2 Interrupt Mask Register */
#define    T2OIM           0       /* Timer2 Overflow Interrupt Mask */
#define    BM_T2OIM	(uint8_t)(1U<<T2OIM)
#define    T2CIM           1       /* Timer2 Compare Interrupt Mask */
#define    BM_T2CIM	(uint8_t)(1U<<T2CIM)

/* T2MR - Timer2 Mode Register */
#define    T2CS0           0       /* Timer2 Clock Select bit 0 */
#define    BM_T2CS0	(uint8_t)(1U<<T2CS0)
#define    T2CS1           1       /* Timer2 Clock Select bit 1 */
#define    BM_T2CS1	(uint8_t)(1U<<T2CS1)
#define    T2PS0           2       /* Timer2 Prescaler Select bit 0 */
#define    BM_T2PS0	(uint8_t)(1U<<T2PS0)
#define    T2PS1           3       /* Timer2 Prescaler Select bit 1 */
#define    BM_T2PS1	(uint8_t)(1U<<T2PS1)
#define    T2PS2           4       /* Timer2 Prescaler Select bit 2 */
#define    BM_T2PS2	(uint8_t)(1U<<T2PS2)
#define    T2PS3           5       /* Timer2 Prescaler Select bit 3 */
#define    BM_T2PS3	(uint8_t)(1U<<T2PS3)
#define    T2DC0           6       /* Timer2 Duty Cycle bit 0 */
#define    BM_T2DC0	(uint8_t)(1U<<T2DC0)
#define    T2DC1           7       /* Timer2 Duty Cycle bit 1 */
#define    BM_T2DC1	(uint8_t)(1U<<T2DC1)


/* ***** TIMER3 *********************** */
/* T3CNTH - Timer3 counter High Byte Register */
#define    T3CNT8          0       /* Timer3 counter bit 8 */
#define    BM_T3CNT8	(uint8_t)(1U<<T3CNT8)
#define    T3CNT9          1       /* Timer3 counter bit 9 */
#define    BM_T3CNT9	(uint8_t)(1U<<T3CNT9)
#define    T3CNT10         2       /* Timer3 counter bit 10 */
#define    BM_T3CNT10	(uint8_t)(1U<<T3CNT10)
#define    T3CNT11         3       /* Timer3 counter bit 11 */
#define    BM_T3CNT11	(uint8_t)(1U<<T3CNT11)
#define    T3CNT12         4       /* Timer3 counter bit 12 */
#define    BM_T3CNT12	(uint8_t)(1U<<T3CNT12)
#define    T3CNT13         5       /* Timer3 counter bit 13 */
#define    BM_T3CNT13	(uint8_t)(1U<<T3CNT13)
#define    T3CNT14         6       /* Timer3 counter bit 14 */
#define    BM_T3CNT14	(uint8_t)(1U<<T3CNT14)
#define    T3CNT15         7       /* Timer3 counter bit 15 */
#define    BM_T3CNT15	(uint8_t)(1U<<T3CNT15)

/* T3CNTL - Timer3 counter Low Byte Register */
#define    T3CNT0          0       /* Timer3 counter bit 0 */
#define    BM_T3CNT0	(uint8_t)(1U<<T3CNT0)
#define    T3CNT1          1       /* Timer3 counter bit 1 */
#define    BM_T3CNT1	(uint8_t)(1U<<T3CNT1)
#define    T3CNT2          2       /* Timer3 counter bit 2 */
#define    BM_T3CNT2	(uint8_t)(1U<<T3CNT2)
#define    T3CNT3          3       /* Timer3 counter bit 3 */
#define    BM_T3CNT3	(uint8_t)(1U<<T3CNT3)
#define    T3CNT4          4       /* Timer3 counter bit 4 */
#define    BM_T3CNT4	(uint8_t)(1U<<T3CNT4)
#define    T3CNT5          5       /* Timer3 counter bit 5 */
#define    BM_T3CNT5	(uint8_t)(1U<<T3CNT5)
#define    T3CNT6          6       /* Timer3 counter bit 6 */
#define    BM_T3CNT6	(uint8_t)(1U<<T3CNT6)
#define    T3CNT7          7       /* Timer3 counter bit 7 */
#define    BM_T3CNT7	(uint8_t)(1U<<T3CNT7)

/* T3CORH - Timer3 compare High Byte Register */
#define    T3COR8          0       /* Timer3 compare bit 8 */
#define    BM_T3COR8	(uint8_t)(1U<<T3COR8)
#define    T3COR9          1       /* Timer3 compare bit 9 */
#define    BM_T3COR9	(uint8_t)(1U<<T3COR9)
#define    T3COR10         2       /* Timer3 compare bit 10 */
#define    BM_T3COR10	(uint8_t)(1U<<T3COR10)
#define    T3COR11         3       /* Timer3 compare bit 11 */
#define    BM_T3COR11	(uint8_t)(1U<<T3COR11)
#define    T3COR12         4       /* Timer3 compare bit 12 */
#define    BM_T3COR12	(uint8_t)(1U<<T3COR12)
#define    T3COR13         5       /* Timer3 compare bit 13 */
#define    BM_T3COR13	(uint8_t)(1U<<T3COR13)
#define    T3COR14         6       /* Timer3 compare bit 14 */
#define    BM_T3COR14	(uint8_t)(1U<<T3COR14)
#define    T3COR15         7       /* Timer3 compare bit 15 */
#define    BM_T3COR15	(uint8_t)(1U<<T3COR15)

/* T3CORL - Timer3 compare Low Byte Register */
#define    T3COR0          0       /* Timer3 compare bit 0 */
#define    BM_T3COR0	(uint8_t)(1U<<T3COR0)
#define    T3COR1          1       /* Timer3 compare bit 1 */
#define    BM_T3COR1	(uint8_t)(1U<<T3COR1)
#define    T3COR2          2       /* Timer3 compare bit 2 */
#define    BM_T3COR2	(uint8_t)(1U<<T3COR2)
#define    T3COR3          3       /* Timer3 compare bit 3 */
#define    BM_T3COR3	(uint8_t)(1U<<T3COR3)
#define    T3COR4          4       /* Timer3 compare bit 4 */
#define    BM_T3COR4	(uint8_t)(1U<<T3COR4)
#define    T3COR5          5       /* Timer3 compare bit 5 */
#define    BM_T3COR5	(uint8_t)(1U<<T3COR5)
#define    T3COR6          6       /* Timer3 compare bit 6 */
#define    BM_T3COR6	(uint8_t)(1U<<T3COR6)
#define    T3COR7          7       /* Timer3 compare bit 7 */
#define    BM_T3COR7	(uint8_t)(1U<<T3COR7)

/* T3CR - Timer3 control Register */
#define    T3OTM           0       /* Timer3 Overflow Toggle Mask */
#define    BM_T3OTM	(uint8_t)(1U<<T3OTM)
#define    T3CTM           1       /* Timer3 Compare Toggle Mask */
#define    BM_T3CTM	(uint8_t)(1U<<T3CTM)
#define    T3CRM           2       /* Timer3 Compare Reset Mask */
#define    BM_T3CRM	(uint8_t)(1U<<T3CRM)
#define    T3CPRM          3       /* Timer3 CaPture Reset Mask */
#define    BM_T3CPRM	(uint8_t)(1U<<T3CPRM)
#define    T3TOP           4       /* Timer3 Toggle Output Preset */
#define    BM_T3TOP	(uint8_t)(1U<<T3TOP)
#define    T3RES           5       /* Timer3 Reset */
#define    BM_T3RES	(uint8_t)(1U<<T3RES)
#define    T3TOS           6       /* Timer3 Toggle with Start */
#define    BM_T3TOS	(uint8_t)(1U<<T3TOS)
#define    T3ENA           7       /* Timer3 Enable */
#define    BM_T3ENA	(uint8_t)(1U<<T3ENA)

/* T3ICRH - Timer3 input capture High Byte Register */
#define    T3ICR8          0       /* Timer3 input capture bit 8 */
#define    BM_T3ICR8	(uint8_t)(1U<<T3ICR8)
#define    T3ICR9          1       /* Timer3 input capture bit 9 */
#define    BM_T3ICR9	(uint8_t)(1U<<T3ICR9)
#define    T3ICR10         2       /* Timer3 input capture bit 10 */
#define    BM_T3ICR10	(uint8_t)(1U<<T3ICR10)
#define    T3ICR11         3       /* Timer3 input capture bit 11 */
#define    BM_T3ICR11	(uint8_t)(1U<<T3ICR11)
#define    T3ICR12         4       /* Timer3 input capture bit 12 */
#define    BM_T3ICR12	(uint8_t)(1U<<T3ICR12)
#define    T3ICR13         5       /* Timer3 input capture bit 13 */
#define    BM_T3ICR13	(uint8_t)(1U<<T3ICR13)
#define    T3ICR14         6       /* Timer3 input capture bit 14 */
#define    BM_T3ICR14	(uint8_t)(1U<<T3ICR14)
#define    T3ICR15         7       /* Timer3 input capture bit 15 */
#define    BM_T3ICR15	(uint8_t)(1U<<T3ICR15)

/* T3ICRL - Timer3 input capture Low Byte Register */
#define    T3ICR0          0       /* Timer3 input capture bit 0 */
#define    BM_T3ICR0	(uint8_t)(1U<<T3ICR0)
#define    T3ICR1          1       /* Timer3 input capture bit 1 */
#define    BM_T3ICR1	(uint8_t)(1U<<T3ICR1)
#define    T3ICR2          2       /* Timer3 input capture bit 2 */
#define    BM_T3ICR2	(uint8_t)(1U<<T3ICR2)
#define    T3ICR3          3       /* Timer3 input capture bit 3 */
#define    BM_T3ICR3	(uint8_t)(1U<<T3ICR3)
#define    T3ICR4          4       /* Timer3 input capture bit 4 */
#define    BM_T3ICR4	(uint8_t)(1U<<T3ICR4)
#define    T3ICR5          5       /* Timer3 input capture bit 5 */
#define    BM_T3ICR5	(uint8_t)(1U<<T3ICR5)
#define    T3ICR6          6       /* Timer3 input capture bit 6 */
#define    BM_T3ICR6	(uint8_t)(1U<<T3ICR6)
#define    T3ICR7          7       /* Timer3 input capture bit 7 */
#define    BM_T3ICR7	(uint8_t)(1U<<T3ICR7)

/* T3IFR - Timer3 interrupt flag Register */
#define    T3OFF           0       /* Timer3 OverFlow Flag */
#define    BM_T3OFF	(uint8_t)(1U<<T3OFF)
#define    T3COF           1       /* Timer3 Compare Flag */
#define    BM_T3COF	(uint8_t)(1U<<T3COF)
#define    T3ICF           2       /* Timer3 Input Capture Flag */
#define    BM_T3ICF	(uint8_t)(1U<<T3ICF)

/* T3IMR - Timer3 interrupt mask Register */
#define    T3OIM           0       /* Timer3 Overflow Interrupt Mask */
#define    BM_T3OIM	(uint8_t)(1U<<T3OIM)
#define    T3CIM           1       /* Timer3 Compare Interrupt Mask */
#define    BM_T3CIM	(uint8_t)(1U<<T3CIM)
#define    T3CPIM          2       /* Timer3 Capture Interrupt Mask */
#define    BM_T3CPIM	(uint8_t)(1U<<T3CPIM)

/* T3MRA - Timer3 mode Register */
#define    T3CS0           0       /* Timer3 Clock Select bit 0 */
#define    BM_T3CS0	(uint8_t)(1U<<T3CS0)
#define    T3CS1           1       /* Timer3 Clock Select bit 1 */
#define    BM_T3CS1	(uint8_t)(1U<<T3CS1)
#define    T3PS0           2       /* Timer3 Prescaler Select bit 0 */
#define    BM_T3PS0	(uint8_t)(1U<<T3PS0)
#define    T3PS1           3       /* Timer3 Prescaler Select bit 1 */
#define    BM_T3PS1	(uint8_t)(1U<<T3PS1)
#define    T3PS2           4       /* Timer3 Prescaler Select bit 2 */
#define    BM_T3PS2	(uint8_t)(1U<<T3PS2)

/* T3MRB - Timer3 mode Register */
#define    T3SCE           1       /* Timer3 Software Capture Enable */
#define    BM_T3SCE	(uint8_t)(1U<<T3SCE)
#define    T3CNC           2       /* Timer3 input Capture Noise Canceller */
#define    BM_T3CNC	(uint8_t)(1U<<T3CNC)
#define    T3CE0           3       /* Timer3 Capture Edge select bit 0 */
#define    BM_T3CE0	(uint8_t)(1U<<T3CE0)
#define    T3CE1           4       /* Timer3 Capture Edge select bit 1 */
#define    BM_T3CE1	(uint8_t)(1U<<T3CE1)
#define    T3ICS0          5       /* Timer3 Input Capture Select bit 0 */
#define    BM_T3ICS0	(uint8_t)(1U<<T3ICS0)
#define    T3ICS1          6       /* Timer3 Input Capture Select bit 1 */
#define    BM_T3ICS1	(uint8_t)(1U<<T3ICS1)
#define    T3ICS2          7       /* Timer3 Input Capture Select bit 2 */
#define    BM_T3ICS2	(uint8_t)(1U<<T3ICS2)


/* ***** TIMER4 *********************** */
/* T4CNTH - Timer4 counter High Byte Register */
#define    T4CNT8          0       /* Timer4 counter bit 8 */
#define    BM_T4CNT8	(uint8_t)(1U<<T4CNT8)
#define    T4CNT9          1       /* Timer4 counter bit 9 */
#define    BM_T4CNT9	(uint8_t)(1U<<T4CNT9)
#define    T4CNT10         2       /* Timer4 counter bit 10 */
#define    BM_T4CNT10	(uint8_t)(1U<<T4CNT10)
#define    T4CNT11         3       /* Timer4 counter bit 11 */
#define    BM_T4CNT11	(uint8_t)(1U<<T4CNT11)
#define    T4CNT12         4       /* Timer4 counter bit 12 */
#define    BM_T4CNT12	(uint8_t)(1U<<T4CNT12)
#define    T4CNT13         5       /* Timer4 counter bit 13 */
#define    BM_T4CNT13	(uint8_t)(1U<<T4CNT13)
#define    T4CNT14         6       /* Timer4 counter bit 14 */
#define    BM_T4CNT14	(uint8_t)(1U<<T4CNT14)
#define    T4CNT15         7       /* Timer4 counter bit 15 */
#define    BM_T4CNT15	(uint8_t)(1U<<T4CNT15)

/* T4CNTL - Timer4 counter Low Byte Register */
#define    T4CNT0          0       /* Timer4 counter bit 0 */
#define    BM_T4CNT0	(uint8_t)(1U<<T4CNT0)
#define    T4CNT1          1       /* Timer4 counter bit 1 */
#define    BM_T4CNT1	(uint8_t)(1U<<T4CNT1)
#define    T4CNT2          2       /* Timer4 counter bit 2 */
#define    BM_T4CNT2	(uint8_t)(1U<<T4CNT2)
#define    T4CNT3          3       /* Timer4 counter bit 3 */
#define    BM_T4CNT3	(uint8_t)(1U<<T4CNT3)
#define    T4CNT4          4       /* Timer4 counter bit 4 */
#define    BM_T4CNT4	(uint8_t)(1U<<T4CNT4)
#define    T4CNT5          5       /* Timer4 counter bit 5 */
#define    BM_T4CNT5	(uint8_t)(1U<<T4CNT5)
#define    T4CNT6          6       /* Timer4 counter bit 6 */
#define    BM_T4CNT6	(uint8_t)(1U<<T4CNT6)
#define    T4CNT7          7       /* Timer4 counter bit 7 */
#define    BM_T4CNT7	(uint8_t)(1U<<T4CNT7)

/* T4CORH - Timer4 compare high Byte Register */
#define    T4COR8          0       /* Timer4 compare bit 8 */
#define    BM_T4COR8	(uint8_t)(1U<<T4COR8)
#define    T4COR9          1       /* Timer4 compare bit 9 */
#define    BM_T4COR9	(uint8_t)(1U<<T4COR9)
#define    T4COR10         2       /* Timer4 compare bit 10 */
#define    BM_T4COR10	(uint8_t)(1U<<T4COR10)
#define    T4COR11         3       /* Timer4 compare bit 11 */
#define    BM_T4COR11	(uint8_t)(1U<<T4COR11)
#define    T4COR12         4       /* Timer4 compare bit 12 */
#define    BM_T4COR12	(uint8_t)(1U<<T4COR12)
#define    T4COR13         5       /* Timer4 compare bit 13 */
#define    BM_T4COR13	(uint8_t)(1U<<T4COR13)
#define    T4COR14         6       /* Timer4 compare bit 14 */
#define    BM_T4COR14	(uint8_t)(1U<<T4COR14)
#define    T4COR15         7       /* Timer4 compare bit 15 */
#define    BM_T4COR15	(uint8_t)(1U<<T4COR15)

/* T4CORL - Timer4 compare Low Byte Register */
#define    T4COR0          0       /* Timer4 compare bit 0 */
#define    BM_T4COR0	(uint8_t)(1U<<T4COR0)
#define    T4COR1          1       /* Timer4 compare bit 1 */
#define    BM_T4COR1	(uint8_t)(1U<<T4COR1)
#define    T4COR2          2       /* Timer4 compare bit 2 */
#define    BM_T4COR2	(uint8_t)(1U<<T4COR2)
#define    T4COR3          3       /* Timer4 compare bit 3 */
#define    BM_T4COR3	(uint8_t)(1U<<T4COR3)
#define    T4COR4          4       /* Timer4 compare bit 4 */
#define    BM_T4COR4	(uint8_t)(1U<<T4COR4)
#define    T4COR5          5       /* Timer4 compare bit 5 */
#define    BM_T4COR5	(uint8_t)(1U<<T4COR5)
#define    T4COR6          6       /* Timer4 compare bit 6 */
#define    BM_T4COR6	(uint8_t)(1U<<T4COR6)
#define    T4COR7          7       /* Timer4 compare bit 7 */
#define    BM_T4COR7	(uint8_t)(1U<<T4COR7)

/* T4CR - Timer4 control Register */
#define    T4OTM           0       /* Timer4 Overflow Toggle Mask */
#define    BM_T4OTM	(uint8_t)(1U<<T4OTM)
#define    T4CTM           1       /* Timer4 Compare Toggle Mask */
#define    BM_T4CTM	(uint8_t)(1U<<T4CTM)
#define    T4CRM           2       /* Timer4 Compare Reset Mask */
#define    BM_T4CRM	(uint8_t)(1U<<T4CRM)
#define    T4CPRM          3       /* Timer4 CaPture Reset Mask */
#define    BM_T4CPRM	(uint8_t)(1U<<T4CPRM)
#define    T4TOP           4       /* Timer4 Toggle Output Preset */
#define    BM_T4TOP	(uint8_t)(1U<<T4TOP)
#define    T4RES           5       /* Timer4 Reset */
#define    BM_T4RES	(uint8_t)(1U<<T4RES)
#define    T4TOS           6       /* Timer4 Toggle with Start */
#define    BM_T4TOS	(uint8_t)(1U<<T4TOS)
#define    T4ENA           7       /* Timer4 Enable */
#define    BM_T4ENA	(uint8_t)(1U<<T4ENA)

/* T4ICRH - Timer4 input capture High Byte Register */
#define    T4ICR8          0       /* Timer4 input capture bit 8 */
#define    BM_T4ICR8	(uint8_t)(1U<<T4ICR8)
#define    T4ICR9          1       /* Timer4 input capture bit 9 */
#define    BM_T4ICR9	(uint8_t)(1U<<T4ICR9)
#define    T4ICR10         2       /* Timer4 input capture bit 10 */
#define    BM_T4ICR10	(uint8_t)(1U<<T4ICR10)
#define    T4ICR11         3       /* Timer4 input capture bit 11 */
#define    BM_T4ICR11	(uint8_t)(1U<<T4ICR11)
#define    T4ICR12         4       /* Timer4 input capture bit 12 */
#define    BM_T4ICR12	(uint8_t)(1U<<T4ICR12)
#define    T4ICR13         5       /* Timer4 input capture bit 13 */
#define    BM_T4ICR13	(uint8_t)(1U<<T4ICR13)
#define    T4ICR14         6       /* Timer4 input capture bit 14 */
#define    BM_T4ICR14	(uint8_t)(1U<<T4ICR14)
#define    T4ICR15         7       /* Timer4 input capture bit 15 */
#define    BM_T4ICR15	(uint8_t)(1U<<T4ICR15)

/* T4ICRL - Timer4 input capture Low Byte Register */
#define    T4ICR0          0       /* Timer4 input capture bit 0 */
#define    BM_T4ICR0	(uint8_t)(1U<<T4ICR0)
#define    T4ICR1          1       /* Timer4 input capture bit 1 */
#define    BM_T4ICR1	(uint8_t)(1U<<T4ICR1)
#define    T4ICR2          2       /* Timer4 input capture bit 2 */
#define    BM_T4ICR2	(uint8_t)(1U<<T4ICR2)
#define    T4ICR3          3       /* Timer4 input capture bit 3 */
#define    BM_T4ICR3	(uint8_t)(1U<<T4ICR3)
#define    T4ICR4          4       /* Timer4 input capture bit 4 */
#define    BM_T4ICR4	(uint8_t)(1U<<T4ICR4)
#define    T4ICR5          5       /* Timer4 input capture bit 5 */
#define    BM_T4ICR5	(uint8_t)(1U<<T4ICR5)
#define    T4ICR6          6       /* Timer4 input capture bit 6 */
#define    BM_T4ICR6	(uint8_t)(1U<<T4ICR6)
#define    T4ICR7          7       /* Timer4 input capture bit 7 */
#define    BM_T4ICR7	(uint8_t)(1U<<T4ICR7)

/* T4IFR - Timer4 interrupt flag Register */
#define    T4OFF           0       /* Timer4 OverFlow Flag */
#define    BM_T4OFF	(uint8_t)(1U<<T4OFF)
#define    T4COF           1       /* Timer4 Compare Flag */
#define    BM_T4COF	(uint8_t)(1U<<T4COF)
#define    T4ICF           2       /* Timer4 Input Capture Flag */
#define    BM_T4ICF	(uint8_t)(1U<<T4ICF)

/* T4IMR - Timer4 interrupt mask Register */
#define    T4OIM           0       /* Timer4 Overflow Interrupt Mask */
#define    BM_T4OIM	(uint8_t)(1U<<T4OIM)
#define    T4CIM           1       /* Timer4 Compare Interrupt Mask */
#define    BM_T4CIM	(uint8_t)(1U<<T4CIM)
#define    T4CPIM          2       /* Timer4 Capture Interrupt Mask */
#define    BM_T4CPIM	(uint8_t)(1U<<T4CPIM)

/* T4MRA - Timer4 mode Register */
#define    T4CS0           0       /* Timer4 Clock Select bit 0 */
#define    BM_T4CS0	(uint8_t)(1U<<T4CS0)
#define    T4CS1           1       /* Timer4 Clock Select bit 1 */
#define    BM_T4CS1	(uint8_t)(1U<<T4CS1)
#define    T4PS0           2       /* Timer4 Prescaler Select bit 0 */
#define    BM_T4PS0	(uint8_t)(1U<<T4PS0)
#define    T4PS1           3       /* Timer4 Prescaler Select bit 1 */
#define    BM_T4PS1	(uint8_t)(1U<<T4PS1)
#define    T4PS2           4       /* Timer4 Prescaler Select bit 2 */
#define    BM_T4PS2	(uint8_t)(1U<<T4PS2)

/* T4MRB - Timer4 mode Register */
#define    T4SCE           1       /* Timer4 Software Capture Enable */
#define    BM_T4SCE	(uint8_t)(1U<<T4SCE)
#define    T4CNC           2       /* Timer4 input Capture Noise Canceller */
#define    BM_T4CNC	(uint8_t)(1U<<T4CNC)
#define    T4CE0           3       /* Timer4 Capture Edge select bit 0 */
#define    BM_T4CE0	(uint8_t)(1U<<T4CE0)
#define    T4CE1           4       /* Timer4 Capture Edge select bit 1 */
#define    BM_T4CE1	(uint8_t)(1U<<T4CE1)
#define    T4ICS0          5       /* Timer4 Input Capture Select bit 0 */
#define    BM_T4ICS0	(uint8_t)(1U<<T4ICS0)
#define    T4ICS1          6       /* Timer4 Input Capture Select bit 1 */
#define    BM_T4ICS1	(uint8_t)(1U<<T4ICS1)
#define    T4ICS2          7       /* Timer4 Input Capture Select bit 2 */
#define    BM_T4ICS2	(uint8_t)(1U<<T4ICS2)


/* ***** TIMER5 *********************** */
/* GTCCR - General Timer/Counter Control Register */
#define    PSR10           0       /* PreScaler Reset */
#define    BM_PSR10	(uint8_t)(1U<<PSR10)
#define    TSM             7       /* Timer/Counter Synchronization Mode */
#define    BM_TSM	(uint8_t)(1U<<TSM)

/* T5CCR - Timer5 Control Register */
#define    T5CS0           0       /* Timer5 Clock Select bit 0 */
#define    BM_T5CS0	(uint8_t)(1U<<T5CS0)
#define    T5CS1           1       /* Timer5 Clock Select bit 1 */
#define    BM_T5CS1	(uint8_t)(1U<<T5CS1)
#define    T5CS2           2       /* Timer5 Clock Select bit 2 */
#define    BM_T5CS2	(uint8_t)(1U<<T5CS2)
#define    T5CTC           3       /* Clear Timer5 on Compare Match */
#define    BM_T5CTC	(uint8_t)(1U<<T5CTC)

/* T5CNTH - Timer5 Counter High byte */
#define    T5CNT8          0       /* Timer5 Counter bit 8 */
#define    BM_T5CNT8	(uint8_t)(1U<<T5CNT8)
#define    T5CNT9          1       /* Timer5 Counter bit 9 */
#define    BM_T5CNT9	(uint8_t)(1U<<T5CNT9)
#define    T5CNT10         2       /* Timer5 Counter bit 10 */
#define    BM_T5CNT10	(uint8_t)(1U<<T5CNT10)
#define    T5CNT11         3       /* Timer5 Counter bit 11 */
#define    BM_T5CNT11	(uint8_t)(1U<<T5CNT11)
#define    T5CNT12         4       /* Timer5 Counter bit 12 */
#define    BM_T5CNT12	(uint8_t)(1U<<T5CNT12)
#define    T5CNT13         5       /* Timer5 Counter bit 13 */
#define    BM_T5CNT13	(uint8_t)(1U<<T5CNT13)
#define    T5CNT14         6       /* Timer5 Counter bit 14 */
#define    BM_T5CNT14	(uint8_t)(1U<<T5CNT14)
#define    T5CNT15         7       /* Timer5 Counter bit 15 */
#define    BM_T5CNT15	(uint8_t)(1U<<T5CNT15)

/* T5CNTL - Timer5 Counter Low byte */
#define    T5CNT0          0       /* Timer5 Counter bit 0 */
#define    BM_T5CNT0	(uint8_t)(1U<<T5CNT0)
#define    T5CNT1          1       /* Timer5 Counter bit 1 */
#define    BM_T5CNT1	(uint8_t)(1U<<T5CNT1)
#define    T5CNT2          2       /* Timer5 Counter bit 2 */
#define    BM_T5CNT2	(uint8_t)(1U<<T5CNT2)
#define    T5CNT3          3       /* Timer5 Counter bit 3 */
#define    BM_T5CNT3	(uint8_t)(1U<<T5CNT3)
#define    T5CNT4          4       /* Timer5 Counter bit 4 */
#define    BM_T5CNT4	(uint8_t)(1U<<T5CNT4)
#define    T5CNT5          5       /* Timer5 Counter bit 5 */
#define    BM_T5CNT5	(uint8_t)(1U<<T5CNT5)
#define    T5CNT6          6       /* Timer5 Counter bit 6 */
#define    BM_T5CNT6	(uint8_t)(1U<<T5CNT6)
#define    T5CNT7          7       /* Timer5 Counter bit 7 */
#define    BM_T5CNT7	(uint8_t)(1U<<T5CNT7)

/* T5IFR - Timer5 Interrupt Flag Register */
#define    T5OFF           0       /* Timer5 Output Overflow Flag */
#define    BM_T5OFF	(uint8_t)(1U<<T5OFF)
#define    T5COF           1       /* Timer5 Output Compare Output Match Flag */
#define    BM_T5COF	(uint8_t)(1U<<T5COF)

/* T5IMR - Timer5 Interrupt Mask Register */
#define    T5OIM           0       /* Timer5 Output Overflow Interrupt Mask */
#define    BM_T5OIM	(uint8_t)(1U<<T5OIM)
#define    T5CIM           1       /* Timer5 Output Compare Interrupt Mask */
#define    BM_T5CIM	(uint8_t)(1U<<T5CIM)

/* T5OCRH - Timer5 Output Compare Register High byte */
#define    T5OCR8          0       /* Timer5 Output Compare Register bit 8 */
#define    BM_T5OCR8	(uint8_t)(1U<<T5OCR8)
#define    T5OCR9          1       /* Timer5 Output Compare Register bit 9 */
#define    BM_T5OCR9	(uint8_t)(1U<<T5OCR9)
#define    T5OCR10         2       /* Timer5 Output Compare Register bit 10 */
#define    BM_T5OCR10	(uint8_t)(1U<<T5OCR10)
#define    T5OCR11         3       /* Timer5 Output Compare Register bit 11 */
#define    BM_T5OCR11	(uint8_t)(1U<<T5OCR11)
#define    T5OCR12         4       /* Timer5 Output Compare Register bit 12 */
#define    BM_T5OCR12	(uint8_t)(1U<<T5OCR12)
#define    T5OCR13         5       /* Timer5 Output Compare Register bit 13 */
#define    BM_T5OCR13	(uint8_t)(1U<<T5OCR13)
#define    T5OCR14         6       /* Timer5 Output Compare Register bit 14 */
#define    BM_T5OCR14	(uint8_t)(1U<<T5OCR14)
#define    T5OCR15         7       /* Timer5 Output Compare Register bit 15 */
#define    BM_T5OCR15	(uint8_t)(1U<<T5OCR15)

/* T5OCRL - Timer5 Output Compare Register Low byte */
#define    T5OCRL0         0       /* Timer5 Output Compare Register Low byte bit 0 */
#define    BM_T5OCRL0	(uint8_t)(1U<<T5OCRL0)
#define    T5OCRL1         1       /* Timer5 Output Compare Register Low byte bit 1 */
#define    BM_T5OCRL1	(uint8_t)(1U<<T5OCRL1)
#define    T5OCRL2         2       /* Timer5 Output Compare Register Low byte bit 2 */
#define    BM_T5OCRL2	(uint8_t)(1U<<T5OCRL2)
#define    T5OCRL3         3       /* Timer5 Output Compare Register Low byte bit 3 */
#define    BM_T5OCRL3	(uint8_t)(1U<<T5OCRL3)
#define    T5OCRL4         4       /* Timer5 Output Compare Register Low byte bit 4 */
#define    BM_T5OCRL4	(uint8_t)(1U<<T5OCRL4)
#define    T5OCRL5         5       /* Timer5 Output Compare Register Low byte bit 5 */
#define    BM_T5OCRL5	(uint8_t)(1U<<T5OCRL5)
#define    T5OCRL6         6       /* Timer5 Output Compare Register Low byte bit 6 */
#define    BM_T5OCRL6	(uint8_t)(1U<<T5OCRL6)
#define    T5OCRL7         7       /* Timer5 Output Compare Register Low byte bit 7 */
#define    BM_T5OCRL7	(uint8_t)(1U<<T5OCRL7)

/* T5TEMP - Timer5 Temp Register */
#define    T5TEMP0         0       /* Timer5 Temp bit 0 */
#define    BM_T5TEMP0	(uint8_t)(1U<<T5TEMP0)
#define    T5TEMP1         1       /* Timer5 Temp bit 1 */
#define    BM_T5TEMP1	(uint8_t)(1U<<T5TEMP1)
#define    T5TEMP2         2       /* Timer5 Temp bit 2 */
#define    BM_T5TEMP2	(uint8_t)(1U<<T5TEMP2)
#define    T5TEMP3         3       /* Timer5 Temp bit 3 */
#define    BM_T5TEMP3	(uint8_t)(1U<<T5TEMP3)
#define    T5TEMP4         4       /* Timer5 Temp bit 4 */
#define    BM_T5TEMP4	(uint8_t)(1U<<T5TEMP4)
#define    T5TEMP5         5       /* Timer5 Temp bit 5 */
#define    BM_T5TEMP5	(uint8_t)(1U<<T5TEMP5)
#define    T5TEMP6         6       /* Timer5 Temp bit 6 */
#define    BM_T5TEMP6	(uint8_t)(1U<<T5TEMP6)
#define    T5TEMP7         7       /* Timer5 Temp bit 7 */
#define    BM_T5TEMP7	(uint8_t)(1U<<T5TEMP7)


/* ***** TMO ************************** */
/* TMOCR - Timer Modulator Output Control Register */
#define    TO1PIS0         0       /* Timer modulator Output 1 Port Interface Select bit 0 */
#define    BM_TO1PIS0	(uint8_t)(1U<<TO1PIS0)
#define    TO1PIS1         1       /* Timer modulator Output 1 Port Interface Select bit 1 */
#define    BM_TO1PIS1	(uint8_t)(1U<<TO1PIS1)
#define    TO2PIS0         2       /* Timer modulator Output 2 Port Interface Select bit 0 */
#define    BM_TO2PIS0	(uint8_t)(1U<<TO2PIS0)
#define    TO2PIS1         3       /* Timer modulator Output 2 Port Interface Select bit 1 */
#define    BM_TO2PIS1	(uint8_t)(1U<<TO2PIS1)
#define    TO3PIS0         4       /* Timer modulator Output 3 Port Interface Select bit 0 */
#define    BM_TO3PIS0	(uint8_t)(1U<<TO3PIS0)
#define    TO3PIS1         5       /* Timer modulator Output 3 Port Interface Select bit 1 */
#define    BM_TO3PIS1	(uint8_t)(1U<<TO3PIS1)
#define    TO4PIS0         6       /* Timer modulator Output 4 Port Interface Select bit 0 */
#define    BM_TO4PIS0	(uint8_t)(1U<<TO4PIS0)
#define    TO4PIS1         7       /* Timer modulator Output 4 Port Interface Select bit 1 */
#define    BM_TO4PIS1	(uint8_t)(1U<<TO4PIS1)


/* ***** TPLF_CAL ********************* */
/* SRCCAL - Slow RC oscillator calibration Register */
#define    SRCCAL0         0       /* Slow RC oscillator calibration bit 0 */
#define    BM_SRCCAL0	(uint8_t)(1U<<SRCCAL0)
#define    SRCCAL1         1       /* Slow RC oscillator calibration bit 1 */
#define    BM_SRCCAL1	(uint8_t)(1U<<SRCCAL1)
#define    SRCCAL2         2       /* Slow RC oscillator calibration bit 2 */
#define    BM_SRCCAL2	(uint8_t)(1U<<SRCCAL2)
#define    SRCCAL3         3       /* Slow RC oscillator calibration bit 3 */
#define    BM_SRCCAL3	(uint8_t)(1U<<SRCCAL3)
#define    SRCCAL4         4       /* Slow RC oscillator calibration bit 4 */
#define    BM_SRCCAL4	(uint8_t)(1U<<SRCCAL4)
#define    SRCCAL5         5       /* Slow RC oscillator calibration bit 5 */
#define    BM_SRCCAL5	(uint8_t)(1U<<SRCCAL5)
#define    SRCCAL6         6       /* Slow RC oscillator calibration bit 6 */
#define    BM_SRCCAL6	(uint8_t)(1U<<SRCCAL6)
#define    SRCCAL7         7       /* Slow RC oscillator calibration bit 7 */
#define    BM_SRCCAL7	(uint8_t)(1U<<SRCCAL7)

/* SRCTCAL - SRC oscillator Temperature Compensation register */
#define    SRCTC0          0       /* SRC Oscillator Temperature Compensation bit 0 */
#define    BM_SRCTC0	(uint8_t)(1U<<SRCTC0)
#define    SRCTC1          1       /* SRC Oscillator Temperature Compensation bit 1 */
#define    BM_SRCTC1	(uint8_t)(1U<<SRCTC1)
#define    SRCTC2          2       /* SRC Oscillator Temperature Compensation bit 2 */
#define    BM_SRCTC2	(uint8_t)(1U<<SRCTC2)
#define    SRCS0           3       /* SRC Oscillator Select delta current bit 0 */
#define    BM_SRCS0	(uint8_t)(1U<<SRCS0)
#define    SRCS1           4       /* SRC Oscillator Select delta current bit 1 */
#define    BM_SRCS1	(uint8_t)(1U<<SRCS1)


/* ***** TRANSPONDER ****************** */
/* TPCR1 - Transponder Control Register 1 */
#define    TPCR1_R0        0       /*  */
#define    BM_TPCR1_R0	(uint8_t)(1U<<TPCR1_R0)
#define    TPCR1_R1        1       /*  */
#define    BM_TPCR1_R1	(uint8_t)(1U<<TPCR1_R1)
#define    TPQPLM          2       /* Transponder Quadrature Pulse Length Operation Modulation */
#define    BM_TPQPLM	(uint8_t)(1U<<TPQPLM)
#define    TPCR1_R3        3       /*  */
#define    BM_TPCR1_R3	(uint8_t)(1U<<TPCR1_R3)
#define    TPBR            4       /* Transponder Bit Rate */
#define    BM_TPBR	(uint8_t)(1U<<TPBR)
#define    TPDFCP0         5       /* Transponder Decoder Field Clock Prescaler value bit 0 */
#define    BM_TPDFCP0	(uint8_t)(1U<<TPDFCP0)
#define    TPDFCP1         6       /* Transponder Decoder Field Clock Prescaler value bit 1 */
#define    BM_TPDFCP1	(uint8_t)(1U<<TPDFCP1)
#define    TPMODE          7       /* Transponder Mode */
#define    BM_TPMODE	(uint8_t)(1U<<TPMODE)

/* TPCR2 - Transponder Control 2 Register */
#define    TPMA            0       /* Transponder Mode Acknowledge */
#define    BM_TPMA	(uint8_t)(1U<<TPMA)
#define    TPMOD           1       /* Transponder Modulation */
#define    BM_TPMOD	(uint8_t)(1U<<TPMOD)
#define    TPPSD           2       /* Transponder Power Switch Disable */
#define    BM_TPPSD	(uint8_t)(1U<<TPPSD)
#define    TPD             3       /* Transponder Disable */
#define    BM_TPD	(uint8_t)(1U<<TPD)
#define    TPNFTO          4       /* Transponder No Field Timeout enable */
#define    BM_TPNFTO	(uint8_t)(1U<<TPNFTO)
#define    TPWDLV0         5       /* Transponder watch dog value for No Field timeout (with SRC clk) bit 0 */
#define    BM_TPWDLV0	(uint8_t)(1U<<TPWDLV0)
#define    TPWDLV1         6       /* Transponder watch dog value for No Field timeout (with SRC clk) bit 1 */
#define    BM_TPWDLV1	(uint8_t)(1U<<TPWDLV1)
#define    TPCR2_R7        7       /*  */
#define    BM_TPCR2_R7	(uint8_t)(1U<<TPCR2_R7)

/* TPCR3 - Transponder Control Register 3 */
#define    TPTD            0       /* Transponder Transmit Data */
#define    BM_TPTD	(uint8_t)(1U<<TPTD)
#define    TPRD            1       /* Transponder Receive Data */
#define    BM_TPRD	(uint8_t)(1U<<TPRD)
#define    TPTLIW          2       /* Transponder Transmit Listen Window */
#define    BM_TPTLIW	(uint8_t)(1U<<TPTLIW)
#define    TPCR3_R3        3       /*  */
#define    BM_TPCR3_R3	(uint8_t)(1U<<TPCR3_R3)
#define    TPCR3_R4        4       /*  */
#define    BM_TPCR3_R4	(uint8_t)(1U<<TPCR3_R4)
#define    TPRCD           5       /* Transponder Reception Continuous Damping */
#define    BM_TPRCD	(uint8_t)(1U<<TPRCD)

/* TPCR4 - Transponder Control Register 4 */
#define    TPBCCS0         0       /* Transponder Battery Charge Current Select bit 0 */
#define    BM_TPBCCS0	(uint8_t)(1U<<TPBCCS0)
#define    TPBCCS1         1       /* Transponder Battery Charge Current Select bit 1 */
#define    BM_TPBCCS1	(uint8_t)(1U<<TPBCCS1)
#define    TPBCCS2         2       /* Transponder Battery Charge Current Select bit 2 */
#define    BM_TPBCCS2	(uint8_t)(1U<<TPBCCS2)
#define    TPBCCS3         3       /* Transponder Battery Charge Current Select bit 3 */
#define    BM_TPBCCS3	(uint8_t)(1U<<TPBCCS3)
#define    TPBCM           4       /* Transponder Battery Charge Mode */
#define    BM_TPBCM	(uint8_t)(1U<<TPBCM)

/* TPCR5 - Transponder Control Register 5 */
#define    TPMUD0          0       /* Transponder Modulation Undamped Level Select bit 0 */
#define    BM_TPMUD0	(uint8_t)(1U<<TPMUD0)
#define    TPMUD1          1       /* Transponder Modulation Undamped Level Select bit 1 */
#define    BM_TPMUD1	(uint8_t)(1U<<TPMUD1)
#define    TPMUD2          2       /* Transponder Modulation Undamped Level Select bit 2 */
#define    BM_TPMUD2	(uint8_t)(1U<<TPMUD2)
#define    TPMD0           4       /* Transponder Modulation Damping Select bit 0 */
#define    BM_TPMD0	(uint8_t)(1U<<TPMD0)
#define    TPMD1           5       /* Transponder Modulation Damping Select bit 1 */
#define    BM_TPMD1	(uint8_t)(1U<<TPMD1)
#define    TPMD2           6       /* Transponder Modulation Damping Select bit 2 */
#define    BM_TPMD2	(uint8_t)(1U<<TPMD2)

/* TPECMR - Transponder Encoder Mode Register */
#define    TPECM10         0       /* Transponder Encoder Mode 1 bit 0 */
#define    BM_TPECM10	(uint8_t)(1U<<TPECM10)
#define    TPECM11         1       /* Transponder Encoder Mode 1 bit 1 */
#define    BM_TPECM11	(uint8_t)(1U<<TPECM11)
#define    TPECM20         2       /* Transponder Encoder Mode 2 bit 0 */
#define    BM_TPECM20	(uint8_t)(1U<<TPECM20)
#define    TPECM21         3       /* Transponder Encoder Mode 2 bit 1 */
#define    BM_TPECM21	(uint8_t)(1U<<TPECM21)
#define    TPECM30         4       /* Transponder Encoder Mode 3 bit 0 */
#define    BM_TPECM30	(uint8_t)(1U<<TPECM30)
#define    TPECM31         5       /* Transponder Encoder Mode 3 bit 1 */
#define    BM_TPECM31	(uint8_t)(1U<<TPECM31)
#define    TPECM40         6       /* Transponder Encoder Mode 4 bit 0 */
#define    BM_TPECM40	(uint8_t)(1U<<TPECM40)
#define    TPECM41         7       /* Transponder Encoder Mode 4 bit 1 */
#define    BM_TPECM41	(uint8_t)(1U<<TPECM41)

/* TPFR - Transponder Flag Register */
#define    TPF             0       /* Transponder Flag */
#define    BM_TPF	(uint8_t)(1U<<TPF)
#define    TPFTF           1       /* Transponder Field Timeout Flag */
#define    BM_TPFTF	(uint8_t)(1U<<TPFTF)
#define    TPNFTF          2       /* Transponder No Field Timeout Flag */
#define    BM_TPNFTF	(uint8_t)(1U<<TPNFTF)
#define    TPBERF          3       /* Transponder Bit Error Flag */
#define    BM_TPBERF	(uint8_t)(1U<<TPBERF)

/* TPIMR - Transponder Interrupt Mask Register */
#define    TPIM            0       /* Transponder Interrupt Mask */
#define    BM_TPIM	(uint8_t)(1U<<TPIM)
#define    TPFTIM          1       /* Transponder Field Timeout Interrupt Mask */
#define    BM_TPFTIM	(uint8_t)(1U<<TPFTIM)
#define    TPNFTIM         2       /* Transponder No Field Timeout Interrupt Mask */
#define    BM_TPNFTIM	(uint8_t)(1U<<TPNFTIM)
#define    TPBERIM         3       /* Transponder Bit Error Interrupt Mask */
#define    BM_TPBERIM	(uint8_t)(1U<<TPBERIM)

/* TPSR - Transponder Status Register */
#define    TPA             0       /* Transponder Active */
#define    BM_TPA	(uint8_t)(1U<<TPA)
#define    TPGAP           1       /* Transponder GAP Signal */
#define    BM_TPGAP	(uint8_t)(1U<<TPGAP)
#define    TPPSW           2       /* Transponder Power Switch */
#define    BM_TPPSW	(uint8_t)(1U<<TPPSW)
#define    TPBCOK          3       /* Transponder Battery Charge OK */
#define    BM_TPBCOK	(uint8_t)(1U<<TPBCOK)


/* ***** TWI ************************** */
/* TWAMR - TWI Address Mask Register */
#define    TWAM0           1       /* TWI Address Mask bit 0 */
#define    BM_TWAM0	(uint8_t)(1U<<TWAM0)
#define    TWAM1           2       /* TWI Address Mask bit 1 */
#define    BM_TWAM1	(uint8_t)(1U<<TWAM1)
#define    TWAM2           3       /* TWI Address Mask bit 2 */
#define    BM_TWAM2	(uint8_t)(1U<<TWAM2)
#define    TWAM3           4       /* TWI Address Mask bit 3 */
#define    BM_TWAM3	(uint8_t)(1U<<TWAM3)
#define    TWAM4           5       /* TWI Address Mask bit 4 */
#define    BM_TWAM4	(uint8_t)(1U<<TWAM4)
#define    TWAM5           6       /* TWI Address Mask bit 5 */
#define    BM_TWAM5	(uint8_t)(1U<<TWAM5)
#define    TWAM6           7       /* TWI Address Mask bit 6 */
#define    BM_TWAM6	(uint8_t)(1U<<TWAM6)

/* TWAR - TWI (Slave) Address Register */
#define    TWGCE           0       /* TWI General Call Recognition Enable */
#define    BM_TWGCE	(uint8_t)(1U<<TWGCE)
#define    TWA0            1       /* TWI (Slave) Address bit 0 */
#define    BM_TWA0	(uint8_t)(1U<<TWA0)
#define    TWA1            2       /* TWI (Slave) Address bit 1 */
#define    BM_TWA1	(uint8_t)(1U<<TWA1)
#define    TWA2            3       /* TWI (Slave) Address bit 2 */
#define    BM_TWA2	(uint8_t)(1U<<TWA2)
#define    TWA3            4       /* TWI (Slave) Address bit 3 */
#define    BM_TWA3	(uint8_t)(1U<<TWA3)
#define    TWA4            5       /* TWI (Slave) Address bit 4 */
#define    BM_TWA4	(uint8_t)(1U<<TWA4)
#define    TWA5            6       /* TWI (Slave) Address bit 5 */
#define    BM_TWA5	(uint8_t)(1U<<TWA5)
#define    TWA6            7       /* TWI (Slave) Address bit 6 */
#define    BM_TWA6	(uint8_t)(1U<<TWA6)

/* TWBR - TWI Bit Rate Register */
#define    TWBR0           0       /* TWI Bit Rate bit 0 */
#define    BM_TWBR0	(uint8_t)(1U<<TWBR0)
#define    TWBR1           1       /* TWI Bit Rate bit 1 */
#define    BM_TWBR1	(uint8_t)(1U<<TWBR1)
#define    TWBR2           2       /* TWI Bit Rate bit 2 */
#define    BM_TWBR2	(uint8_t)(1U<<TWBR2)
#define    TWBR3           3       /* TWI Bit Rate bit 3 */
#define    BM_TWBR3	(uint8_t)(1U<<TWBR3)
#define    TWBR4           4       /* TWI Bit Rate bit 4 */
#define    BM_TWBR4	(uint8_t)(1U<<TWBR4)
#define    TWBR5           5       /* TWI Bit Rate bit 5 */
#define    BM_TWBR5	(uint8_t)(1U<<TWBR5)
#define    TWBR6           6       /* TWI Bit Rate bit 6 */
#define    BM_TWBR6	(uint8_t)(1U<<TWBR6)
#define    TWBR7           7       /* TWI Bit Rate bit 7 */
#define    BM_TWBR7	(uint8_t)(1U<<TWBR7)

/* TWCR - TWI Control Register */
#define    TWIE            0       /* TWI Interrupt Enable */
#define    BM_TWIE	(uint8_t)(1U<<TWIE)
#define    TWEN            2       /* TWI Enable */
#define    BM_TWEN	(uint8_t)(1U<<TWEN)
#define    TWWC            3       /* TWI Write Collision Flag */
#define    BM_TWWC	(uint8_t)(1U<<TWWC)
#define    TWSTO           4       /* TWI STOP Condition */
#define    BM_TWSTO	(uint8_t)(1U<<TWSTO)
#define    TWSTA           5       /* TWI START Condition */
#define    BM_TWSTA	(uint8_t)(1U<<TWSTA)
#define    TWEA            6       /* TWI Enable Acknowledge */
#define    BM_TWEA	(uint8_t)(1U<<TWEA)
#define    TWINT           7       /* TWI Interrupt Flag */
#define    BM_TWINT	(uint8_t)(1U<<TWINT)

/* TWDR - TWI Data Register */
#define    TWD0            0       /* TWI Data bit 0 */
#define    BM_TWD0	(uint8_t)(1U<<TWD0)
#define    TWD1            1       /* TWI Data bit 1 */
#define    BM_TWD1	(uint8_t)(1U<<TWD1)
#define    TWD2            2       /* TWI Data bit 2 */
#define    BM_TWD2	(uint8_t)(1U<<TWD2)
#define    TWD3            3       /* TWI Data bit 3 */
#define    BM_TWD3	(uint8_t)(1U<<TWD3)
#define    TWD4            4       /* TWI Data bit 4 */
#define    BM_TWD4	(uint8_t)(1U<<TWD4)
#define    TWD5            5       /* TWI Data bit 5 */
#define    BM_TWD5	(uint8_t)(1U<<TWD5)
#define    TWD6            6       /* TWI Data bit 6 */
#define    BM_TWD6	(uint8_t)(1U<<TWD6)
#define    TWD7            7       /* TWI Data bit 7 */
#define    BM_TWD7	(uint8_t)(1U<<TWD7)

/* TWSR - TWI Status Register */
#define    TWPS0           0       /* TWI Prescaler bit 0 */
#define    BM_TWPS0	(uint8_t)(1U<<TWPS0)
#define    TWPS1           1       /* TWI Prescaler bit 1 */
#define    BM_TWPS1	(uint8_t)(1U<<TWPS1)
#define    TWS0            3       /* TWI Status bit 0 */
#define    BM_TWS0	(uint8_t)(1U<<TWS0)
#define    TWS1            4       /* TWI Status bit 1 */
#define    BM_TWS1	(uint8_t)(1U<<TWS1)
#define    TWS2            5       /* TWI Status bit 2 */
#define    BM_TWS2	(uint8_t)(1U<<TWS2)
#define    TWS3            6       /* TWI Status bit 3 */
#define    BM_TWS3	(uint8_t)(1U<<TWS3)
#define    TWS4            7       /* TWI Status bit 4 */
#define    BM_TWS4	(uint8_t)(1U<<TWS4)


/* ***** TXDSP ************************ */
/* BBTE2 - Base Band Test Enable 2 */
#define    TDEPO           0       /* Tx DSP External Pin Output */
#define    BM_TDEPO	(uint8_t)(1U<<TDEPO)
#define    DITDIS          1       /* Dither Disable on Sigma Delta Modulator */
#define    BM_DITDIS	(uint8_t)(1U<<DITDIS)

/* FFREQ1H - Fractional Frequency 1 Setting, High Byte */
#define    FFREQ1H0        0       /* Fractional Frequency 1 High Byte bit 0 */
#define    BM_FFREQ1H0	(uint8_t)(1U<<FFREQ1H0)
#define    FFREQ1H1        1       /* Fractional Frequency 1 High Byte bit 1 */
#define    BM_FFREQ1H1	(uint8_t)(1U<<FFREQ1H1)

/* FFREQ1L - Fractional Frequency 1 Setting, Low Byte */
#define    FFREQ1L0        0       /* Fractional Frequency 1 Low Byte bit 0 */
#define    BM_FFREQ1L0	(uint8_t)(1U<<FFREQ1L0)
#define    FFREQ1L1        1       /* Fractional Frequency 1 Low Byte bit 1 */
#define    BM_FFREQ1L1	(uint8_t)(1U<<FFREQ1L1)
#define    FFREQ1L2        2       /* Fractional Frequency 1 Low Byte bit 2 */
#define    BM_FFREQ1L2	(uint8_t)(1U<<FFREQ1L2)
#define    FFREQ1L3        3       /* Fractional Frequency 1 Low Byte bit 3 */
#define    BM_FFREQ1L3	(uint8_t)(1U<<FFREQ1L3)
#define    FFREQ1L4        4       /* Fractional Frequency 1 Low Byte bit 4 */
#define    BM_FFREQ1L4	(uint8_t)(1U<<FFREQ1L4)
#define    FFREQ1L5        5       /* Fractional Frequency 1 Low Byte bit 5 */
#define    BM_FFREQ1L5	(uint8_t)(1U<<FFREQ1L5)
#define    FFREQ1L6        6       /* Fractional Frequency 1 Low Byte bit 6 */
#define    BM_FFREQ1L6	(uint8_t)(1U<<FFREQ1L6)
#define    FFREQ1L7        7       /* Fractional Frequency 1 Low Byte bit 7 */
#define    BM_FFREQ1L7	(uint8_t)(1U<<FFREQ1L7)

/* FFREQ1M - Fractional Frequency 1 Setting, Middle Byte */
#define    FFREQ1M0        0       /* Fractional Frequency 1 Middle Byte bit 0 */
#define    BM_FFREQ1M0	(uint8_t)(1U<<FFREQ1M0)
#define    FFREQ1M1        1       /* Fractional Frequency 1 Middle Byte bit 1 */
#define    BM_FFREQ1M1	(uint8_t)(1U<<FFREQ1M1)
#define    FFREQ1M2        2       /* Fractional Frequency 1 Middle Byte bit 2 */
#define    BM_FFREQ1M2	(uint8_t)(1U<<FFREQ1M2)
#define    FFREQ1M3        3       /* Fractional Frequency 1 Middle Byte bit 3 */
#define    BM_FFREQ1M3	(uint8_t)(1U<<FFREQ1M3)
#define    FFREQ1M4        4       /* Fractional Frequency 1 Middle Byte bit 4 */
#define    BM_FFREQ1M4	(uint8_t)(1U<<FFREQ1M4)
#define    FFREQ1M5        5       /* Fractional Frequency 1 Middle Byte bit 5 */
#define    BM_FFREQ1M5	(uint8_t)(1U<<FFREQ1M5)
#define    FFREQ1M6        6       /* Fractional Frequency 1 Middle Byte bit 6 */
#define    BM_FFREQ1M6	(uint8_t)(1U<<FFREQ1M6)
#define    FFREQ1M7        7       /* Fractional Frequency 1 Middle Byte bit 7 */
#define    BM_FFREQ1M7	(uint8_t)(1U<<FFREQ1M7)

/* FFREQ2H - Fractional Frequency 2 Setting, High Byte */
#define    FFREQ2H0        0       /* Fractional Frequency 2 High Byte bit 0 */
#define    BM_FFREQ2H0	(uint8_t)(1U<<FFREQ2H0)
#define    FFREQ2H1        1       /* Fractional Frequency 2 High Byte bit 1 */
#define    BM_FFREQ2H1	(uint8_t)(1U<<FFREQ2H1)

/* FFREQ2L - Fractional Frequency 2 Setting, Low Byte */
#define    FFREQ2L0        0       /* Fractional Frequency 2 Low Byte bit 0 */
#define    BM_FFREQ2L0	(uint8_t)(1U<<FFREQ2L0)
#define    FFREQ2L1        1       /* Fractional Frequency 2 Low Byte bit 1 */
#define    BM_FFREQ2L1	(uint8_t)(1U<<FFREQ2L1)
#define    FFREQ2L2        2       /* Fractional Frequency 2 Low Byte bit 2 */
#define    BM_FFREQ2L2	(uint8_t)(1U<<FFREQ2L2)
#define    FFREQ2L3        3       /* Fractional Frequency 2 Low Byte bit 3 */
#define    BM_FFREQ2L3	(uint8_t)(1U<<FFREQ2L3)
#define    FFREQ2L4        4       /* Fractional Frequency 2 Low Byte bit 4 */
#define    BM_FFREQ2L4	(uint8_t)(1U<<FFREQ2L4)
#define    FFREQ2L5        5       /* Fractional Frequency 2 Low Byte bit 5 */
#define    BM_FFREQ2L5	(uint8_t)(1U<<FFREQ2L5)
#define    FFREQ2L6        6       /* Fractional Frequency 2 Low Byte bit 6 */
#define    BM_FFREQ2L6	(uint8_t)(1U<<FFREQ2L6)
#define    FFREQ2L7        7       /* Fractional Frequency 2 Low Byte bit 7 */
#define    BM_FFREQ2L7	(uint8_t)(1U<<FFREQ2L7)

/* FFREQ2M - Fractional Frequency 2 Setting, Middle Byte */
#define    FFREQ2M0        0       /* Fractional Frequency 2 Middle Byte bit 0 */
#define    BM_FFREQ2M0	(uint8_t)(1U<<FFREQ2M0)
#define    FFREQ2M1        1       /* Fractional Frequency 2 Middle Byte bit 1 */
#define    BM_FFREQ2M1	(uint8_t)(1U<<FFREQ2M1)
#define    FFREQ2M2        2       /* Fractional Frequency 2 Middle Byte bit 2 */
#define    BM_FFREQ2M2	(uint8_t)(1U<<FFREQ2M2)
#define    FFREQ2M3        3       /* Fractional Frequency 2 Middle Byte bit 3 */
#define    BM_FFREQ2M3	(uint8_t)(1U<<FFREQ2M3)
#define    FFREQ2M4        4       /* Fractional Frequency 2 Middle Byte bit 4 */
#define    BM_FFREQ2M4	(uint8_t)(1U<<FFREQ2M4)
#define    FFREQ2M5        5       /* Fractional Frequency 2 Middle Byte bit 5 */
#define    BM_FFREQ2M5	(uint8_t)(1U<<FFREQ2M5)
#define    FFREQ2M6        6       /* Fractional Frequency 2 Middle Byte bit 6 */
#define    BM_FFREQ2M6	(uint8_t)(1U<<FFREQ2M6)
#define    FFREQ2M7        7       /* Fractional Frequency 2 Middle Byte bit 7 */
#define    BM_FFREQ2M7	(uint8_t)(1U<<FFREQ2M7)

/* FSCR - Frequency Synthesizer Control Register */
#define    TXMOD           0       /* Tx Modulation */
#define    BM_TXMOD	(uint8_t)(1U<<TXMOD)
#define    SFM             1       /* Select FSK Modulation */
#define    BM_SFM	(uint8_t)(1U<<SFM)
#define    TXMS0           2       /* Tx Modulation Source bit 0 */
#define    BM_TXMS0	(uint8_t)(1U<<TXMS0)
#define    TXMS1           3       /* Tx Modulation Source bit 1 */
#define    BM_TXMS1	(uint8_t)(1U<<TXMS1)
#define    PAOER           4       /* Power Amplifier Output Enable Register */
#define    BM_PAOER	(uint8_t)(1U<<PAOER)
#define    PAON            7       /* Power Amplifier Output is On */
#define    BM_PAON	(uint8_t)(1U<<PAON)

/* FSEN - Frequency Synthesizer Enable register */
#define    SDPU            0       /* Sigma-delta Modulator Power Up */
#define    BM_SDPU	(uint8_t)(1U<<SDPU)
#define    SDEN            1       /* Sigma-delta Modulator Enable */
#define    BM_SDEN	(uint8_t)(1U<<SDEN)
#define    GAEN            2       /* Gauss Filtering Enable */
#define    BM_GAEN	(uint8_t)(1U<<GAEN)
#define    PEEN            3       /* Pre-emphasis Filtering Enable */
#define    BM_PEEN	(uint8_t)(1U<<PEEN)
#define    ASEN            4       /* ASK Shaping Enable */
#define    BM_ASEN	(uint8_t)(1U<<ASEN)
#define    ANTT            5       /* Antenna Tuning Active */
#define    BM_ANTT	(uint8_t)(1U<<ANTT)

/* FSFCR - Frequency Synthesizer Filter Control Register */
#define    BTSEL0          0       /* Gauss Filter BT Selection bit 0 */
#define    BM_BTSEL0	(uint8_t)(1U<<BTSEL0)
#define    BTSEL1          1       /* Gauss Filter BT Selection bit 1 */
#define    BM_BTSEL1	(uint8_t)(1U<<BTSEL1)
#define    ASDIV0          4       /* ASK Shaping Divider bit 0 */
#define    BM_ASDIV0	(uint8_t)(1U<<ASDIV0)
#define    ASDIV1          5       /* ASK Shaping Divider bit 1 */
#define    BM_ASDIV1	(uint8_t)(1U<<ASDIV1)
#define    ASDIV2          6       /* ASK Shaping Divider bit 2 */
#define    BM_ASDIV2	(uint8_t)(1U<<ASDIV2)
#define    ASDIV3          7       /* ASK Shaping Divider bit 3 */
#define    BM_ASDIV3	(uint8_t)(1U<<ASDIV3)

/* GACDIVH - Gauss Clock Divider, High Byte */
#define    GACDIV8         0       /* Gauss Clock Divider bit 8 */
#define    BM_GACDIV8	(uint8_t)(1U<<GACDIV8)
#define    GACDIV9         1       /* Gauss Clock Divider bit 9 */
#define    BM_GACDIV9	(uint8_t)(1U<<GACDIV9)
#define    GACDIV10        2       /* Gauss Clock Divider bit 10 */
#define    BM_GACDIV10	(uint8_t)(1U<<GACDIV10)
#define    GACDIV11        3       /* Gauss Clock Divider bit 11 */
#define    BM_GACDIV11	(uint8_t)(1U<<GACDIV11)
#define    GACDIV12        4       /* Gauss Clock Divider bit 12 */
#define    BM_GACDIV12	(uint8_t)(1U<<GACDIV12)

/* GACDIVL - Gauss Clock Divider, Low Byte */
#define    GACDIV0         0       /* Gauss Clock Divider bit 0 */
#define    BM_GACDIV0	(uint8_t)(1U<<GACDIV0)
#define    GACDIV1         1       /* Gauss Clock Divider bit 1 */
#define    BM_GACDIV1	(uint8_t)(1U<<GACDIV1)
#define    GACDIV2         2       /* Gauss Clock Divider bit 2 */
#define    BM_GACDIV2	(uint8_t)(1U<<GACDIV2)
#define    GACDIV3         3       /* Gauss Clock Divider bit 3 */
#define    BM_GACDIV3	(uint8_t)(1U<<GACDIV3)
#define    GACDIV4         4       /* Gauss Clock Divider bit 4 */
#define    BM_GACDIV4	(uint8_t)(1U<<GACDIV4)
#define    GACDIV5         5       /* Gauss Clock Divider bit 5 */
#define    BM_GACDIV5	(uint8_t)(1U<<GACDIV5)
#define    GACDIV6         6       /* Gauss Clock Divider bit 6 */
#define    BM_GACDIV6	(uint8_t)(1U<<GACDIV6)
#define    GACDIV7         7       /* Gauss Clock Divider bit 7 */
#define    BM_GACDIV7	(uint8_t)(1U<<GACDIV7)


/* ***** TXM ************************** */
/* TMCIH - Tx Modulator CRC Init Value, High Byte */
#define    TMCI8           0       /* Tx Modulator CRC Init Value bit 8 */
#define    BM_TMCI8	(uint8_t)(1U<<TMCI8)
#define    TMCI9           1       /* Tx Modulator CRC Init Value bit 9 */
#define    BM_TMCI9	(uint8_t)(1U<<TMCI9)
#define    TMCI10          2       /* Tx Modulator CRC Init Value bit 10 */
#define    BM_TMCI10	(uint8_t)(1U<<TMCI10)
#define    TMCI11          3       /* Tx Modulator CRC Init Value bit 11 */
#define    BM_TMCI11	(uint8_t)(1U<<TMCI11)
#define    TMCI12          4       /* Tx Modulator CRC Init Value bit 12 */
#define    BM_TMCI12	(uint8_t)(1U<<TMCI12)
#define    TMCI13          5       /* Tx Modulator CRC Init Value bit 13 */
#define    BM_TMCI13	(uint8_t)(1U<<TMCI13)
#define    TMCI14          6       /* Tx Modulator CRC Init Value bit 14 */
#define    BM_TMCI14	(uint8_t)(1U<<TMCI14)
#define    TMCI15          7       /* Tx Modulator CRC Init Value bit 15 */
#define    BM_TMCI15	(uint8_t)(1U<<TMCI15)

/* TMCIL - Tx Modulator CRC Init Value Low Byte */
#define    TMCI0           0       /* Tx Modulator CRC Init Value bit 0 */
#define    BM_TMCI0	(uint8_t)(1U<<TMCI0)
#define    TMCI1           1       /* Tx Modulator CRC Init Value bit 1 */
#define    BM_TMCI1	(uint8_t)(1U<<TMCI1)
#define    TMCI2           2       /* Tx Modulator CRC Init Value bit 2 */
#define    BM_TMCI2	(uint8_t)(1U<<TMCI2)
#define    TMCI3           3       /* Tx Modulator CRC Init Value bit 3 */
#define    BM_TMCI3	(uint8_t)(1U<<TMCI3)
#define    TMCI4           4       /* Tx Modulator CRC Init Value bit 4 */
#define    BM_TMCI4	(uint8_t)(1U<<TMCI4)
#define    TMCI5           5       /* Tx Modulator CRC Init Value bit 5 */
#define    BM_TMCI5	(uint8_t)(1U<<TMCI5)
#define    TMCI6           6       /* Tx Modulator CRC Init Value bit 6 */
#define    BM_TMCI6	(uint8_t)(1U<<TMCI6)
#define    TMCI7           7       /* Tx Modulator CRC Init Value bit 7 */
#define    BM_TMCI7	(uint8_t)(1U<<TMCI7)

/* TMCPH - Tx Modulator CRC Polynomial, High Byte */
#define    TMCP8           0       /* Tx Modulator CRC Polynomial bit 8 */
#define    BM_TMCP8	(uint8_t)(1U<<TMCP8)
#define    TMCP9           1       /* Tx Modulator CRC Polynomial bit 9 */
#define    BM_TMCP9	(uint8_t)(1U<<TMCP9)
#define    TMCP10          2       /* Tx Modulator CRC Polynomial bit 10 */
#define    BM_TMCP10	(uint8_t)(1U<<TMCP10)
#define    TMCP11          3       /* Tx Modulator CRC Polynomial bit 11 */
#define    BM_TMCP11	(uint8_t)(1U<<TMCP11)
#define    TMCP12          4       /* Tx Modulator CRC Polynomial bit 12 */
#define    BM_TMCP12	(uint8_t)(1U<<TMCP12)
#define    TMCP13          5       /* Tx Modulator CRC Polynomial bit 13 */
#define    BM_TMCP13	(uint8_t)(1U<<TMCP13)
#define    TMCP14          6       /* Tx Modulator CRC Polynomial bit 14 */
#define    BM_TMCP14	(uint8_t)(1U<<TMCP14)
#define    TMCP15          7       /* Tx Modulator CRC Polynomial bit 15 */
#define    BM_TMCP15	(uint8_t)(1U<<TMCP15)

/* TMCPL - Tx Modulator CRC Polynomial, Low Byte */
#define    TMCP0           0       /* Tx Modulator CRC Polynomial bit 0 */
#define    BM_TMCP0	(uint8_t)(1U<<TMCP0)
#define    TMCP1           1       /* Tx Modulator CRC Polynomial bit 1 */
#define    BM_TMCP1	(uint8_t)(1U<<TMCP1)
#define    TMCP2           2       /* Tx Modulator CRC Polynomial bit 2 */
#define    BM_TMCP2	(uint8_t)(1U<<TMCP2)
#define    TMCP3           3       /* Tx Modulator CRC Polynomial bit 3 */
#define    BM_TMCP3	(uint8_t)(1U<<TMCP3)
#define    TMCP4           4       /* Tx Modulator CRC Polynomial bit 4 */
#define    BM_TMCP4	(uint8_t)(1U<<TMCP4)
#define    TMCP5           5       /* Tx Modulator CRC Polynomial bit 5 */
#define    BM_TMCP5	(uint8_t)(1U<<TMCP5)
#define    TMCP6           6       /* Tx Modulator CRC Polynomial bit 6 */
#define    BM_TMCP6	(uint8_t)(1U<<TMCP6)
#define    TMCP7           7       /* Tx Modulator CRC Polynomial bit 7 */
#define    BM_TMCP7	(uint8_t)(1U<<TMCP7)

/* TMCR1 - Tx Modulator Control Register 1 */
#define    TMPIS0          0       /* Tx Modulator Port Interface Select bit 0 */
#define    BM_TMPIS0	(uint8_t)(1U<<TMPIS0)
#define    TMPIS1          1       /* Tx Modulator Port Interface Select bit 1 */
#define    BM_TMPIS1	(uint8_t)(1U<<TMPIS1)
#define    TMPIS2          2       /* Tx Modulator Port Interface Select bit 2 */
#define    BM_TMPIS2	(uint8_t)(1U<<TMPIS2)
#define    TMSCS           3       /* Tx Modulator Serial Output Clock Select */
#define    BM_TMSCS	(uint8_t)(1U<<TMSCS)
#define    TMCIM           4       /* Tx Modulator Transmission Complete Interrupt Mask */
#define    BM_TMCIM	(uint8_t)(1U<<TMCIM)

/* TMCR2 - Tx Modulator Control Register 2 */
#define    TMCRCE          0       /* Tx Modulator CRC Enable */
#define    BM_TMCRCE	(uint8_t)(1U<<TMCRCE)
#define    TMCRCSE0        1       /* Tx Modulator CRC Select bit 0 */
#define    BM_TMCRCSE0	(uint8_t)(1U<<TMCRCSE0)
#define    TMCRCSE1        2       /* Tx Modulator CRC Select bit 1 */
#define    BM_TMCRCSE1	(uint8_t)(1U<<TMCRCSE1)
#define    TMNRZE          3       /* Tx Modulator NRZ Mode Enable */
#define    BM_TMNRZE	(uint8_t)(1U<<TMNRZE)
#define    TMPOL           4       /* Tx Modulator Polarity */
#define    BM_TMPOL	(uint8_t)(1U<<TMPOL)
#define    TMSSE           5       /* Tx Modulator Stop Sequence Enable */
#define    BM_TMSSE	(uint8_t)(1U<<TMSSE)
#define    TMLSB           6       /* Tx Modulator Least Significant Bit First */
#define    BM_TMLSB	(uint8_t)(1U<<TMLSB)

/* TMCRCH - Tx Modulator CRC Result, High Byte */
#define    TMCRC8          0       /* Tx Modulator CRC Result bit 8 */
#define    BM_TMCRC8	(uint8_t)(1U<<TMCRC8)
#define    TMCRC9          1       /* Tx Modulator CRC Result bit 9 */
#define    BM_TMCRC9	(uint8_t)(1U<<TMCRC9)
#define    TMCRC10         2       /* Tx Modulator CRC Result bit 10 */
#define    BM_TMCRC10	(uint8_t)(1U<<TMCRC10)
#define    TMCRC11         3       /* Tx Modulator CRC Result bit 11 */
#define    BM_TMCRC11	(uint8_t)(1U<<TMCRC11)
#define    TMCRC12         4       /* Tx Modulator CRC Result bit 12 */
#define    BM_TMCRC12	(uint8_t)(1U<<TMCRC12)
#define    TMCRC13         5       /* Tx Modulator CRC Result bit 13 */
#define    BM_TMCRC13	(uint8_t)(1U<<TMCRC13)
#define    TMCRC14         6       /* Tx Modulator CRC Result bit 14 */
#define    BM_TMCRC14	(uint8_t)(1U<<TMCRC14)
#define    TMCRC15         7       /* Tx Modulator CRC Result bit 15 */
#define    BM_TMCRC15	(uint8_t)(1U<<TMCRC15)

/* TMCRCL - Tx Modulator CRC Result, Low Byte */
#define    TMCRC0          0       /* Tx Modulator CRC Result bit 0 */
#define    BM_TMCRC0	(uint8_t)(1U<<TMCRC0)
#define    TMCRC1          1       /* Tx Modulator CRC Result bit 1 */
#define    BM_TMCRC1	(uint8_t)(1U<<TMCRC1)
#define    TMCRC2          2       /* Tx Modulator CRC Result bit 2 */
#define    BM_TMCRC2	(uint8_t)(1U<<TMCRC2)
#define    TMCRC3          3       /* Tx Modulator CRC Result bit 3 */
#define    BM_TMCRC3	(uint8_t)(1U<<TMCRC3)
#define    TMCRC4          4       /* Tx Modulator CRC Result bit 4 */
#define    BM_TMCRC4	(uint8_t)(1U<<TMCRC4)
#define    TMCRC5          5       /* Tx Modulator CRC Result bit 5 */
#define    BM_TMCRC5	(uint8_t)(1U<<TMCRC5)
#define    TMCRC6          6       /* Tx Modulator CRC Result bit 6 */
#define    BM_TMCRC6	(uint8_t)(1U<<TMCRC6)
#define    TMCRC7          7       /* Tx Modulator CRC Result bit 7 */
#define    BM_TMCRC7	(uint8_t)(1U<<TMCRC7)

/* TMCSB - Tx Modulator CRC Skip Bit Number */
#define    TMCSB0          0       /* Tx Modulator CRC Skip Bit Number bit 0 */
#define    BM_TMCSB0	(uint8_t)(1U<<TMCSB0)
#define    TMCSB1          1       /* Tx Modulator CRC Skip Bit Number bit 1 */
#define    BM_TMCSB1	(uint8_t)(1U<<TMCSB1)
#define    TMCSB2          2       /* Tx Modulator CRC Skip Bit Number bit 2 */
#define    BM_TMCSB2	(uint8_t)(1U<<TMCSB2)
#define    TMCSB3          3       /* Tx Modulator CRC Skip Bit Number bit 3 */
#define    BM_TMCSB3	(uint8_t)(1U<<TMCSB3)
#define    TMCSB4          4       /* Tx Modulator CRC Skip Bit Number bit 4 */
#define    BM_TMCSB4	(uint8_t)(1U<<TMCSB4)
#define    TMCSB5          5       /* Tx Modulator CRC Skip Bit Number bit 5 */
#define    BM_TMCSB5	(uint8_t)(1U<<TMCSB5)
#define    TMCSB6          6       /* Tx Modulator CRC Skip Bit Number bit 6 */
#define    BM_TMCSB6	(uint8_t)(1U<<TMCSB6)
#define    TMCSB7          7       /* Tx Modulator CRC Skip Bit Number bit 7 */
#define    BM_TMCSB7	(uint8_t)(1U<<TMCSB7)

/* TMFSM - Tx Modulator Finite State Machine */
#define    TMSSM0          0       /* Tx Modulator Sub State Machine bit 0 */
#define    BM_TMSSM0	(uint8_t)(1U<<TMSSM0)
#define    TMSSM1          1       /* Tx Modulator Sub State Machine bit 1 */
#define    BM_TMSSM1	(uint8_t)(1U<<TMSSM1)
#define    TMSSM2          2       /* Tx Modulator Sub State Machine bit 2 */
#define    BM_TMSSM2	(uint8_t)(1U<<TMSSM2)
#define    TMSSM3          3       /* Tx Modulator Sub State Machine bit 3 */
#define    BM_TMSSM3	(uint8_t)(1U<<TMSSM3)
#define    TMMSM0          4       /* Tx Modulator Master State Machine bit 0 */
#define    BM_TMMSM0	(uint8_t)(1U<<TMMSM0)
#define    TMMSM1          5       /* Tx Modulator Master State Machine bit 1 */
#define    BM_TMMSM1	(uint8_t)(1U<<TMMSM1)
#define    TMMSM2          6       /* Tx Modulator Master State Machine bit 2 */
#define    BM_TMMSM2	(uint8_t)(1U<<TMMSM2)

/* TMSHR - Tx Modulator Shift Register */
#define    TMSHR0          0       /* Tx Modulator Shift Register bit 0 */
#define    BM_TMSHR0	(uint8_t)(1U<<TMSHR0)
#define    TMSHR1          1       /* Tx Modulator Shift Register bit 1 */
#define    BM_TMSHR1	(uint8_t)(1U<<TMSHR1)
#define    TMSHR2          2       /* Tx Modulator Shift Register bit 2 */
#define    BM_TMSHR2	(uint8_t)(1U<<TMSHR2)
#define    TMSHR3          3       /* Tx Modulator Shift Register bit 3 */
#define    BM_TMSHR3	(uint8_t)(1U<<TMSHR3)
#define    TMSHR4          4       /* Tx Modulator Shift Register bit 4 */
#define    BM_TMSHR4	(uint8_t)(1U<<TMSHR4)
#define    TMSHR5          5       /* Tx Modulator Shift Register bit 5 */
#define    BM_TMSHR5	(uint8_t)(1U<<TMSHR5)
#define    TMSHR6          6       /* Tx Modulator Shift Register bit 6 */
#define    BM_TMSHR6	(uint8_t)(1U<<TMSHR6)
#define    TMSHR7          7       /* Tx Modulator Shift Register bit 7 */
#define    BM_TMSHR7	(uint8_t)(1U<<TMSHR7)

/* TMSR - Tx Modulator Status Register */
#define    TMTCF           0       /* Tx Modulator Transmission Complete Status Flag */
#define    BM_TMTCF	(uint8_t)(1U<<TMTCF)

/* TMSSC - Tx Modulator Stop Sequence Configuration */
#define    TMSSP0          0       /* Tx Modulator Stop Sequence Pattern bit 0 */
#define    BM_TMSSP0	(uint8_t)(1U<<TMSSP0)
#define    TMSSP1          1       /* Tx Modulator Stop Sequence Pattern bit 1 */
#define    BM_TMSSP1	(uint8_t)(1U<<TMSSP1)
#define    TMSSP2          2       /* Tx Modulator Stop Sequence Pattern bit 2 */
#define    BM_TMSSP2	(uint8_t)(1U<<TMSSP2)
#define    TMSSP3          3       /* Tx Modulator Stop Sequence Pattern bit 3 */
#define    BM_TMSSP3	(uint8_t)(1U<<TMSSP3)
#define    TMSSL0          4       /* Tx Modulator Stop Sequence Length bit 0 */
#define    BM_TMSSL0	(uint8_t)(1U<<TMSSL0)
#define    TMSSL1          5       /* Tx Modulator Stop Sequence Length bit 1 */
#define    BM_TMSSL1	(uint8_t)(1U<<TMSSL1)
#define    TMSSL2          6       /* Tx Modulator Stop Sequence Length bit 2 */
#define    BM_TMSSL2	(uint8_t)(1U<<TMSSL2)
#define    TMSSH           7       /* Tx Modulator Stop Sequence Hold Mode */
#define    BM_TMSSH	(uint8_t)(1U<<TMSSH)

/* TMTLH - Tx Modulator Telegram Length Register, High Byte */
#define    TMTL8           0       /* Tx Modulator Telegram Length bit 8 */
#define    BM_TMTL8	(uint8_t)(1U<<TMTL8)
#define    TMTL9           1       /* Tx Modulator Telegram Length bit 9 */
#define    BM_TMTL9	(uint8_t)(1U<<TMTL9)
#define    TMTL10          2       /* Tx Modulator Telegram Length bit 10 */
#define    BM_TMTL10	(uint8_t)(1U<<TMTL10)
#define    TMTL11          3       /* Tx Modulator Telegram Length bit 11 */
#define    BM_TMTL11	(uint8_t)(1U<<TMTL11)

/* TMTLL - Tx Modulator Telegram Length Register, Low Byte */
#define    TMTL0           0       /* Tx Modulator Telegram Length bit 0 */
#define    BM_TMTL0	(uint8_t)(1U<<TMTL0)
#define    TMTL1           1       /* Tx Modulator Telegram Length bit 1 */
#define    BM_TMTL1	(uint8_t)(1U<<TMTL1)
#define    TMTL2           2       /* Tx Modulator Telegram Length bit 2 */
#define    BM_TMTL2	(uint8_t)(1U<<TMTL2)
#define    TMTL3           3       /* Tx Modulator Telegram Length bit 3 */
#define    BM_TMTL3	(uint8_t)(1U<<TMTL3)
#define    TMTL4           4       /* Tx Modulator Telegram Length bit 4 */
#define    BM_TMTL4	(uint8_t)(1U<<TMTL4)
#define    TMTL5           5       /* Tx Modulator Telegram Length bit 5 */
#define    BM_TMTL5	(uint8_t)(1U<<TMTL5)
#define    TMTL6           6       /* Tx Modulator Telegram Length bit 6 */
#define    BM_TMTL6	(uint8_t)(1U<<TMTL6)
#define    TMTL7           7       /* Tx Modulator Telegram Length bit 7 */
#define    BM_TMTL7	(uint8_t)(1U<<TMTL7)



/* ***** LOCKSBITS ******************************************************** */
#define    LB1             0       /* Lock bit */
#define    LB2             1       /* Lock bit */
#define    BLB01           2       /* Boot Lock bit */
#define    BLB02           3       /* Boot Lock bit */
#define    BLB11           4       /* Boot lock bit */
#define    BLB12           5       /* Boot lock bit */


/* ***** FUSES ************************************************************ */
/* LOW fuse bits */



/* ***** CPU REGISTER DEFINITIONS ***************************************** */
#define    XH              r27
#define    XL              r26
#define    YH              r29
#define    YL              r28
#define    ZH              r31
#define    ZL              r30



/* ***** DATA MEMORY DECLARATIONS ***************************************** */
#define    FLASHEND        0xffff  /* Note: Byte address */
#define    IOEND           0x01ff
#define    SRAM_START      0x0200
#define    SRAM_SIZE       1024
#define    RAMEND          0x05ff
#define    XRAMEND         0x0000
#define    E2END           0x08ff
#define    EEPROMEND       0x08ff
#define    EEADRBITS       11



/* ***** BOOTLOADER DECLARATIONS ****************************************** */
#define    NRWW_START_ADDR 0xc00
#define    NRWW_STOP_ADDR  0xfff
#define    RWW_START_ADDR  0x0
#define    RWW_STOP_ADDR   0xbff
#define    PAGESIZE        32
#define    FIRSTBOOTSTART  0x7000
#define    SECONDBOOTSTART 0x6000
#define    THIRDBOOTSTART  0x4000
#define    FOURTHBOOTSTART 0x3800
#define    SMALLBOOTSTART  FIRSTBOOTSTART
#define    LARGEBOOTSTART  FOURTHBOOTSTART

#endif /* ENABLE_BIT_DEFINITIONS */



/* ***** INTERRUPT VECTORS ************************************************ */
#define    RESET_vect      (0x00)  /* External Pin, Power-on Reset, Brown-out Reset and Watchdog Reset */
#define    INT0_vect       (0x04)  /* External Interrupt Request 0 */
#define    INT1_vect       (0x08)  /* External Interrupt Request 1 */
#define    PCINT0_vect     (0x0c)  /* Pin Change Interrupt Request 0 */
#define    PCINT1_vect     (0x10)  /* Pin Change Interrupt Request 1 */
#define    VMON_vect       (0x14)  /* Voltage Monitoring Interrupt */
#define    AVCCR_vect      (0x18)  /* AVCC Reset Interrupt */
#define    AVCCL_vect      (0x1c)  /* AVCC Low Interrupt */
#define    T0INT_vect      (0x20)  /* Timer0 Interval Interrupt */
#define    T1COMP_vect     (0x24)  /* Timer/Counter1 Compare Match Interrupt */
#define    T1OVF_vect      (0x28)  /* Timer/Counter1 Overflow Interrupt */
#define    T2COMP_vect     (0x2c)  /* Timer/Counter2 Compare Match Interrupt */
#define    T2OVF_vect      (0x30)  /* Timer/Counter2 Overflow Interrupt */
#define    T3CAP_vect      (0x34)  /* Timer/Counter3 Capture Event Interrupt */
#define    T3COMP_vect     (0x38)  /* Timer/Counter3 Compare Match Interrupt */
#define    T3OVF_vect      (0x3c)  /* Timer/Counter3 Overflow Interrupt */
#define    T4CAP_vect      (0x40)  /* Timer/Counter4 Capture Event Interrupt */
#define    T4COMP_vect     (0x44)  /* Timer/Counter4 Compare Match Interrupt */
#define    T4OVF_vect      (0x48)  /* Timer/Counter4 Overflow Interrupt */
#define    T5COMP_vect     (0x4c)  /* Timer/Counter5 Compare Match Interrupt */
#define    T5OVF_vect      (0x50)  /* Timer/Counter5 Overflow Interrupt */
#define    SPI_STC_vect    (0x54)  /* SPI Serial Transfer Complete Interrupt */
#define    SRX_FIFO_vect   (0x58)  /* SPI Rx Buffer Interrupt */
#define    STX_FIFO_vect   (0x5c)  /* SPI Tx Buffer Interrupt */
#define    SSM_vect        (0x60)  /* Sequencer State Machine Interrupt */
#define    DFFLR_vect      (0x64)  /* Data FIFO fill level reached Interrupt */
#define    DFOUE_vect      (0x68)  /* Data FIFO overflow or underflow error Interrupt */
#define    SFFLR_vect      (0x6c)  /* RSSI/Preamble FIFO fill level reached Interrupt */
#define    SFOUE_vect      (0x70)  /* RSSI/Preamble FIFO overflow or underflow error Interrupt */
#define    TMTCF_vect      (0x74)  /* Tx Modulator Telegram Finish Interrupt */
#define    AES_vect        (0x78)  /* AES Krypto Unit Interrupt */
#define    TPINT_vect      (0x7c)  /* Transponder Mode Interrupt */
#define    TPTOERR_vect    (0x80)  /* Transponder Timeout Error Interrupt */
#define    LFID0INT_vect   (0x84)  /* LF receiver Identifier 0 Interrupt */
#define    LFID1INT_vect   (0x88)  /* LF receiver Identifier 1 Interrupt */
#define    LFFEINT_vect    (0x8c)  /* LF receiver Frame End Interrupt */
#define    LFBCR_vect      (0x90)  /* LF receiver Bit Count Reached Interrupt */
#define    LFPBD_vect      (0x94)  /* LF receiver PreBurst Detected Interrupt */
#define    LFDE_vect       (0x98)  /* LF receiver Decoder Error Interrupt */
#define    LFEOT_vect      (0x9c)  /* LF receiver End of Telegram Interrupt */
#define    LFTCOR_vect     (0xa0)  /* LF receiver Timer Compare Match Interrupt */
#define    LFRSCO_vect     (0xa4)  /* LF receiver RSSI measurement Interrupt */
#define    PHFFLR_vect     (0xa8)  /* Protocol Handler FIFO Fill Level Reached Interrupt */
#define    PHFOUE_vect     (0xac)  /* Protocol Handler FIFO Overflow or Underflow Error Interrupt */
#define    EXCM_vect       (0xb0)  /* External input Clock monitoring Interrupt */
#define    E2CINT_vect     (0xb4)  /* EEPROM Error Correction Interrupt */
#define    ERDY_vect       (0xb8)  /* EEPROM Ready Interrupt */
#define    SPM_RDY_vect    (0xbc)  /* Store Program Memory Ready */
#define    TWI_vect        (0xc0)  /* TWI Interrupt */



#endif /* __IOATA5700_H (define part) */

#pragma language=default

#endif  /* __IOATA5700_H (SFR part)*/

/* ***** END OF FILE ****************************************************** */
#endif /* IOATA5700_H_OVERALL */
