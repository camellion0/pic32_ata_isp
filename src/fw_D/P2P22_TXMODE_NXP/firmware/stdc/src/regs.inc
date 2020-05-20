;***** THIS IS A MACHINE GENERATED FILE - DO NOT EDIT ********************
;***** Created: 2015-03-13 15:19 ******* Source: ATA5702M322_internal.xml 
;*************************************************************************
;* A P P L I C A T I O N   N O T E   F O R   T H E   A V R   F A M I L Y
;* 
;* Number            : AVR000
;* File Name         : "ATA5702M322_internaldef.inc"
;* Title             : Register/Bit Definitions for the ATA5702M322
;* Date              : 2015-03-13
;* Version           : 2.25
;* Support E-mail    : avr@atmel.com
;* Target MCU        : ATA5702M322
;* 
;* DESCRIPTION
;* When including this file in the assembly program file, all I/O register 
;* names and I/O register bit names appearing in the data book can be used.
;* In addition, the six registers forming the three data pointers X, Y and 
;* Z have been assigned names XL - ZH. Highest RAM address for Internal 
;* SRAM is also defined 
;* 
;* The Register names are represented by their hexadecimal address.
;* 
;* The Register Bit names are represented by their bit number (0-7).
;* 
;* Please observe the difference in using the bit names with instructions
;* such as "sbr"/"cbr" (set/clear bit in register) and "sbrs"/"sbrc"
;* (skip if bit in register set/cleared). The following example illustrates
;* this:
;* 
;* in    r16,PORTB             ;read PORTB latch
;* sbr   r16,(1<<PB6)+(1<<PB5) ;set PB6 and PB5 (use masks, not bit#)
;* out   PORTB,r16             ;output to PORTB
;* 
;* in    r16,TIFR              ;read the Timer Interrupt Flag Register
;* sbrc  r16,TOV0              ;test the overflow flag (use bit#)
;* rjmp  TOV0_is_set           ;jump if set
;* ...                         ;otherwise do something else
;*************************************************************************
#ifndef _REGDEF_INC_
#define _REGDEF_INC_
; ***** SPECIFY DEVICE ***************************************************
;.device ATA5702M322
SIGNATURE_000       EQU 0x1e
SIGNATURE_001       EQU 0x95
SIGNATURE_002       EQU 0x69
; ***** I/O REGISTER DEFINITIONS *****************************************
; NOTE:
; Definitions marked "MEMORY MAPPED"are extended I/O ports
; and cannot be used with IN/OUT instructions
GPIOR0       EQU 0x00
PRR1       EQU 0x01
PRR2       EQU 0x02
PINB       EQU 0x03
DDRB       EQU 0x04
PORTB       EQU 0x05
PINC       EQU 0x06
DDRC       EQU 0x07
PORTC       EQU 0x08
PIND       EQU 0x09
DDRD       EQU 0x0a
PORTD       EQU 0x0b
TPCR2       EQU 0x0c
TPFR       EQU 0x0d
MCUCR       EQU 0x0e
FSCR       EQU 0x0f
T1CR       EQU 0x11
T2CR       EQU 0x12
T3CR       EQU 0x13
T4CR       EQU 0x14
LTCMR       EQU 0x15
EECR2       EQU 0x16
PHTCR       EQU 0x17
LDFFL       EQU 0x18
LDFD       EQU 0x19
PRR0       EQU 0x1a
PHFR       EQU 0x1b
LFFR       EQU 0x1c
AESCR       EQU 0x1d
AESSR       EQU 0x1e
EECR       EQU 0x1f
EEDR       EQU 0x20
EEARL       EQU 0x21
EEARH       EQU 0x22
EEPR0       EQU 0x23
GPIOR1       EQU 0x24
GPIOR2       EQU 0x25
PCICR       EQU 0x26
EIMSK       EQU 0x27
EIFR       EQU 0x28
LDFCKSW       EQU 0x29
VMSCR       EQU 0x2a
MCUSR       EQU 0x2b
SPCR       EQU 0x2c
SPSR       EQU 0x2d
SPDR       EQU 0x2e
LFCR0       EQU 0x2f
LFCR1       EQU 0x30
DWDR       EQU 0x31
T0IFR       EQU 0x32
SPMCSR       EQU 0x37
SMCR       EQU 0x38
TPSR       EQU 0x39
LFCR2       EQU 0x3a
LFCR3       EQU 0x3b
SPL       EQU 0x3d
SPH       EQU 0x3e
SREG       EQU 0x3f
FSEN       EQU 0x60       ; MEMORY MAPPED
FSFCR       EQU 0x61       ; MEMORY MAPPED
GACDIVL       EQU 0x62       ; MEMORY MAPPED
GACDIVH       EQU 0x63       ; MEMORY MAPPED
FFREQ1L       EQU 0x64       ; MEMORY MAPPED
FFREQ1M       EQU 0x65       ; MEMORY MAPPED
FFREQ1H       EQU 0x66       ; MEMORY MAPPED
FFREQ2L       EQU 0x67       ; MEMORY MAPPED
FFREQ2M       EQU 0x68       ; MEMORY MAPPED
FFREQ2H       EQU 0x69       ; MEMORY MAPPED
BBTE2       EQU 0x6a       ; MEMORY MAPPED
EICRA       EQU 0x6b       ; MEMORY MAPPED
PCMSK0       EQU 0x6c       ; MEMORY MAPPED
PCMSK1       EQU 0x6d       ; MEMORY MAPPED
WDTCR       EQU 0x6e       ; MEMORY MAPPED
T1CNT       EQU 0x6f       ; MEMORY MAPPED
T1COR       EQU 0x70       ; MEMORY MAPPED
T1MR       EQU 0x71       ; MEMORY MAPPED
T1IMR       EQU 0x72       ; MEMORY MAPPED
T2CNT       EQU 0x73       ; MEMORY MAPPED
T2COR       EQU 0x74       ; MEMORY MAPPED
T2MR       EQU 0x75       ; MEMORY MAPPED
T2IMR       EQU 0x76       ; MEMORY MAPPED
T3CNTL       EQU 0x77       ; MEMORY MAPPED
T3CNTH       EQU 0x78       ; MEMORY MAPPED
T3CORL       EQU 0x79       ; MEMORY MAPPED
T3CORH       EQU 0x7a       ; MEMORY MAPPED
T3ICRL       EQU 0x7b       ; MEMORY MAPPED
T3ICRH       EQU 0x7c       ; MEMORY MAPPED
T3MRA       EQU 0x7d       ; MEMORY MAPPED
T3MRB       EQU 0x7e       ; MEMORY MAPPED
T3IMR       EQU 0x7f       ; MEMORY MAPPED
T4CNTL       EQU 0x80       ; MEMORY MAPPED
T4CNTH       EQU 0x81       ; MEMORY MAPPED
T4CORL       EQU 0x82       ; MEMORY MAPPED
T4CORH       EQU 0x83       ; MEMORY MAPPED
T4ICRL       EQU 0x84       ; MEMORY MAPPED
T4ICRH       EQU 0x85       ; MEMORY MAPPED
T4MRA       EQU 0x86       ; MEMORY MAPPED
T4MRB       EQU 0x87       ; MEMORY MAPPED
T4IMR       EQU 0x88       ; MEMORY MAPPED
T5TEMP       EQU 0x89       ; MEMORY MAPPED
T5OCRL       EQU 0x8a       ; MEMORY MAPPED
T5OCRH       EQU 0x8b       ; MEMORY MAPPED
T5CCR       EQU 0x8c       ; MEMORY MAPPED
T5CNTL       EQU 0x8d       ; MEMORY MAPPED
T5CNTH       EQU 0x8e       ; MEMORY MAPPED
T5IMR       EQU 0x8f       ; MEMORY MAPPED
LFCALR1       EQU 0x90       ; MEMORY MAPPED
LFCALR2       EQU 0x91       ; MEMORY MAPPED
LFCALR3       EQU 0x92       ; MEMORY MAPPED
LFCALR4       EQU 0x93       ; MEMORY MAPPED
LFCALR5       EQU 0x94       ; MEMORY MAPPED
LFCALR6       EQU 0x95       ; MEMORY MAPPED
LFCALR7       EQU 0x96       ; MEMORY MAPPED
LFCALR8       EQU 0x97       ; MEMORY MAPPED
LFCALR9       EQU 0x98       ; MEMORY MAPPED
LFCALR10       EQU 0x99       ; MEMORY MAPPED
LFCALR11       EQU 0x9a       ; MEMORY MAPPED
LFCALR12       EQU 0x9b       ; MEMORY MAPPED
LFCALR13       EQU 0x9c       ; MEMORY MAPPED
LFCALR14       EQU 0x9d       ; MEMORY MAPPED
LFCALR15       EQU 0x9e       ; MEMORY MAPPED
LFCALR16       EQU 0x9f       ; MEMORY MAPPED
LFCALR17       EQU 0xa0       ; MEMORY MAPPED
LFCALR18       EQU 0xa1       ; MEMORY MAPPED
LFCALR19       EQU 0xa2       ; MEMORY MAPPED
LFCALR20       EQU 0xa3       ; MEMORY MAPPED
LFCALR21       EQU 0xa4       ; MEMORY MAPPED
LFCALR22       EQU 0xa5       ; MEMORY MAPPED
LFCALR23       EQU 0xa6       ; MEMORY MAPPED
LFCALR24       EQU 0xa7       ; MEMORY MAPPED
LFCALR25       EQU 0xa8       ; MEMORY MAPPED
LFCALR26       EQU 0xa9       ; MEMORY MAPPED
LFCALR27       EQU 0xaa       ; MEMORY MAPPED
LFCALR28       EQU 0xab       ; MEMORY MAPPED
LFCALR29       EQU 0xac       ; MEMORY MAPPED
LFCALR30       EQU 0xad       ; MEMORY MAPPED
LFCALR31       EQU 0xae       ; MEMORY MAPPED
LFCALR32       EQU 0xaf       ; MEMORY MAPPED
LFCALR33       EQU 0xb0       ; MEMORY MAPPED
LFCALR34       EQU 0xb1       ; MEMORY MAPPED
LFCALR35       EQU 0xb2       ; MEMORY MAPPED
LFCALR36       EQU 0xb3       ; MEMORY MAPPED
LFCALR37       EQU 0xb4       ; MEMORY MAPPED
LFCALR38       EQU 0xb5       ; MEMORY MAPPED
LFCALR39       EQU 0xb6       ; MEMORY MAPPED
LFCALR40       EQU 0xb7       ; MEMORY MAPPED
LFCALR41       EQU 0xb8       ; MEMORY MAPPED
LFCALR42       EQU 0xb9       ; MEMORY MAPPED
LFCALR43       EQU 0xba       ; MEMORY MAPPED
LFCALR44       EQU 0xbb       ; MEMORY MAPPED
LFCALR45       EQU 0xbc       ; MEMORY MAPPED
LFCALR46       EQU 0xbd       ; MEMORY MAPPED
LFCALR47       EQU 0xbe       ; MEMORY MAPPED
LFCALR48       EQU 0xbf       ; MEMORY MAPPED
LFCALR49       EQU 0xc0       ; MEMORY MAPPED
LFCALR50       EQU 0xc1       ; MEMORY MAPPED
LFCALR51       EQU 0xc2       ; MEMORY MAPPED
LFCALR52       EQU 0xc3       ; MEMORY MAPPED
LFCALR53       EQU 0xc4       ; MEMORY MAPPED
XFUSE       EQU 0xc5       ; MEMORY MAPPED
MRCCAL       EQU 0xc6       ; MEMORY MAPPED
FRCCAL       EQU 0xc7       ; MEMORY MAPPED
RCTCAL       EQU 0xc8       ; MEMORY MAPPED
CMSR       EQU 0xc9       ; MEMORY MAPPED
CMOCR       EQU 0xca       ; MEMORY MAPPED
SUPFR       EQU 0xcb       ; MEMORY MAPPED
SUPCR       EQU 0xcc       ; MEMORY MAPPED
SUPCA1       EQU 0xcd       ; MEMORY MAPPED
SUPCA2       EQU 0xce       ; MEMORY MAPPED
SUPCA3       EQU 0xcf       ; MEMORY MAPPED
SUPCA4       EQU 0xd0       ; MEMORY MAPPED
CALRDY       EQU 0xd1       ; MEMORY MAPPED
DFS       EQU 0xd2       ; MEMORY MAPPED
DFL       EQU 0xd5       ; MEMORY MAPPED
DFWP       EQU 0xd6       ; MEMORY MAPPED
DFRP       EQU 0xd7       ; MEMORY MAPPED
DFD       EQU 0xd8       ; MEMORY MAPPED
DFI       EQU 0xd9       ; MEMORY MAPPED
DFC       EQU 0xda       ; MEMORY MAPPED
SFS       EQU 0xdb       ; MEMORY MAPPED
SFL       EQU 0xdc       ; MEMORY MAPPED
SFWP       EQU 0xdd       ; MEMORY MAPPED
SFRP       EQU 0xde       ; MEMORY MAPPED
SFD       EQU 0xdf       ; MEMORY MAPPED
SFI       EQU 0xe0       ; MEMORY MAPPED
SFC       EQU 0xe1       ; MEMORY MAPPED
SSMCR       EQU 0xe2       ; MEMORY MAPPED
GTCCR       EQU 0xe3       ; MEMORY MAPPED
SSMFBR       EQU 0xe4       ; MEMORY MAPPED
SSMRR       EQU 0xe5       ; MEMORY MAPPED
SSMSR       EQU 0xe6       ; MEMORY MAPPED
SSMIFR       EQU 0xe7       ; MEMORY MAPPED
SSMIMR       EQU 0xe8       ; MEMORY MAPPED
MSMSTR       EQU 0xe9       ; MEMORY MAPPED
SSMSTR       EQU 0xea       ; MEMORY MAPPED
VXMCTRL       EQU 0xeb       ; MEMORY MAPPED
MSMCR1       EQU 0xec       ; MEMORY MAPPED
MSMCR2       EQU 0xed       ; MEMORY MAPPED
MSMCR3       EQU 0xee       ; MEMORY MAPPED
MSMCR4       EQU 0xef       ; MEMORY MAPPED
SP2CR       EQU 0xf7       ; MEMORY MAPPED
SP2DR       EQU 0xf8       ; MEMORY MAPPED
SP2SR       EQU 0xf9       ; MEMORY MAPPED
TRCIDL       EQU 0xfc       ; MEMORY MAPPED
TRCIDH       EQU 0xfd       ; MEMORY MAPPED
TRCDR       EQU 0xff       ; MEMORY MAPPED
FESR       EQU 0x100       ; MEMORY MAPPED
FEEN1       EQU 0x101       ; MEMORY MAPPED
FEEN2       EQU 0x102       ; MEMORY MAPPED
FELNA       EQU 0x103       ; MEMORY MAPPED
FEAT       EQU 0x104       ; MEMORY MAPPED
FEPAC       EQU 0x105       ; MEMORY MAPPED
FEVCT       EQU 0x106       ; MEMORY MAPPED
FEBT       EQU 0x107       ; MEMORY MAPPED
FEMS       EQU 0x108       ; MEMORY MAPPED
FETN4       EQU 0x109       ; MEMORY MAPPED
FECR       EQU 0x10a       ; MEMORY MAPPED
FEVCO       EQU 0x10b       ; MEMORY MAPPED
FEALR       EQU 0x10c       ; MEMORY MAPPED
FEANT       EQU 0x10d       ; MEMORY MAPPED
FEBIA       EQU 0x10e       ; MEMORY MAPPED
SPARE1       EQU 0x10f       ; MEMORY MAPPED
SPARE2       EQU 0x110       ; MEMORY MAPPED
SPARE3       EQU 0x111       ; MEMORY MAPPED
SPARE4       EQU 0x112       ; MEMORY MAPPED
CLKOD       EQU 0x115       ; MEMORY MAPPED
CLKOCR       EQU 0x116       ; MEMORY MAPPED
FETE1       EQU 0x11c       ; MEMORY MAPPED
FETE2       EQU 0x11d       ; MEMORY MAPPED
FETE3       EQU 0x11e       ; MEMORY MAPPED
FETD       EQU 0x11f       ; MEMORY MAPPED
TMFSM       EQU 0x120       ; MEMORY MAPPED
TMCRCL       EQU 0x121       ; MEMORY MAPPED
TMCRCH       EQU 0x122       ; MEMORY MAPPED
TMCSB       EQU 0x123       ; MEMORY MAPPED
TMCIL       EQU 0x124       ; MEMORY MAPPED
TMCIH       EQU 0x125       ; MEMORY MAPPED
TMCPL       EQU 0x126       ; MEMORY MAPPED
TMCPH       EQU 0x127       ; MEMORY MAPPED
TMSHR       EQU 0x128       ; MEMORY MAPPED
TMTLL       EQU 0x129       ; MEMORY MAPPED
TMTLH       EQU 0x12a       ; MEMORY MAPPED
TMSSC       EQU 0x12b       ; MEMORY MAPPED
TMSR       EQU 0x12c       ; MEMORY MAPPED
TMCR2       EQU 0x12d       ; MEMORY MAPPED
TMCR1       EQU 0x12e       ; MEMORY MAPPED
LFDSR1       EQU 0x130       ; MEMORY MAPPED
LFDSR2       EQU 0x131       ; MEMORY MAPPED
LFDSR3       EQU 0x132       ; MEMORY MAPPED
LFDSR4       EQU 0x133       ; MEMORY MAPPED
LFDSR5       EQU 0x134       ; MEMORY MAPPED
LFDSR6       EQU 0x135       ; MEMORY MAPPED
LFDSR7       EQU 0x136       ; MEMORY MAPPED
LFDSR8       EQU 0x137       ; MEMORY MAPPED
LFDSR9       EQU 0x138       ; MEMORY MAPPED
LFDSR10       EQU 0x139       ; MEMORY MAPPED
LFDSR11       EQU 0x13a       ; MEMORY MAPPED
EEPR1       EQU 0x13b       ; MEMORY MAPPED
EEPR2       EQU 0x13c       ; MEMORY MAPPED
EEPR3       EQU 0x13d       ; MEMORY MAPPED
CRCCR       EQU 0x145       ; MEMORY MAPPED
CRCDOR       EQU 0x146       ; MEMORY MAPPED
LFSRCTM       EQU 0x151       ; MEMORY MAPPED
DBCR       EQU 0x152       ; MEMORY MAPPED
DBTC       EQU 0x153       ; MEMORY MAPPED
DBENB       EQU 0x154       ; MEMORY MAPPED
DBENC       EQU 0x155       ; MEMORY MAPPED
DBGSW       EQU 0x156       ; MEMORY MAPPED
SFFR       EQU 0x157       ; MEMORY MAPPED
SFIR       EQU 0x158       ; MEMORY MAPPED
T2IFR       EQU 0x159       ; MEMORY MAPPED
PGMST       EQU 0x15a       ; MEMORY MAPPED
EEST       EQU 0x15b       ; MEMORY MAPPED
LFSRCTL       EQU 0x15c       ; MEMORY MAPPED
PCIFR       EQU 0x161       ; MEMORY MAPPED
T0CR       EQU 0x162       ; MEMORY MAPPED
DBEND       EQU 0x164       ; MEMORY MAPPED
TPCR1       EQU 0x165       ; MEMORY MAPPED
TPIMR       EQU 0x166       ; MEMORY MAPPED
TPDCR1       EQU 0x167       ; MEMORY MAPPED
TPDCR2       EQU 0x168       ; MEMORY MAPPED
TPDCR3       EQU 0x169       ; MEMORY MAPPED
TPDCR4       EQU 0x16a       ; MEMORY MAPPED
TPDCR5       EQU 0x16b       ; MEMORY MAPPED
TPECR1       EQU 0x16c       ; MEMORY MAPPED
TPECR2       EQU 0x16d       ; MEMORY MAPPED
TPECR3       EQU 0x16e       ; MEMORY MAPPED
TPECR4       EQU 0x16f       ; MEMORY MAPPED
TPECMR       EQU 0x170       ; MEMORY MAPPED
TPCR3       EQU 0x171       ; MEMORY MAPPED
TPCR4       EQU 0x172       ; MEMORY MAPPED
TPCR5       EQU 0x173       ; MEMORY MAPPED
TPCALR1       EQU 0x175       ; MEMORY MAPPED
TPCALR2       EQU 0x176       ; MEMORY MAPPED
TPCALR3       EQU 0x177       ; MEMORY MAPPED
TPCALR4       EQU 0x178       ; MEMORY MAPPED
TPCALR5       EQU 0x179       ; MEMORY MAPPED
TPCALR6       EQU 0x17a       ; MEMORY MAPPED
TPCALR7       EQU 0x17b       ; MEMORY MAPPED
TPCALR8       EQU 0x17c       ; MEMORY MAPPED
TPCALR9       EQU 0x17d       ; MEMORY MAPPED
TPCALR10       EQU 0x17e       ; MEMORY MAPPED
AESDPR       EQU 0x17f       ; MEMORY MAPPED
AESKR       EQU 0x180       ; MEMORY MAPPED
AESDR       EQU 0x181       ; MEMORY MAPPED
GPIOR3       EQU 0x182       ; MEMORY MAPPED
GPIOR4       EQU 0x183       ; MEMORY MAPPED
GPIOR5       EQU 0x184       ; MEMORY MAPPED
GPIOR6       EQU 0x185       ; MEMORY MAPPED
GPIOR7       EQU 0x186       ; MEMORY MAPPED
GPIOR8       EQU 0x187       ; MEMORY MAPPED
PHBCRR       EQU 0x188       ; MEMORY MAPPED
LFCPR       EQU 0x18e       ; MEMORY MAPPED
LFIMR       EQU 0x18f       ; MEMORY MAPPED
PHID00       EQU 0x190       ; MEMORY MAPPED
PHID01       EQU 0x191       ; MEMORY MAPPED
PHID02       EQU 0x192       ; MEMORY MAPPED
PHID03       EQU 0x193       ; MEMORY MAPPED
PHID0L       EQU 0x194       ; MEMORY MAPPED
PHID10       EQU 0x195       ; MEMORY MAPPED
PHID11       EQU 0x196       ; MEMORY MAPPED
PHID12       EQU 0x197       ; MEMORY MAPPED
PHID13       EQU 0x198       ; MEMORY MAPPED
PHID1L       EQU 0x199       ; MEMORY MAPPED
PHIDFR       EQU 0x19a       ; MEMORY MAPPED
LFSYSY0       EQU 0x19b       ; MEMORY MAPPED
LFSYSY1       EQU 0x19c       ; MEMORY MAPPED
LFSYSY2       EQU 0x19d       ; MEMORY MAPPED
LFSYSY3       EQU 0x19e       ; MEMORY MAPPED
LFSYLE       EQU 0x19f       ; MEMORY MAPPED
LFSTOP       EQU 0x1a0       ; MEMORY MAPPED
LTCOR       EQU 0x1a1       ; MEMORY MAPPED
T1IFR       EQU 0x1a2       ; MEMORY MAPPED
PHTBLR       EQU 0x1a4       ; MEMORY MAPPED
PHDFR       EQU 0x1a5       ; MEMORY MAPPED
LTEMR       EQU 0x1a6       ; MEMORY MAPPED
LFQC3       EQU 0x1a7       ; MEMORY MAPPED
LFQC2       EQU 0x1a8       ; MEMORY MAPPED
LFQC1       EQU 0x1a9       ; MEMORY MAPPED
TW2BR       EQU 0x1aa       ; MEMORY MAPPED
TW2CR       EQU 0x1ab       ; MEMORY MAPPED
TW2SR       EQU 0x1ac       ; MEMORY MAPPED
TW2DR       EQU 0x1ad       ; MEMORY MAPPED
TW2AR       EQU 0x1ae       ; MEMORY MAPPED
TW2AMR       EQU 0x1af       ; MEMORY MAPPED
RSCR       EQU 0x1b0       ; MEMORY MAPPED
RSSR       EQU 0x1b1       ; MEMORY MAPPED
RSMS1R       EQU 0x1b2       ; MEMORY MAPPED
RSMS2R       EQU 0x1b3       ; MEMORY MAPPED
RSFR       EQU 0x1b4       ; MEMORY MAPPED
RSCALIB       EQU 0x1b6       ; MEMORY MAPPED
RSDLYR       EQU 0x1b7       ; MEMORY MAPPED
RSRES1L       EQU 0x1b8       ; MEMORY MAPPED
RSRES1H       EQU 0x1b9       ; MEMORY MAPPED
RSRES2L       EQU 0x1ba       ; MEMORY MAPPED
RSRES2H       EQU 0x1bb       ; MEMORY MAPPED
RSRES3L       EQU 0x1bc       ; MEMORY MAPPED
RSRES3H       EQU 0x1bd       ; MEMORY MAPPED
RSRES4L       EQU 0x1be       ; MEMORY MAPPED
RSRES4H       EQU 0x1bf       ; MEMORY MAPPED
RSSRCR       EQU 0x1c0       ; MEMORY MAPPED
SD12RR       EQU 0x1c1       ; MEMORY MAPPED
SD13RR       EQU 0x1c2       ; MEMORY MAPPED
SD23RR       EQU 0x1c3       ; MEMORY MAPPED
SD360R       EQU 0x1c4       ; MEMORY MAPPED
RSDBGR       EQU 0x1c5       ; MEMORY MAPPED
LDFS       EQU 0x1d1       ; MEMORY MAPPED
T4IFR       EQU 0x1d2       ; MEMORY MAPPED
LDFWP       EQU 0x1d3       ; MEMORY MAPPED
LDFRP       EQU 0x1d4       ; MEMORY MAPPED
T5IFR       EQU 0x1d5       ; MEMORY MAPPED
LDFIM       EQU 0x1d6       ; MEMORY MAPPED
LDFC       EQU 0x1d7       ; MEMORY MAPPED
PHIMR       EQU 0x1d8       ; MEMORY MAPPED
PHCRCR       EQU 0x1d9       ; MEMORY MAPPED
PHCSTL       EQU 0x1da       ; MEMORY MAPPED
PHCSTH       EQU 0x1db       ; MEMORY MAPPED
PHCRPL       EQU 0x1dc       ; MEMORY MAPPED
PHCRPH       EQU 0x1dd       ; MEMORY MAPPED
PHCSRL       EQU 0x1de       ; MEMORY MAPPED
PHCSRH       EQU 0x1df       ; MEMORY MAPPED
CRCDIR       EQU 0x1e0       ; MEMORY MAPPED
T3IFR       EQU 0x1e1       ; MEMORY MAPPED
CMCR       EQU 0x1e3       ; MEMORY MAPPED
CMIMR       EQU 0x1e4       ; MEMORY MAPPED
CLPR       EQU 0x1e5       ; MEMORY MAPPED
VMCR       EQU 0x1e6       ; MEMORY MAPPED
DBONDR       EQU 0x1e7       ; MEMORY MAPPED
CALRDYLF       EQU 0x1e8       ; MEMORY MAPPED
TW1BR       EQU 0x1e9       ; MEMORY MAPPED
TW1CR       EQU 0x1ea       ; MEMORY MAPPED
TW1SR       EQU 0x1eb       ; MEMORY MAPPED
TW1DR       EQU 0x1ec       ; MEMORY MAPPED
TW1AR       EQU 0x1ed       ; MEMORY MAPPED
TW1AMR       EQU 0x1ee       ; MEMORY MAPPED
PDSCR       EQU 0x1ef       ; MEMORY MAPPED
TMOCR       EQU 0x1f0       ; MEMORY MAPPED
SRCCAL       EQU 0x1f1       ; MEMORY MAPPED
SRCTCAL       EQU 0x1f2       ; MEMORY MAPPED
SUPCA5       EQU 0x1f3       ; MEMORY MAPPED
SUPCA6       EQU 0x1f4       ; MEMORY MAPPED
SUPCA7       EQU 0x1f5       ; MEMORY MAPPED
SUPCA8       EQU 0x1f6       ; MEMORY MAPPED
SUPCA9       EQU 0x1f7       ; MEMORY MAPPED
SUPCA10       EQU 0x1f8       ; MEMORY MAPPED
TPCALR11       EQU 0x1f9       ; MEMORY MAPPED
TPCALR12       EQU 0x1fa       ; MEMORY MAPPED
TPCALR13       EQU 0x1fb       ; MEMORY MAPPED
PMTER       EQU 0x1fe       ; MEMORY MAPPED
SRCCALL       EQU 0x1ff       ; MEMORY MAPPED
; ***** BIT DEFINITIONS **************************************************
; ***** AES **************************
; AESCR - AES Control Register
AESWK       EQU 0       ; AES Write Key
BM_AESWK    EQU (1<<AESWK)       

AESWD       EQU 1       ; AES Write Data
BM_AESWD    EQU (1<<AESWD)       

AESIM       EQU 2       ; AES Interrupt Mask
BM_AESIM    EQU (1<<AESIM)       

AESD       EQU 3       ; AES Direction
BM_AESD    EQU (1<<AESD)       

AESXOR       EQU 4       ; AES State XOR load
BM_AESXOR    EQU (1<<AESXOR)       

AESRES       EQU 5       ; AES Reset
BM_AESRES    EQU (1<<AESRES)       

AESLKM       EQU 6       ; AES Load Key Memory
BM_AESLKM    EQU (1<<AESLKM)       

AESE       EQU 7       ; AES Enable
BM_AESE    EQU (1<<AESE)       

; AESDPR - AES Data Pointer Register
AESDPR0       EQU 0       ; AES Data Pointer Register bit 0
BM_AESDPR0    EQU (1<<AESDPR0)       

AESDPR1       EQU 1       ; AES Data Pointer Register bit 1
BM_AESDPR1    EQU (1<<AESDPR1)       

AESDPR2       EQU 2       ; AES Data Pointer Register bit 2
BM_AESDPR2    EQU (1<<AESDPR2)       

AESDPR3       EQU 3       ; AES Data Pointer Register bit 3
BM_AESDPR3    EQU (1<<AESDPR3)       

; AESDR - AES Data Register
AESDR0       EQU 0       ; AES Data Register bit 0
BM_AESDR0    EQU (1<<AESDR0)       

AESDR1       EQU 1       ; AES Data Register bit 1
BM_AESDR1    EQU (1<<AESDR1)       

AESDR2       EQU 2       ; AES Data Register bit 2
BM_AESDR2    EQU (1<<AESDR2)       

AESDR3       EQU 3       ; AES Data Register bit 3
BM_AESDR3    EQU (1<<AESDR3)       

AESDR4       EQU 4       ; AES Data Register bit 4
BM_AESDR4    EQU (1<<AESDR4)       

AESDR5       EQU 5       ; AES Data Register bit 5
BM_AESDR5    EQU (1<<AESDR5)       

AESDR6       EQU 6       ; AES Data Register bit 6
BM_AESDR6    EQU (1<<AESDR6)       

AESDR7       EQU 7       ; AES Data Register bit 7
BM_AESDR7    EQU (1<<AESDR7)       

; AESKR - AES Key Register
AESKR0       EQU 0       ; AES Key Register bit 0
BM_AESKR0    EQU (1<<AESKR0)       

AESKR1       EQU 1       ; AES Key Register bit 1
BM_AESKR1    EQU (1<<AESKR1)       

AESKR2       EQU 2       ; AES Key Register bit 2
BM_AESKR2    EQU (1<<AESKR2)       

AESKR3       EQU 3       ; AES Key Register bit 3
BM_AESKR3    EQU (1<<AESKR3)       

AESKR4       EQU 4       ; AES Key Register bit 4
BM_AESKR4    EQU (1<<AESKR4)       

AESKR5       EQU 5       ; AES Key Register bit 5
BM_AESKR5    EQU (1<<AESKR5)       

AESKR6       EQU 6       ; AES Key Register bit 6
BM_AESKR6    EQU (1<<AESKR6)       

AESKR7       EQU 7       ; AES Key Register bit 7
BM_AESKR7    EQU (1<<AESKR7)       

; AESSR - AES Status Register
AESRF       EQU 0       ; AES Ready Flag
BM_AESRF    EQU (1<<AESRF)       

AESERF       EQU 7       ; AES Error Flag
BM_AESERF    EQU (1<<AESERF)       

; ***** CLK **************************
; CLKOCR - Clock output control Register
CLKOS0       EQU 0       ; Clock output source bit 0
BM_CLKOS0    EQU (1<<CLKOS0)       

CLKOS1       EQU 1       ; Clock output source bit 1
BM_CLKOS1    EQU (1<<CLKOS1)       

CLKOEN       EQU 2       ; Clock output driver enable
BM_CLKOEN    EQU (1<<CLKOEN)       

; CLKOD - Clock output divider settings Register
CLKOD0       EQU 0       ; Clock output divider bit 0
BM_CLKOD0    EQU (1<<CLKOD0)       

CLKOD1       EQU 1       ; Clock output divider bit 1
BM_CLKOD1    EQU (1<<CLKOD1)       

CLKOD2       EQU 2       ; Clock output divider bit 2
BM_CLKOD2    EQU (1<<CLKOD2)       

CLKOD3       EQU 3       ; Clock output divider bit 3
BM_CLKOD3    EQU (1<<CLKOD3)       

CLKOD4       EQU 4       ; Clock output divider bit 4
BM_CLKOD4    EQU (1<<CLKOD4)       

CLKOD5       EQU 5       ; Clock output divider bit 5
BM_CLKOD5    EQU (1<<CLKOD5)       

CLKOD6       EQU 6       ; Clock output divider bit 6
BM_CLKOD6    EQU (1<<CLKOD6)       

CLKOD7       EQU 7       ; Clock output divider bit 7
BM_CLKOD7    EQU (1<<CLKOD7)       

; CMOCR - Clock management override control register
FRCAO       EQU 0       ; FRC Always On
BM_FRCAO    EQU (1<<FRCAO)       

MRCAO       EQU 1       ; MRC Always On
BM_MRCAO    EQU (1<<MRCAO)       

FRCACT       EQU 2       ; FRC Active
BM_FRCACT    EQU (1<<FRCACT)       

; CMSR - Clock management status Register
ECF       EQU 0       ; External clock fail
BM_ECF    EQU (1<<ECF)       

; FRCCAL - Fast RC oscillator calibration Register
FRCCAL0       EQU 0       ; Fast RC oscillator calibration bit 0
BM_FRCCAL0    EQU (1<<FRCCAL0)       

FRCCAL1       EQU 1       ; Fast RC oscillator calibration bit 1
BM_FRCCAL1    EQU (1<<FRCCAL1)       

FRCCAL2       EQU 2       ; Fast RC oscillator calibration bit 2
BM_FRCCAL2    EQU (1<<FRCCAL2)       

FRCCAL3       EQU 3       ; Fast RC oscillator calibration bit 3
BM_FRCCAL3    EQU (1<<FRCCAL3)       

FRCCAL4       EQU 4       ; Fast RC oscillator calibration bit 4
BM_FRCCAL4    EQU (1<<FRCCAL4)       

; MRCCAL - Middle RC oscillator calibration Register
MRCCAL1       EQU 1       ; Middle RC oscillator calibration bit 1
BM_MRCCAL1    EQU (1<<MRCCAL1)       

MRCCAL2       EQU 2       ; Middle RC oscillator calibration bit 2
BM_MRCCAL2    EQU (1<<MRCCAL2)       

MRCCAL3       EQU 3       ; Middle RC oscillator calibration bit 3
BM_MRCCAL3    EQU (1<<MRCCAL3)       

MRCCAL4       EQU 4       ; Middle RC oscillator calibration bit 4
BM_MRCCAL4    EQU (1<<MRCCAL4)       

MRCCAL5       EQU 5       ; Middle RC oscillator calibration bit 5
BM_MRCCAL5    EQU (1<<MRCCAL5)       

MRCCAL6       EQU 6       ; Middle RC oscillator calibration bit 6
BM_MRCCAL6    EQU (1<<MRCCAL6)       

MRCCAL7       EQU 7       ; Middle RC oscillator calibration bit 7
BM_MRCCAL7    EQU (1<<MRCCAL7)       

; PRR0 - Power reduction Register 0
PRSPI       EQU 0       ; Power Reduction Serial Peripheral Interface
BM_PRSPI    EQU (1<<PRSPI)       

PRLFRS       EQU 1       ; Power Reduction LF RSSI
BM_PRLFRS    EQU (1<<PRLFRS)       

PRTXDC       EQU 2       ; Power Reduction Transmit DSP Control
BM_PRTXDC    EQU (1<<PRTXDC)       

PRCRC       EQU 3       ; Power Reduction CRC
BM_PRCRC    EQU (1<<PRCRC)       

PRVM       EQU 4       ; Power Reduction Voltage Monitor
BM_PRVM    EQU (1<<PRVM)       

PRCO       EQU 5       ; Power Reduction Clock Output
BM_PRCO    EQU (1<<PRCO)       

PRCU       EQU 6       ; Power Reduction Crypto Unit
BM_PRCU    EQU (1<<PRCU)       

PRTWI1       EQU 7       ; Power Reduction Two Wire Interface 1
BM_PRTWI1    EQU (1<<PRTWI1)       

; PRR1 - Power reduction Register 1
PRT1       EQU 0       ; Power Reduction Timer 1
BM_PRT1    EQU (1<<PRT1)       

PRT2       EQU 1       ; Power Reduction Timer 2
BM_PRT2    EQU (1<<PRT2)       

PRT3       EQU 2       ; Power Reduction Timer 3
BM_PRT3    EQU (1<<PRT3)       

PRT4       EQU 3       ; Power Reduction Timer 4
BM_PRT4    EQU (1<<PRT4)       

PRT5       EQU 4       ; Power Reduction Timer 5
BM_PRT5    EQU (1<<PRT5)       

PRLFR       EQU 5       ; Power Reduction LF Receiver
BM_PRLFR    EQU (1<<PRLFR)       

PRLFTP       EQU 6       ; Power Reduction LF Transponder
BM_PRLFTP    EQU (1<<PRLFTP)       

PRLFPH       EQU 7       ; Power Reduction LF Protocol Handler
BM_PRLFPH    EQU (1<<PRLFPH)       

; PRR2 - Power reduction register 2
PRSPI2       EQU 0       ; Power Reduction SPI2
BM_PRSPI2    EQU (1<<PRSPI2)       

PRTWI2       EQU 1       ; Power Reduction TWI2
BM_PRTWI2    EQU (1<<PRTWI2)       

PRSF       EQU 2       ; Power Reduction Support FIFO
BM_PRSF    EQU (1<<PRSF)       

PRDF       EQU 3       ; Power Reduction Data FIFO
BM_PRDF    EQU (1<<PRDF)       

PRR2_R4       EQU 4       ; 
BM_PRR2_R4    EQU (1<<PRR2_R4)       

PRR2_R5       EQU 5       ; 
BM_PRR2_R5    EQU (1<<PRR2_R5)       

PRTM       EQU 6       ; Power Reduction Tx Modulator
BM_PRTM    EQU (1<<PRTM)       

PRSSM       EQU 7       ; Power Reduction Sequencer State Machine
BM_PRSSM    EQU (1<<PRSSM)       

; XFUSE - 
; ***** CPU **************************
; CLPR - Clock Prescaler Register
CLKPS0       EQU 0       ; System Clock Prescaler Select bit 0
BM_CLKPS0    EQU (1<<CLKPS0)       

CLKPS1       EQU 1       ; System Clock Prescaler Select bit 1
BM_CLKPS1    EQU (1<<CLKPS1)       

CLKPS2       EQU 2       ; System Clock Prescaler Select bit 2
BM_CLKPS2    EQU (1<<CLKPS2)       

CLTPS0       EQU 3       ; Timer Clock Prescaler Select bit 0
BM_CLTPS0    EQU (1<<CLTPS0)       

CLTPS1       EQU 4       ; Timer Clock Prescaler Select bit 1
BM_CLTPS1    EQU (1<<CLTPS1)       

CLTPS2       EQU 5       ; Timer Clock Prescaler Select bit 2
BM_CLTPS2    EQU (1<<CLTPS2)       

CLPCE       EQU 7       ; Clock Prescaler Change Enable
BM_CLPCE    EQU (1<<CLPCE)       

; CMCR - Clock Management Control Register
CMM0       EQU 0       ; Clock Management Mode bit 0
BM_CMM0    EQU (1<<CMM0)       

CMM1       EQU 1       ; Clock Management Mode bit 1
BM_CMM1    EQU (1<<CMM1)       

CMM2       EQU 2       ; Clock Management Mode bit 2
BM_CMM2    EQU (1<<CMM2)       

CCS       EQU 3       ; Core Clock Select
BM_CCS    EQU (1<<CCS)       

CMONEN       EQU 6       ; Clock monitor enable
BM_CMONEN    EQU (1<<CMONEN)       

CMCCE       EQU 7       ; Clock management control change enable
BM_CMCCE    EQU (1<<CMCCE)       

; CMIMR - Clock interrupt mask Register
ECIE       EQU 0       ; External clock interrupt enable
BM_ECIE    EQU (1<<ECIE)       

; DWDR - Debug Wire Data Register
DWDR0       EQU 0       ; Debug Wire Data Register bit 0
BM_DWDR0    EQU (1<<DWDR0)       

DWDR1       EQU 1       ; Debug Wire Data Register bit 1
BM_DWDR1    EQU (1<<DWDR1)       

DWDR2       EQU 2       ; Debug Wire Data Register bit 2
BM_DWDR2    EQU (1<<DWDR2)       

DWDR3       EQU 3       ; Debug Wire Data Register bit 3
BM_DWDR3    EQU (1<<DWDR3)       

DWDR4       EQU 4       ; Debug Wire Data Register bit 4
BM_DWDR4    EQU (1<<DWDR4)       

DWDR5       EQU 5       ; Debug Wire Data Register bit 5
BM_DWDR5    EQU (1<<DWDR5)       

DWDR6       EQU 6       ; Debug Wire Data Register bit 6
BM_DWDR6    EQU (1<<DWDR6)       

DWDR7       EQU 7       ; Debug Wire Data Register bit 7
BM_DWDR7    EQU (1<<DWDR7)       

; MCUCR - MCU control Register
IVL0       EQU 0       ; Interrupt Vector Location bit 0
BM_IVL0    EQU (1<<IVL0)       

IVL1       EQU 1       ; Interrupt Vector Location bit 1
BM_IVL1    EQU (1<<IVL1)       

SPIIO       EQU 2       ; SPI Interrupt Only
BM_SPIIO    EQU (1<<SPIIO)       

ENPS       EQU 3       ; Enable Port Settings
BM_ENPS    EQU (1<<ENPS)       

PUD       EQU 4       ; Pull-up Resistors Disable
BM_PUD    EQU (1<<PUD)       

TRCCE       EQU 5       ; Trace Change Enable
BM_TRCCE    EQU (1<<TRCCE)       

TRCEN       EQU 6       ; Trace Enable
BM_TRCEN    EQU (1<<TRCEN)       

IVSEL       EQU 7       ; Interrupt Vector Select
BM_IVSEL    EQU (1<<IVSEL)       

; MCUSR - MCU Status Register
PORF       EQU 0       ; Power-On Reset Flag
BM_PORF    EQU (1<<PORF)       

EXTRF       EQU 1       ; External Reset Flag
BM_EXTRF    EQU (1<<EXTRF)       

MCUSR_R2       EQU 2       ; 
BM_MCUSR_R2    EQU (1<<MCUSR_R2)       

WDRF       EQU 3       ; Watchdog Reset Flag
BM_WDRF    EQU (1<<WDRF)       

TPRF       EQU 5       ; Transponder Reset Flag
BM_TPRF    EQU (1<<TPRF)       

MCUSR_R6       EQU 6       ; 
BM_MCUSR_R6    EQU (1<<MCUSR_R6)       

; SMCR - Sleep mode control Register
SE       EQU 0       ; Sleep Enable
BM_SE    EQU (1<<SE)       

SM0       EQU 1       ; Sleep Mode Select bit 0
BM_SM0    EQU (1<<SM0)       

SM1       EQU 2       ; Sleep Mode Select bit 1
BM_SM1    EQU (1<<SM1)       

SM2       EQU 3       ; Sleep Mode Select bit 2
BM_SM2    EQU (1<<SM2)       

; SPH - Stack Pointer High
SP8       EQU 0       ; Stack pointer bit 8
BM_SP8    EQU (1<<SP8)       

SP9       EQU 1       ; Stack pointer bit 9
BM_SP9    EQU (1<<SP9)       

SP10       EQU 2       ; Stack pointer bit 10
BM_SP10    EQU (1<<SP10)       

; SPL - Stack Pointer Low
SP0       EQU 0       ; Stack pointer bit 0
BM_SP0    EQU (1<<SP0)       

SP1       EQU 1       ; Stack pointer bit 1
BM_SP1    EQU (1<<SP1)       

SP2       EQU 2       ; Stack pointer bit 2
BM_SP2    EQU (1<<SP2)       

SP3       EQU 3       ; Stack pointer bit 3
BM_SP3    EQU (1<<SP3)       

SP4       EQU 4       ; Stack pointer bit 4
BM_SP4    EQU (1<<SP4)       

SP5       EQU 5       ; Stack pointer bit 5
BM_SP5    EQU (1<<SP5)       

SP6       EQU 6       ; Stack pointer bit 6
BM_SP6    EQU (1<<SP6)       

SP7       EQU 7       ; Stack pointer bit 7
BM_SP7    EQU (1<<SP7)       

; SPMCSR - Store Program Memory Control and Status Register
SELFPRGEN       EQU 0       ; Self Programming Enable
BM_SELFPRGEN    EQU (1<<SELFPRGEN)       

PGERS       EQU 1       ; Page Erase
BM_PGERS    EQU (1<<PGERS)       

PGWRT       EQU 2       ; Page Write
BM_PGWRT    EQU (1<<PGWRT)       

FLSEL0       EQU 3       ; Fuse Lockbit select Bit 0
BM_FLSEL0    EQU (1<<FLSEL0)       

FLSEL1       EQU 4       ; Fuse Lockbit select Bit 1
BM_FLSEL1    EQU (1<<FLSEL1)       

FLSEL2       EQU 5       ; Fuse Lockbit select Bit 2
BM_FLSEL2    EQU (1<<FLSEL2)       

RWWSB       EQU 6       ; Read-While-Write Section Busy
BM_RWWSB    EQU (1<<RWWSB)       

SPMIE       EQU 7       ; SPM Interrupt Enable
BM_SPMIE    EQU (1<<SPMIE)       

; SREG - Status Register
SREG_C       EQU 0       ; Carry Flag
BM_SREG_C    EQU (1<<SREG_C)       

SREG_Z       EQU 1       ; Zero Flag
BM_SREG_Z    EQU (1<<SREG_Z)       

SREG_N       EQU 2       ; Negative Flag
BM_SREG_N    EQU (1<<SREG_N)       

SREG_V       EQU 3       ; Two's Complement Overflow Flag
BM_SREG_V    EQU (1<<SREG_V)       

SREG_S       EQU 4       ; Sign Bit
BM_SREG_S    EQU (1<<SREG_S)       

SREG_H       EQU 5       ; Half Carry Flag
BM_SREG_H    EQU (1<<SREG_H)       

SREG_T       EQU 6       ; Bit Copy Storage
BM_SREG_T    EQU (1<<SREG_T)       

SREG_I       EQU 7       ; Global Interrupt Enable
BM_SREG_I    EQU (1<<SREG_I)       

; ***** CRC **************************
; CRCCR - CRC Control Register
CRCRS       EQU 0       ; CRC Data Register Reset
BM_CRCRS    EQU (1<<CRCRS)       

REFLI       EQU 1       ; Reflect Data Input Byte
BM_REFLI    EQU (1<<REFLI)       

REFLO       EQU 2       ; Reflect Data Output Byte
BM_REFLO    EQU (1<<REFLO)       

; CRCDIR - CRC Data Input Register
CRCDIR0       EQU 0       ; CRC Data Input Register bit 0
BM_CRCDIR0    EQU (1<<CRCDIR0)       

CRCDIR1       EQU 1       ; CRC Data Input Register bit 1
BM_CRCDIR1    EQU (1<<CRCDIR1)       

CRCDIR2       EQU 2       ; CRC Data Input Register bit 2
BM_CRCDIR2    EQU (1<<CRCDIR2)       

CRCDIR3       EQU 3       ; CRC Data Input Register bit 3
BM_CRCDIR3    EQU (1<<CRCDIR3)       

CRCDIR4       EQU 4       ; CRC Data Input Register bit 4
BM_CRCDIR4    EQU (1<<CRCDIR4)       

CRCDIR5       EQU 5       ; CRC Data Input Register bit 5
BM_CRCDIR5    EQU (1<<CRCDIR5)       

CRCDIR6       EQU 6       ; CRC Data Input Register bit 6
BM_CRCDIR6    EQU (1<<CRCDIR6)       

CRCDIR7       EQU 7       ; CRC Data Input Register bit 7
BM_CRCDIR7    EQU (1<<CRCDIR7)       

; CRCDOR - CRC Data Output Register
CRCDOR0       EQU 0       ; CRC Data Output Register bit 0
BM_CRCDOR0    EQU (1<<CRCDOR0)       

CRCDOR1       EQU 1       ; CRC Data Output Register bit 1
BM_CRCDOR1    EQU (1<<CRCDOR1)       

CRCDOR2       EQU 2       ; CRC Data Output Register bit 2
BM_CRCDOR2    EQU (1<<CRCDOR2)       

CRCDOR3       EQU 3       ; CRC Data Output Register bit 3
BM_CRCDOR3    EQU (1<<CRCDOR3)       

CRCDOR4       EQU 4       ; CRC Data Output Register bit 4
BM_CRCDOR4    EQU (1<<CRCDOR4)       

CRCDOR5       EQU 5       ; CRC Data Output Register bit 5
BM_CRCDOR5    EQU (1<<CRCDOR5)       

CRCDOR6       EQU 6       ; CRC Data Output Register bit 6
BM_CRCDOR6    EQU (1<<CRCDOR6)       

CRCDOR7       EQU 7       ; CRC Data Output Register bit 7
BM_CRCDOR7    EQU (1<<CRCDOR7)       

; ***** DEBOUNCE *********************
; DBCR - DeBounce Control Register
DBMD       EQU 0       ; Debounce Mode
BM_DBMD    EQU (1<<DBMD)       

DBCS       EQU 1       ; Debounce Clock Select
BM_DBCS    EQU (1<<DBCS)       

DBTMS       EQU 2       ; Debounce Timer Mask Select
BM_DBTMS    EQU (1<<DBTMS)       

DBHA       EQU 3       ; Debounce Handshake Active
BM_DBHA    EQU (1<<DBHA)       

; DBENB - DeBounce Enable Port B
DBENB0       EQU 0       ; DeBounce Enable Port B Pin 0
BM_DBENB0    EQU (1<<DBENB0)       

DBENB1       EQU 1       ; DeBounce Enable Port B Pin 1
BM_DBENB1    EQU (1<<DBENB1)       

DBENB2       EQU 2       ; DeBounce Enable Port B Pin 2
BM_DBENB2    EQU (1<<DBENB2)       

DBENB3       EQU 3       ; DeBounce Enable Port B Pin 3
BM_DBENB3    EQU (1<<DBENB3)       

DBENB4       EQU 4       ; DeBounce Enable Port B Pin 4
BM_DBENB4    EQU (1<<DBENB4)       

DBENB5       EQU 5       ; DeBounce Enable Port B Pin 5
BM_DBENB5    EQU (1<<DBENB5)       

DBENB6       EQU 6       ; DeBounce Enable Port B Pin 6
BM_DBENB6    EQU (1<<DBENB6)       

DBENB7       EQU 7       ; DeBounce Enable Port B Pin 7
BM_DBENB7    EQU (1<<DBENB7)       

; DBENC - DeBounce Enable Port C
DBENC0       EQU 0       ; DeBounce Enable Port C Pin 0
BM_DBENC0    EQU (1<<DBENC0)       

DBENC1       EQU 1       ; DeBounce Enable Port C Pin 1
BM_DBENC1    EQU (1<<DBENC1)       

DBENC2       EQU 2       ; DeBounce Enable Port C Pin 2
BM_DBENC2    EQU (1<<DBENC2)       

; DBEND - DeBounce Enable Port D
DBEND0       EQU 0       ; DeBounce Enable Port D Pin 0
BM_DBEND0    EQU (1<<DBEND0)       

DBEND1       EQU 1       ; DeBounce Enable Port D Pin 1
BM_DBEND1    EQU (1<<DBEND1)       

DBEND2       EQU 2       ; DeBounce Enable Port D Pin 2
BM_DBEND2    EQU (1<<DBEND2)       

DBEND3       EQU 3       ; DeBounce Enable Port D Pin 3
BM_DBEND3    EQU (1<<DBEND3)       

DBEND4       EQU 4       ; DeBounce Enable Port D Pin 4
BM_DBEND4    EQU (1<<DBEND4)       

DBEND5       EQU 5       ; DeBounce Enable Port D Pin 5
BM_DBEND5    EQU (1<<DBEND5)       

DBEND6       EQU 6       ; DeBounce Enable Port D Pin 6
BM_DBEND6    EQU (1<<DBEND6)       

DBEND7       EQU 7       ; DeBounce Enable Port D Pin 7
BM_DBEND7    EQU (1<<DBEND7)       

; DBTC - Debounce Timer Compare Register
DBTC0       EQU 0       ; Debounce Timer Compare bit 0
BM_DBTC0    EQU (1<<DBTC0)       

DBTC1       EQU 1       ; Debounce Timer Compare bit 1
BM_DBTC1    EQU (1<<DBTC1)       

DBTC2       EQU 2       ; Debounce Timer Compare bit 2
BM_DBTC2    EQU (1<<DBTC2)       

DBTC3       EQU 3       ; Debounce Timer Compare bit 3
BM_DBTC3    EQU (1<<DBTC3)       

DBTC4       EQU 4       ; Debounce Timer Compare bit 4
BM_DBTC4    EQU (1<<DBTC4)       

DBTC5       EQU 5       ; Debounce Timer Compare bit 5
BM_DBTC5    EQU (1<<DBTC5)       

DBTC6       EQU 6       ; Debounce Timer Compare bit 6
BM_DBTC6    EQU (1<<DBTC6)       

DBTC7       EQU 7       ; Debounce Timer Compare bit 7
BM_DBTC7    EQU (1<<DBTC7)       

; ***** DEBUG ************************
; DBGSW - Debugging Support Switch
DBGGS0       EQU 0       ; Debugging Support Group Select bit 0
BM_DBGGS0    EQU (1<<DBGGS0)       

DBGGS1       EQU 1       ; Debugging Support Group Select bit 1
BM_DBGGS1    EQU (1<<DBGGS1)       

DBGGS2       EQU 2       ; Debugging Support Group Select bit 2
BM_DBGGS2    EQU (1<<DBGGS2)       

DBGGS3       EQU 3       ; Debugging Support Group Select bit 3
BM_DBGGS3    EQU (1<<DBGGS3)       

CPBFOS0       EQU 4       ; CPU Busy Flag Output Select bit 0
BM_CPBFOS0    EQU (1<<CPBFOS0)       

CPBFOS1       EQU 5       ; CPU Busy Flag Output Select bit 1
BM_CPBFOS1    EQU (1<<CPBFOS1)       

CPBF       EQU 6       ; CPU Busy Flag
BM_CPBF    EQU (1<<CPBF)       

ATEST       EQU 7       ; Analog Test Pins Enable
BM_ATEST    EQU (1<<ATEST)       

; DBONDR - Downbond Test Register
BBESD       EQU 0       ; Bond BBESD
BM_BBESD    EQU (1<<BBESD)       

AGND_BB       EQU 1       ; Bond AGND BB
BM_AGND_BB    EQU (1<<AGND_BB)       

ISO_GND       EQU 2       ; Bond ISO SUBS3 GND
BM_ISO_GND    EQU (1<<ISO_GND)       

AGND_LF       EQU 3       ; Bond AGND LF
BM_AGND_LF    EQU (1<<AGND_LF)       

BTEST4       EQU 4       ; Bond Test 4
BM_BTEST4    EQU (1<<BTEST4)       

BTEST5       EQU 5       ; Bond Test 5
BM_BTEST5    EQU (1<<BTEST5)       

BTEST6       EQU 6       ; Bond Test 6
BM_BTEST6    EQU (1<<BTEST6)       

; TRCDR - Trace Data Register
TRCDR0       EQU 0       ; 
BM_TRCDR0    EQU (1<<TRCDR0)       

TRCDR1       EQU 1       ; 
BM_TRCDR1    EQU (1<<TRCDR1)       

TRCDR2       EQU 2       ; 
BM_TRCDR2    EQU (1<<TRCDR2)       

TRCDR3       EQU 3       ; 
BM_TRCDR3    EQU (1<<TRCDR3)       

TRCDR4       EQU 4       ; 
BM_TRCDR4    EQU (1<<TRCDR4)       

TRCDR5       EQU 5       ; 
BM_TRCDR5    EQU (1<<TRCDR5)       

TRCDR6       EQU 6       ; 
BM_TRCDR6    EQU (1<<TRCDR6)       

TRCDR7       EQU 7       ; 
BM_TRCDR7    EQU (1<<TRCDR7)       

; TRCIDH - Trace ID Register High byte
TRCID8       EQU 0       ; 
BM_TRCID8    EQU (1<<TRCID8)       

TRCID9       EQU 1       ; 
BM_TRCID9    EQU (1<<TRCID9)       

TRCID10       EQU 2       ; 
BM_TRCID10    EQU (1<<TRCID10)       

TRCID11       EQU 3       ; 
BM_TRCID11    EQU (1<<TRCID11)       

TRCID12       EQU 4       ; 
BM_TRCID12    EQU (1<<TRCID12)       

TRCID13       EQU 5       ; 
BM_TRCID13    EQU (1<<TRCID13)       

TRCID14       EQU 6       ; 
BM_TRCID14    EQU (1<<TRCID14)       

TRCID15       EQU 7       ; 
BM_TRCID15    EQU (1<<TRCID15)       

; TRCIDL - Trace ID Register Low byte
TRCID0       EQU 0       ; 
BM_TRCID0    EQU (1<<TRCID0)       

TRCID1       EQU 1       ; 
BM_TRCID1    EQU (1<<TRCID1)       

TRCID2       EQU 2       ; 
BM_TRCID2    EQU (1<<TRCID2)       

TRCID3       EQU 3       ; 
BM_TRCID3    EQU (1<<TRCID3)       

TRCID4       EQU 4       ; 
BM_TRCID4    EQU (1<<TRCID4)       

TRCID5       EQU 5       ; 
BM_TRCID5    EQU (1<<TRCID5)       

TRCID6       EQU 6       ; 
BM_TRCID6    EQU (1<<TRCID6)       

TRCID7       EQU 7       ; 
BM_TRCID7    EQU (1<<TRCID7)       

; ***** DFIFO ************************
; DFC - Data FIFO Configuration Register
DFFLC0       EQU 0       ; Data FIFO Fill-Level Configuration bit 0
BM_DFFLC0    EQU (1<<DFFLC0)       

DFFLC1       EQU 1       ; Data FIFO Fill-Level Configuration bit 1
BM_DFFLC1    EQU (1<<DFFLC1)       

DFFLC2       EQU 2       ; Data FIFO Fill-Level Configuration bit 2
BM_DFFLC2    EQU (1<<DFFLC2)       

DFFLC3       EQU 3       ; Data FIFO Fill-Level Configuration bit 3
BM_DFFLC3    EQU (1<<DFFLC3)       

DFFLC4       EQU 4       ; Data FIFO Fill-Level Configuration bit 4
BM_DFFLC4    EQU (1<<DFFLC4)       

DFFLC5       EQU 5       ; Data FIFO Fill-Level Configuration bit 5
BM_DFFLC5    EQU (1<<DFFLC5)       

DFDRA       EQU 7       ; Data FIFO Direct Read Access Operational Mode
BM_DFDRA    EQU (1<<DFDRA)       

; DFI - Data FIFO Interrupt Mask Register
DFFLIM       EQU 0       ; Data FIFO Fill-level Interrupt Mask
BM_DFFLIM    EQU (1<<DFFLIM)       

DFERIM       EQU 1       ; Data FIFO Error Interrupt Mask
BM_DFERIM    EQU (1<<DFERIM)       

; DFL - Data FIFO Fill Level Register
DFFLS0       EQU 0       ; Data FIFO Fill Level Status bit 0
BM_DFFLS0    EQU (1<<DFFLS0)       

DFFLS1       EQU 1       ; Data FIFO Fill Level Status bit 1
BM_DFFLS1    EQU (1<<DFFLS1)       

DFFLS2       EQU 2       ; Data FIFO Fill Level Status bit 2
BM_DFFLS2    EQU (1<<DFFLS2)       

DFFLS3       EQU 3       ; Data FIFO Fill Level Status bit 3
BM_DFFLS3    EQU (1<<DFFLS3)       

DFFLS4       EQU 4       ; Data FIFO Fill Level Status bit 4
BM_DFFLS4    EQU (1<<DFFLS4)       

DFFLS5       EQU 5       ; Data FIFO Fill Level Status bit 5
BM_DFFLS5    EQU (1<<DFFLS5)       

DFCLR       EQU 7       ; Data FIFO Clear
BM_DFCLR    EQU (1<<DFCLR)       

; DFRP - Data FIFO Read Pointer
DFRP0       EQU 0       ; Data FIFO Read Pointer bit 0
BM_DFRP0    EQU (1<<DFRP0)       

DFRP1       EQU 1       ; Data FIFO Read Pointer bit 1
BM_DFRP1    EQU (1<<DFRP1)       

DFRP2       EQU 2       ; Data FIFO Read Pointer bit 2
BM_DFRP2    EQU (1<<DFRP2)       

DFRP3       EQU 3       ; Data FIFO Read Pointer bit 3
BM_DFRP3    EQU (1<<DFRP3)       

DFRP4       EQU 4       ; Data FIFO Read Pointer bit 4
BM_DFRP4    EQU (1<<DFRP4)       

DFRP5       EQU 5       ; Data FIFO Read Pointer bit 5
BM_DFRP5    EQU (1<<DFRP5)       

; DFS - Data FIFO Status Register
DFFLRF       EQU 0       ; Data FIFO Fill-Level Reached Status Flag
BM_DFFLRF    EQU (1<<DFFLRF)       

DFUFL       EQU 1       ; Data FIFO Underflow Flag
BM_DFUFL    EQU (1<<DFUFL)       

DFOFL       EQU 2       ; Data FIFO Overflow Flag
BM_DFOFL    EQU (1<<DFOFL)       

; DFWP - Data FIFO Write Pointer
DFWP0       EQU 0       ; Data FIFO Write Pointer bit 0
BM_DFWP0    EQU (1<<DFWP0)       

DFWP1       EQU 1       ; Data FIFO Write Pointer bit 1
BM_DFWP1    EQU (1<<DFWP1)       

DFWP2       EQU 2       ; Data FIFO Write Pointer bit 2
BM_DFWP2    EQU (1<<DFWP2)       

DFWP3       EQU 3       ; Data FIFO Write Pointer bit 3
BM_DFWP3    EQU (1<<DFWP3)       

DFWP4       EQU 4       ; Data FIFO Write Pointer bit 4
BM_DFWP4    EQU (1<<DFWP4)       

DFWP5       EQU 5       ; Data FIFO Write Pointer bit 5
BM_DFWP5    EQU (1<<DFWP5)       

; ***** EEPROM ***********************
; EEARH - EEPROM Address Register High Byte
EEAR8       EQU 0       ; EEPROM Read/Write Address bit 8
BM_EEAR8    EQU (1<<EEAR8)       

EEAR9       EQU 1       ; EEPROM Read/Write Address bit 9
BM_EEAR9    EQU (1<<EEAR9)       

EEAR10       EQU 2       ; EEPROM Read/Write Address bit 10
BM_EEAR10    EQU (1<<EEAR10)       

EEAR11       EQU 3       ; EEPROM Read/Write Address bit 11
BM_EEAR11    EQU (1<<EEAR11)       

; EEARL - EEPROM Address Register Low Byte
EEAR0       EQU 0       ; EEPROM Read/Write Address bit 0
BM_EEAR0    EQU (1<<EEAR0)       

EEAR1       EQU 1       ; EEPROM Read/Write Address bit 1
BM_EEAR1    EQU (1<<EEAR1)       

EEAR2       EQU 2       ; EEPROM Read/Write Address bit 2
BM_EEAR2    EQU (1<<EEAR2)       

EEAR3       EQU 3       ; EEPROM Read/Write Address bit 3
BM_EEAR3    EQU (1<<EEAR3)       

EEAR4       EQU 4       ; EEPROM Read/Write Address bit 4
BM_EEAR4    EQU (1<<EEAR4)       

EEAR5       EQU 5       ; EEPROM Read/Write Address bit 5
BM_EEAR5    EQU (1<<EEAR5)       

EEAR6       EQU 6       ; EEPROM Read/Write Address bit 6
BM_EEAR6    EQU (1<<EEAR6)       

EEAR7       EQU 7       ; EEPROM Read/Write Address bit 7
BM_EEAR7    EQU (1<<EEAR7)       

; EECR - EEPROM Control Register
EERE       EQU 0       ; EEPROM Read Enable
BM_EERE    EQU (1<<EERE)       

EEWE       EQU 1       ; EEPROM Write Enable
BM_EEWE    EQU (1<<EEWE)       

EEMWE       EQU 2       ; EEPROM Master Write Enable
BM_EEMWE    EQU (1<<EEMWE)       

EERIE       EQU 3       ; EEPROM Ready Interrupt Enable
BM_EERIE    EQU (1<<EERIE)       

EEPM0       EQU 4       ; EEPROM Programming Mode bit 0
BM_EEPM0    EQU (1<<EEPM0)       

EEPM1       EQU 5       ; EEPROM Programming Mode bit 1
BM_EEPM1    EQU (1<<EEPM1)       

EEPAGE       EQU 6       ; EEPROM page access (multiple bytes access mode)
BM_EEPAGE    EQU (1<<EEPAGE)       

NVMBSY       EQU 7       ; Non-volatile memory busy
BM_NVMBSY    EQU (1<<NVMBSY)       

; EECR2 - EEPROM Control Register 2
EEBRE       EQU 0       ; EEPROM Burst Read Enable
BM_EEBRE    EQU (1<<EEBRE)       

E2CIM       EQU 1       ; EEPROM Error Code Correction Interrupt Mask
BM_E2CIM    EQU (1<<E2CIM)       

E2AVF       EQU 5       ; EEPROM Access Violation Flag
BM_E2AVF    EQU (1<<E2AVF)       

E2FF       EQU 6       ; EEPROM error Fault Flag bit
BM_E2FF    EQU (1<<E2FF)       

E2CF       EQU 7       ; EEPROM Error Correction Code Flag
BM_E2CF    EQU (1<<E2CF)       

; EEPR0 - EEPROM Protection Register 0
EEPS0WD       EQU 0       ; EEPROM Protection Section 0 Write Disable
BM_EEPS0WD    EQU (1<<EEPS0WD)       

EEPS0RD       EQU 1       ; EEPROM Protection Section 0 Read Disable
BM_EEPS0RD    EQU (1<<EEPS0RD)       

EEPS1WD       EQU 2       ; EEPROM Protection Section 1 Write Disable
BM_EEPS1WD    EQU (1<<EEPS1WD)       

EEPS1RD       EQU 3       ; EEPROM Protection Section 1 Read Disable
BM_EEPS1RD    EQU (1<<EEPS1RD)       

EEPS2WD       EQU 4       ; EEPROM Protection Section 2 Write Disable
BM_EEPS2WD    EQU (1<<EEPS2WD)       

EEPS2RD       EQU 5       ; EEPROM Protection Section 2 Read Disable
BM_EEPS2RD    EQU (1<<EEPS2RD)       

EEPS3WD       EQU 6       ; EEPROM Protection Section 3 Write Disable
BM_EEPS3WD    EQU (1<<EEPS3WD)       

EEPS3RD       EQU 7       ; EEPROM Protection Section 3 Read Disable
BM_EEPS3RD    EQU (1<<EEPS3RD)       

; EEPR1 - EEPROM Protection Register 1
EEPS4WD       EQU 0       ; EEPROM Protection Section 4 Write Disable
BM_EEPS4WD    EQU (1<<EEPS4WD)       

EEPS4RD       EQU 1       ; EEPROM Protection Section 4 Read Disable
BM_EEPS4RD    EQU (1<<EEPS4RD)       

EEPS5WD       EQU 2       ; EEPROM Protection Section 5 Write Disable
BM_EEPS5WD    EQU (1<<EEPS5WD)       

EEPS5RD       EQU 3       ; EEPROM Protection Section 5 Read Disable
BM_EEPS5RD    EQU (1<<EEPS5RD)       

EEPS6WD       EQU 4       ; EEPROM Protection Section 6 Write Disable
BM_EEPS6WD    EQU (1<<EEPS6WD)       

EEPS6RD       EQU 5       ; EEPROM Protection Section 6 Read Disable
BM_EEPS6RD    EQU (1<<EEPS6RD)       

EEPS7WD       EQU 6       ; EEPROM Protection Section 7 Write Disable
BM_EEPS7WD    EQU (1<<EEPS7WD)       

EEPS7RD       EQU 7       ; EEPROM Protection Section 7 Read Disable
BM_EEPS7RD    EQU (1<<EEPS7RD)       

; EEPR2 - EEPROM Protection Register 2
EEPS8WD       EQU 0       ; EEPROM Protection Section 8 Write Disable
BM_EEPS8WD    EQU (1<<EEPS8WD)       

EEPS8RD       EQU 1       ; EEPROM Protection Section 8 Read Disable
BM_EEPS8RD    EQU (1<<EEPS8RD)       

EEPS9WD       EQU 2       ; EEPROM Protection Section 9 Write Disable
BM_EEPS9WD    EQU (1<<EEPS9WD)       

EEPS9RD       EQU 3       ; EEPROM Protection Section 9 Read Disable
BM_EEPS9RD    EQU (1<<EEPS9RD)       

EEPS10WD       EQU 4       ; EEPROM Protection Section 10 Write Disableble
BM_EEPS10WD    EQU (1<<EEPS10WD)       

EEPS10RD       EQU 5       ; EEPROM Protection Section 10 Read Disable
BM_EEPS10RD    EQU (1<<EEPS10RD)       

EEPS11WD       EQU 6       ; EEPROM Protection Section 11 Write Disable
BM_EEPS11WD    EQU (1<<EEPS11WD)       

EEPS11RD       EQU 7       ; EEPROM Protection Section 11 Read Disable
BM_EEPS11RD    EQU (1<<EEPS11RD)       

; EEPR3 - EEPROM Protection Register 3
EEPS12WD       EQU 0       ; EEPROM Protection Section 12 Write Disable
BM_EEPS12WD    EQU (1<<EEPS12WD)       

EEPS12RD       EQU 1       ; EEPROM Protection Section 12 Read Disable
BM_EEPS12RD    EQU (1<<EEPS12RD)       

; ***** FE ***************************
; FEALR - Front-End Antenna Level Detector Range
RNGE0       EQU 0       ; Range of the ANT_TUNE level detector bit 0
BM_RNGE0    EQU (1<<RNGE0)       

RNGE1       EQU 1       ; Range of the ANT_TUNE level detector bit 1
BM_RNGE1    EQU (1<<RNGE1)       

; FEANT - Front-End Antenna
LVLC0       EQU 0       ; antenna signal LeVeL bit 0
BM_LVLC0    EQU (1<<LVLC0)       

LVLC1       EQU 1       ; antenna signal LeVeL bit 1
BM_LVLC1    EQU (1<<LVLC1)       

LVLC2       EQU 2       ; antenna signal LeVeL bit 2
BM_LVLC2    EQU (1<<LVLC2)       

LVLC3       EQU 3       ; antenna signal LeVeL bit 3
BM_LVLC3    EQU (1<<LVLC3)       

; FEAT - Front-End Antenna Tuning
ANTN0       EQU 0       ; Antenna Tuning bit 0
BM_ANTN0    EQU (1<<ANTN0)       

ANTN1       EQU 1       ; Antenna Tuning bit 1
BM_ANTN1    EQU (1<<ANTN1)       

ANTN2       EQU 2       ; Antenna Tuning bit 2
BM_ANTN2    EQU (1<<ANTN2)       

ANTN3       EQU 3       ; Antenna Tuning bit 3
BM_ANTN3    EQU (1<<ANTN3)       

; FEBIA - Reserved
FEBIA_R0       EQU 0       ; 
BM_FEBIA_R0    EQU (1<<FEBIA_R0)       

FEBIA_R1       EQU 1       ; 
BM_FEBIA_R1    EQU (1<<FEBIA_R1)       

FEBIA_R2       EQU 2       ; 
BM_FEBIA_R2    EQU (1<<FEBIA_R2)       

FEBIA_R3       EQU 3       ; 
BM_FEBIA_R3    EQU (1<<FEBIA_R3)       

FEBIA_R4       EQU 4       ; 
BM_FEBIA_R4    EQU (1<<FEBIA_R4)       

FEBIA_R5       EQU 5       ; 
BM_FEBIA_R5    EQU (1<<FEBIA_R5)       

FEBIA_R6       EQU 6       ; 
BM_FEBIA_R6    EQU (1<<FEBIA_R6)       

FEBIA_R7       EQU 7       ; 
BM_FEBIA_R7    EQU (1<<FEBIA_R7)       

; FEBT - Front-End RC Tuning Register
CTN20       EQU 0       ; Capacitor Tuning bit 0
BM_CTN20    EQU (1<<CTN20)       

CTN21       EQU 1       ; Capacitor Tuning bit 1
BM_CTN21    EQU (1<<CTN21)       

RTN20       EQU 2       ; Resistor Tuning bit 0
BM_RTN20    EQU (1<<RTN20)       

RTN21       EQU 3       ; Resistor Tuning bit 1
BM_RTN21    EQU (1<<RTN21)       

; FECR - Front-End Control Register
LBNHB       EQU 0       ; Select Low-Band Not High-Band
BM_LBNHB    EQU (1<<LBNHB)       

S4N3       EQU 1       ; Select 433 Not 315MHz Band
BM_S4N3    EQU (1<<S4N3)       

FECR_R2       EQU 2       ; 
BM_FECR_R2    EQU (1<<FECR_R2)       

FECR_R3       EQU 3       ; 
BM_FECR_R3    EQU (1<<FECR_R3)       

PLCKG       EQU 4       ; PLL Lock Detect Gate
BM_PLCKG    EQU (1<<PLCKG)       

ANPS       EQU 5       ; ASK Not DPSK Switch
BM_ANPS    EQU (1<<ANPS)       

; FEEN1 - Front-End Enable Register 1
PLEN       EQU 0       ; PLL enable
BM_PLEN    EQU (1<<PLEN)       

PLCAL       EQU 1       ; PLL calibration mode
BM_PLCAL    EQU (1<<PLCAL)       

XTOEN       EQU 2       ; Cristal Oscillator enable
BM_XTOEN    EQU (1<<XTOEN)       

FEEN1_R3       EQU 3       ; 
BM_FEEN1_R3    EQU (1<<FEEN1_R3)       

FEEN1_R4       EQU 4       ; 
BM_FEEN1_R4    EQU (1<<FEEN1_R4)       

FEEN1_R5       EQU 5       ; 
BM_FEEN1_R5    EQU (1<<FEEN1_R5)       

PLSP1       EQU 6       ; PLL Speedup 1
BM_PLSP1    EQU (1<<PLSP1)       

ATEN       EQU 7       ; Antenna Tuning Enable
BM_ATEN    EQU (1<<ATEN)       

; FEEN2 - Front-End Enable Register 2
FEEN2_R0       EQU 0       ; 
BM_FEEN2_R0    EQU (1<<FEEN2_R0)       

FEEN2_R1       EQU 1       ; 
BM_FEEN2_R1    EQU (1<<FEEN2_R1)       

PAEN       EQU 2       ; Power Amplifier enable
BM_PAEN    EQU (1<<PAEN)       

FEEN2_R3       EQU 3       ; 
BM_FEEN2_R3    EQU (1<<FEEN2_R3)       

PLPEN       EQU 4       ; PLL Post En IQ divider
BM_PLPEN    EQU (1<<PLPEN)       

FEEN2_R5       EQU 5       ; 
BM_FEEN2_R5    EQU (1<<FEEN2_R5)       

CPBIA       EQU 6       ; Cap Array Bias
BM_CPBIA    EQU (1<<CPBIA)       

FEEN2_R7       EQU 7       ; 
BM_FEEN2_R7    EQU (1<<FEEN2_R7)       

; FELNA - Reserved
FELNA_R0       EQU 0       ; 
BM_FELNA_R0    EQU (1<<FELNA_R0)       

FELNA_R1       EQU 1       ; 
BM_FELNA_R1    EQU (1<<FELNA_R1)       

FELNA_R2       EQU 2       ; 
BM_FELNA_R2    EQU (1<<FELNA_R2)       

FELNA_R3       EQU 3       ; 
BM_FELNA_R3    EQU (1<<FELNA_R3)       

FELNA_R4       EQU 4       ; 
BM_FELNA_R4    EQU (1<<FELNA_R4)       

FELNA_R5       EQU 5       ; 
BM_FELNA_R5    EQU (1<<FELNA_R5)       

FELNA_R6       EQU 6       ; 
BM_FELNA_R6    EQU (1<<FELNA_R6)       

FELNA_R7       EQU 7       ; 
BM_FELNA_R7    EQU (1<<FELNA_R7)       

; FEMS - Front-End Main and Swallow Control Register
PLLS0       EQU 0       ; PLL Swallow bit 0
BM_PLLS0    EQU (1<<PLLS0)       

PLLS1       EQU 1       ; PLL Swallow bit 1
BM_PLLS1    EQU (1<<PLLS1)       

PLLS2       EQU 2       ; PLL Swallow bit 2
BM_PLLS2    EQU (1<<PLLS2)       

PLLS3       EQU 3       ; PLL Swallow bit 3
BM_PLLS3    EQU (1<<PLLS3)       

PLLM0       EQU 4       ; PLL Mode bit 0
BM_PLLM0    EQU (1<<PLLM0)       

PLLM1       EQU 5       ; PLL Mode bit 1
BM_PLLM1    EQU (1<<PLLM1)       

PLLM2       EQU 6       ; PLL Mode bit 2
BM_PLLM2    EQU (1<<PLLM2)       

PLLM3       EQU 7       ; PLL Mode bit 3
BM_PLLM3    EQU (1<<PLLM3)       

; FEPAC - Front-End Power Amplifier Control Register
PACR0       EQU 0       ; Power Amplifier Control Register bit 0
BM_PACR0    EQU (1<<PACR0)       

PACR1       EQU 1       ; Power Amplifier Control Register bit 1
BM_PACR1    EQU (1<<PACR1)       

PACR2       EQU 2       ; Power Amplifier Control Register bit 2
BM_PACR2    EQU (1<<PACR2)       

PACR3       EQU 3       ; Power Amplifier Control Register bit 3
BM_PACR3    EQU (1<<PACR3)       

PACR4       EQU 4       ; Power Amplifier Control Register bit 4
BM_PACR4    EQU (1<<PACR4)       

PACR5       EQU 5       ; Power Amplifier Control Register bit 5
BM_PACR5    EQU (1<<PACR5)       

PACR6       EQU 6       ; Power Amplifier Control Register bit 6
BM_PACR6    EQU (1<<PACR6)       

PACR7       EQU 7       ; Power Amplifier Control Register bit 7
BM_PACR7    EQU (1<<PACR7)       

; FESR - Front-End Status Register
XRDY       EQU 2       ; XTO ready
BM_XRDY    EQU (1<<XRDY)       

PLCK       EQU 3       ; PLL locked
BM_PLCK    EQU (1<<PLCK)       

ANTS       EQU 4       ; Antenna saturated
BM_ANTS    EQU (1<<ANTS)       

; FETD - Front-End Test Data Register
FETM0       EQU 0       ; 
BM_FETM0    EQU (1<<FETM0)       

FETM1       EQU 1       ; 
BM_FETM1    EQU (1<<FETM1)       

FETM2       EQU 2       ; 
BM_FETM2    EQU (1<<FETM2)       

FETM3       EQU 3       ; 
BM_FETM3    EQU (1<<FETM3)       

FETM4       EQU 4       ; 
BM_FETM4    EQU (1<<FETM4)       

FETM5       EQU 5       ; 
BM_FETM5    EQU (1<<FETM5)       

FETM6       EQU 6       ; 
BM_FETM6    EQU (1<<FETM6)       

FETM7       EQU 7       ; 
BM_FETM7    EQU (1<<FETM7)       

; FETE1 - Front-End Test Enable Register 1
ADCT       EQU 0       ; 
BM_ADCT    EQU (1<<ADCT)       

XTOT       EQU 1       ; 
BM_XTOT    EQU (1<<XTOT)       

LNLT       EQU 2       ; 
BM_LNLT    EQU (1<<LNLT)       

LNHT       EQU 3       ; 
BM_LNHT    EQU (1<<LNHT)       

PATE       EQU 4       ; 
BM_PATE    EQU (1<<PATE)       

AMPT       EQU 5       ; 
BM_AMPT    EQU (1<<AMPT)       

VCOT       EQU 6       ; 
BM_VCOT    EQU (1<<VCOT)       

ANTN       EQU 7       ; 
BM_ANTN    EQU (1<<ANTN)       

; FETE2 - Front-End Test Enable Register 2
RCCT       EQU 0       ; 
BM_RCCT    EQU (1<<RCCT)       

PPFT       EQU 1       ; 
BM_PPFT    EQU (1<<PPFT)       

LFT       EQU 2       ; 
BM_LFT    EQU (1<<LFT)       

CPT       EQU 3       ; 
BM_CPT    EQU (1<<CPT)       

PFDT       EQU 4       ; 
BM_PFDT    EQU (1<<PFDT)       

DADCT       EQU 5       ; 
BM_DADCT    EQU (1<<DADCT)       

PRET       EQU 6       ; 
BM_PRET    EQU (1<<PRET)       

SWALT       EQU 7       ; 
BM_SWALT    EQU (1<<SWALT)       

; FETE3 - Front-End Test Enable Register 3
BIOUT       EQU 0       ; 
BM_BIOUT    EQU (1<<BIOUT)       

RMPTST       EQU 1       ; 
BM_RMPTST    EQU (1<<RMPTST)       

FETE3_R2       EQU 2       ; 
BM_FETE3_R2    EQU (1<<FETE3_R2)       

FETE3_R3       EQU 3       ; 
BM_FETE3_R3    EQU (1<<FETE3_R3)       

FETE3_R4       EQU 4       ; 
BM_FETE3_R4    EQU (1<<FETE3_R4)       

FETE3_R5       EQU 5       ; 
BM_FETE3_R5    EQU (1<<FETE3_R5)       

FETE3_R6       EQU 6       ; 
BM_FETE3_R6    EQU (1<<FETE3_R6)       

FETE3_R7       EQU 7       ; 
BM_FETE3_R7    EQU (1<<FETE3_R7)       

; FETN4 - Front-End RC Tuning 4bit Register
CTN40       EQU 0       ; 4 bit Capacitor Tuning bit 0
BM_CTN40    EQU (1<<CTN40)       

CTN41       EQU 1       ; 4 bit Capacitor Tuning bit 1
BM_CTN41    EQU (1<<CTN41)       

CTN42       EQU 2       ; 4 bit Capacitor Tuning bit 2
BM_CTN42    EQU (1<<CTN42)       

CTN43       EQU 3       ; 4 bit Capacitor Tuning bit 3
BM_CTN43    EQU (1<<CTN43)       

RTN40       EQU 4       ; 4 bit Resistor Tuning bit 0
BM_RTN40    EQU (1<<RTN40)       

RTN41       EQU 5       ; 4 bit Resistor Tuning bit 1
BM_RTN41    EQU (1<<RTN41)       

RTN42       EQU 6       ; 4 bit Resistor Tuning bit 2
BM_RTN42    EQU (1<<RTN42)       

RTN43       EQU 7       ; 4 bit Resistor Tuning bit 3
BM_RTN43    EQU (1<<RTN43)       

; FEVCO - Front-End VCO and PLL control
CPCC0       EQU 0       ; Charge pump current control bit 0
BM_CPCC0    EQU (1<<CPCC0)       

CPCC1       EQU 1       ; Charge pump current control bit 1
BM_CPCC1    EQU (1<<CPCC1)       

CPCC2       EQU 2       ; Charge pump current control bit 2
BM_CPCC2    EQU (1<<CPCC2)       

CPCC3       EQU 3       ; Charge pump current control bit 3
BM_CPCC3    EQU (1<<CPCC3)       

VCOB0       EQU 4       ; VCO Bias bit 0
BM_VCOB0    EQU (1<<VCOB0)       

VCOB1       EQU 5       ; VCO Bias bit 1
BM_VCOB1    EQU (1<<VCOB1)       

VCOB2       EQU 6       ; VCO Bias bit 2
BM_VCOB2    EQU (1<<VCOB2)       

VCOB3       EQU 7       ; VCO Bias bit 3
BM_VCOB3    EQU (1<<VCOB3)       

; FEVCT - Front-End VCO Tuning Register
FEVCT0       EQU 0       ; Front-End VCO Tuning Register bit 0
BM_FEVCT0    EQU (1<<FEVCT0)       

FEVCT1       EQU 1       ; Front-End VCO Tuning Register bit 1
BM_FEVCT1    EQU (1<<FEVCT1)       

FEVCT2       EQU 2       ; Front-End VCO Tuning Register bit 2
BM_FEVCT2    EQU (1<<FEVCT2)       

FEVCT3       EQU 3       ; Front-End VCO Tuning Register bit 3
BM_FEVCT3    EQU (1<<FEVCT3)       

FEVCT_R4       EQU 4       ; 
BM_FEVCT_R4    EQU (1<<FEVCT_R4)       

FEVCT_R5       EQU 5       ; 
BM_FEVCT_R5    EQU (1<<FEVCT_R5)       

FEVCT_R6       EQU 6       ; 
BM_FEVCT_R6    EQU (1<<FEVCT_R6)       

FEVCT_R7       EQU 7       ; 
BM_FEVCT_R7    EQU (1<<FEVCT_R7)       

; SPARE1 - Front-End Spare Register 1
SPARE10       EQU 0       ; 
BM_SPARE10    EQU (1<<SPARE10)       

SPARE11       EQU 1       ; 
BM_SPARE11    EQU (1<<SPARE11)       

SPARE12       EQU 2       ; 
BM_SPARE12    EQU (1<<SPARE12)       

SPARE13       EQU 3       ; 
BM_SPARE13    EQU (1<<SPARE13)       

SPARE14       EQU 4       ; 
BM_SPARE14    EQU (1<<SPARE14)       

SPARE15       EQU 5       ; 
BM_SPARE15    EQU (1<<SPARE15)       

SPARE16       EQU 6       ; 
BM_SPARE16    EQU (1<<SPARE16)       

SPARE17       EQU 7       ; 
BM_SPARE17    EQU (1<<SPARE17)       

; SPARE2 - Front-End Spare Register 2
SPARE20       EQU 0       ; 
BM_SPARE20    EQU (1<<SPARE20)       

SPARE21       EQU 1       ; 
BM_SPARE21    EQU (1<<SPARE21)       

SPARE22       EQU 2       ; 
BM_SPARE22    EQU (1<<SPARE22)       

SPARE23       EQU 3       ; 
BM_SPARE23    EQU (1<<SPARE23)       

SPARE24       EQU 4       ; 
BM_SPARE24    EQU (1<<SPARE24)       

SPARE25       EQU 5       ; 
BM_SPARE25    EQU (1<<SPARE25)       

SPARE26       EQU 6       ; 
BM_SPARE26    EQU (1<<SPARE26)       

SPARE27       EQU 7       ; 
BM_SPARE27    EQU (1<<SPARE27)       

; SPARE3 - Front-End Spare Register 3
SPARE30       EQU 0       ; 
BM_SPARE30    EQU (1<<SPARE30)       

SPARE31       EQU 1       ; 
BM_SPARE31    EQU (1<<SPARE31)       

SPARE32       EQU 2       ; 
BM_SPARE32    EQU (1<<SPARE32)       

SPARE33       EQU 3       ; 
BM_SPARE33    EQU (1<<SPARE33)       

SPARE34       EQU 4       ; 
BM_SPARE34    EQU (1<<SPARE34)       

SPARE35       EQU 5       ; 
BM_SPARE35    EQU (1<<SPARE35)       

SPARE36       EQU 6       ; 
BM_SPARE36    EQU (1<<SPARE36)       

SPARE37       EQU 7       ; 
BM_SPARE37    EQU (1<<SPARE37)       

; SPARE4 - Front-End Spare Register 4
SPARE40       EQU 0       ; 
BM_SPARE40    EQU (1<<SPARE40)       

SPARE41       EQU 1       ; 
BM_SPARE41    EQU (1<<SPARE41)       

SPARE42       EQU 2       ; 
BM_SPARE42    EQU (1<<SPARE42)       

SPARE43       EQU 3       ; 
BM_SPARE43    EQU (1<<SPARE43)       

SPARE44       EQU 4       ; 
BM_SPARE44    EQU (1<<SPARE44)       

SPARE45       EQU 5       ; 
BM_SPARE45    EQU (1<<SPARE45)       

SPARE46       EQU 6       ; 
BM_SPARE46    EQU (1<<SPARE46)       

SPARE47       EQU 7       ; 
BM_SPARE47    EQU (1<<SPARE47)       

; ***** GPIOREGS_DVCC ****************
; GPIOR0 - General Purpose I/O Register 0
GPIOR00       EQU 0       ; 
BM_GPIOR00    EQU (1<<GPIOR00)       

GPIOR01       EQU 1       ; 
BM_GPIOR01    EQU (1<<GPIOR01)       

GPIOR02       EQU 2       ; 
BM_GPIOR02    EQU (1<<GPIOR02)       

GPIOR03       EQU 3       ; 
BM_GPIOR03    EQU (1<<GPIOR03)       

GPIOR04       EQU 4       ; 
BM_GPIOR04    EQU (1<<GPIOR04)       

GPIOR05       EQU 5       ; 
BM_GPIOR05    EQU (1<<GPIOR05)       

GPIOR06       EQU 6       ; 
BM_GPIOR06    EQU (1<<GPIOR06)       

GPIOR07       EQU 7       ; 
BM_GPIOR07    EQU (1<<GPIOR07)       

; GPIOR1 - General Purpose I/O Register 1
GPIOR10       EQU 0       ; 
BM_GPIOR10    EQU (1<<GPIOR10)       

GPIOR11       EQU 1       ; 
BM_GPIOR11    EQU (1<<GPIOR11)       

GPIOR12       EQU 2       ; 
BM_GPIOR12    EQU (1<<GPIOR12)       

GPIOR13       EQU 3       ; 
BM_GPIOR13    EQU (1<<GPIOR13)       

GPIOR14       EQU 4       ; 
BM_GPIOR14    EQU (1<<GPIOR14)       

GPIOR15       EQU 5       ; 
BM_GPIOR15    EQU (1<<GPIOR15)       

GPIOR16       EQU 6       ; 
BM_GPIOR16    EQU (1<<GPIOR16)       

GPIOR17       EQU 7       ; 
BM_GPIOR17    EQU (1<<GPIOR17)       

; GPIOR2 - General Purpose I/O Register 2
GPIOR20       EQU 0       ; 
BM_GPIOR20    EQU (1<<GPIOR20)       

GPIOR21       EQU 1       ; 
BM_GPIOR21    EQU (1<<GPIOR21)       

GPIOR22       EQU 2       ; 
BM_GPIOR22    EQU (1<<GPIOR22)       

GPIOR23       EQU 3       ; 
BM_GPIOR23    EQU (1<<GPIOR23)       

GPIOR24       EQU 4       ; 
BM_GPIOR24    EQU (1<<GPIOR24)       

GPIOR25       EQU 5       ; 
BM_GPIOR25    EQU (1<<GPIOR25)       

GPIOR26       EQU 6       ; 
BM_GPIOR26    EQU (1<<GPIOR26)       

GPIOR27       EQU 7       ; 
BM_GPIOR27    EQU (1<<GPIOR27)       

; ***** GPIOREGS_LFVCC ***************
; GPIOR3 - General Purpose I/O Register 3
GPIOR30       EQU 0       ; 
BM_GPIOR30    EQU (1<<GPIOR30)       

GPIOR31       EQU 1       ; 
BM_GPIOR31    EQU (1<<GPIOR31)       

GPIOR32       EQU 2       ; 
BM_GPIOR32    EQU (1<<GPIOR32)       

GPIOR33       EQU 3       ; 
BM_GPIOR33    EQU (1<<GPIOR33)       

GPIOR34       EQU 4       ; 
BM_GPIOR34    EQU (1<<GPIOR34)       

GPIOR35       EQU 5       ; 
BM_GPIOR35    EQU (1<<GPIOR35)       

GPIOR36       EQU 6       ; 
BM_GPIOR36    EQU (1<<GPIOR36)       

GPIOR37       EQU 7       ; 
BM_GPIOR37    EQU (1<<GPIOR37)       

; GPIOR4 - General Purpose I/O Register 4
GPIOR40       EQU 0       ; 
BM_GPIOR40    EQU (1<<GPIOR40)       

GPIOR41       EQU 1       ; 
BM_GPIOR41    EQU (1<<GPIOR41)       

GPIOR42       EQU 2       ; 
BM_GPIOR42    EQU (1<<GPIOR42)       

GPIOR43       EQU 3       ; 
BM_GPIOR43    EQU (1<<GPIOR43)       

GPIOR44       EQU 4       ; 
BM_GPIOR44    EQU (1<<GPIOR44)       

GPIOR45       EQU 5       ; 
BM_GPIOR45    EQU (1<<GPIOR45)       

GPIOR46       EQU 6       ; 
BM_GPIOR46    EQU (1<<GPIOR46)       

GPIOR47       EQU 7       ; 
BM_GPIOR47    EQU (1<<GPIOR47)       

; GPIOR5 - General Purpose I/O Register 5
GPIOR50       EQU 0       ; 
BM_GPIOR50    EQU (1<<GPIOR50)       

GPIOR51       EQU 1       ; 
BM_GPIOR51    EQU (1<<GPIOR51)       

GPIOR52       EQU 2       ; 
BM_GPIOR52    EQU (1<<GPIOR52)       

GPIOR53       EQU 3       ; 
BM_GPIOR53    EQU (1<<GPIOR53)       

GPIOR54       EQU 4       ; 
BM_GPIOR54    EQU (1<<GPIOR54)       

GPIOR55       EQU 5       ; 
BM_GPIOR55    EQU (1<<GPIOR55)       

GPIOR56       EQU 6       ; 
BM_GPIOR56    EQU (1<<GPIOR56)       

GPIOR57       EQU 7       ; 
BM_GPIOR57    EQU (1<<GPIOR57)       

; GPIOR6 - General Purpose I/O Register 6
GPIOR60       EQU 0       ; 
BM_GPIOR60    EQU (1<<GPIOR60)       

GPIOR61       EQU 1       ; 
BM_GPIOR61    EQU (1<<GPIOR61)       

GPIOR62       EQU 2       ; 
BM_GPIOR62    EQU (1<<GPIOR62)       

GPIOR63       EQU 3       ; 
BM_GPIOR63    EQU (1<<GPIOR63)       

GPIOR64       EQU 4       ; 
BM_GPIOR64    EQU (1<<GPIOR64)       

GPIOR65       EQU 5       ; 
BM_GPIOR65    EQU (1<<GPIOR65)       

GPIOR66       EQU 6       ; 
BM_GPIOR66    EQU (1<<GPIOR66)       

GPIOR67       EQU 7       ; 
BM_GPIOR67    EQU (1<<GPIOR67)       

; GPIOR7 - General Purpose I/O Register 7
GPIOR70       EQU 0       ; 
BM_GPIOR70    EQU (1<<GPIOR70)       

GPIOR71       EQU 1       ; 
BM_GPIOR71    EQU (1<<GPIOR71)       

GPIOR72       EQU 2       ; 
BM_GPIOR72    EQU (1<<GPIOR72)       

GPIOR73       EQU 3       ; 
BM_GPIOR73    EQU (1<<GPIOR73)       

GPIOR74       EQU 4       ; 
BM_GPIOR74    EQU (1<<GPIOR74)       

GPIOR75       EQU 5       ; 
BM_GPIOR75    EQU (1<<GPIOR75)       

GPIOR76       EQU 6       ; 
BM_GPIOR76    EQU (1<<GPIOR76)       

GPIOR77       EQU 7       ; 
BM_GPIOR77    EQU (1<<GPIOR77)       

; GPIOR8 - General Purpose I/O Register 8
GPIOR80       EQU 0       ; 
BM_GPIOR80    EQU (1<<GPIOR80)       

GPIOR81       EQU 1       ; 
BM_GPIOR81    EQU (1<<GPIOR81)       

GPIOR82       EQU 2       ; 
BM_GPIOR82    EQU (1<<GPIOR82)       

GPIOR83       EQU 3       ; 
BM_GPIOR83    EQU (1<<GPIOR83)       

GPIOR84       EQU 4       ; 
BM_GPIOR84    EQU (1<<GPIOR84)       

GPIOR85       EQU 5       ; 
BM_GPIOR85    EQU (1<<GPIOR85)       

GPIOR86       EQU 6       ; 
BM_GPIOR86    EQU (1<<GPIOR86)       

GPIOR87       EQU 7       ; 
BM_GPIOR87    EQU (1<<GPIOR87)       

; ***** INT **************************
; EICRA - External Interrupt control Register
ISC00       EQU 0       ; Interrupt Sense Control 0 bit 0
BM_ISC00    EQU (1<<ISC00)       

ISC01       EQU 1       ; Interrupt Sense Control 0 bit 1
BM_ISC01    EQU (1<<ISC01)       

ISC10       EQU 2       ; Interrupt Sense Control 1 bit 0
BM_ISC10    EQU (1<<ISC10)       

ISC11       EQU 3       ; Interrupt Sense Control 1 bit 1
BM_ISC11    EQU (1<<ISC11)       

; EIFR - External Interrupt Flag Register
INTF0       EQU 0       ; External Interrupt Flag 0
BM_INTF0    EQU (1<<INTF0)       

INTF1       EQU 1       ; External Interrupt Flag 1
BM_INTF1    EQU (1<<INTF1)       

; EIMSK - External Interrupt Mask Register
INT0       EQU 0       ; External Interrupt Request 0 Enable
BM_INT0    EQU (1<<INT0)       

INT1       EQU 1       ; External Interrupt Request 1 Enable
BM_INT1    EQU (1<<INT1)       

; PCICR - Pin change Interrupt control Register
PCIE0       EQU 0       ; Pin Change Interrupt Enable 0
BM_PCIE0    EQU (1<<PCIE0)       

PCIE1       EQU 1       ; Pin Change Interrupt Enable 1
BM_PCIE1    EQU (1<<PCIE1)       

; PCIFR - Pin change Interrupt flag Register
PCIF0       EQU 0       ; Pin Change Interrupt Flag 0
BM_PCIF0    EQU (1<<PCIF0)       

PCIF1       EQU 1       ; Pin Change Interrupt Flag 1
BM_PCIF1    EQU (1<<PCIF1)       

; PCMSK0 - Pin change Mask Register 0
PCINT0       EQU 0       ; Pin Change Enable Mask bit 0
BM_PCINT0    EQU (1<<PCINT0)       

PCINT1       EQU 1       ; Pin Change Enable Mask bit 1
BM_PCINT1    EQU (1<<PCINT1)       

PCINT2       EQU 2       ; Pin Change Enable Mask bit 2
BM_PCINT2    EQU (1<<PCINT2)       

PCINT3       EQU 3       ; Pin Change Enable Mask bit 3
BM_PCINT3    EQU (1<<PCINT3)       

PCINT4       EQU 4       ; Pin Change Enable Mask bit 4
BM_PCINT4    EQU (1<<PCINT4)       

PCINT5       EQU 5       ; Pin Change Enable Mask bit 5
BM_PCINT5    EQU (1<<PCINT5)       

PCINT6       EQU 6       ; Pin Change Enable Mask bit 6
BM_PCINT6    EQU (1<<PCINT6)       

PCINT7       EQU 7       ; Pin Change Enable Mask bit 7
BM_PCINT7    EQU (1<<PCINT7)       

; PCMSK1 - Pin change Mask Register 1
PCINT8       EQU 0       ; Pin Change Enable Mask bit 8
BM_PCINT8    EQU (1<<PCINT8)       

PCINT9       EQU 1       ; Pin Change Enable Mask bit 9
BM_PCINT9    EQU (1<<PCINT9)       

PCINT10       EQU 2       ; Pin Change Enable Mask bit 10
BM_PCINT10    EQU (1<<PCINT10)       

PCINT11       EQU 3       ; Pin Change Enable Mask bit 11
BM_PCINT11    EQU (1<<PCINT11)       

PCINT12       EQU 4       ; Pin Change Enable Mask bit 12
BM_PCINT12    EQU (1<<PCINT12)       

PCINT13       EQU 5       ; Pin Change Enable Mask bit 13
BM_PCINT13    EQU (1<<PCINT13)       

PCINT14       EQU 6       ; Pin Change Enable Mask bit 14
BM_PCINT14    EQU (1<<PCINT14)       

PCINT15       EQU 7       ; Pin Change Enable Mask bit 15
BM_PCINT15    EQU (1<<PCINT15)       

; ***** LED **************************
; PDSCR - Pad Driver Strength Control Register
PDSC0       EQU 0       ; Pad Driver Strength Control bit 0
BM_PDSC0    EQU (1<<PDSC0)       

PDSC1       EQU 1       ; Pad Driver Strength Control bit 1
BM_PDSC1    EQU (1<<PDSC1)       

PDSC2       EQU 2       ; Pad Driver Strength Control bit 2
BM_PDSC2    EQU (1<<PDSC2)       

PDSC3       EQU 3       ; Pad Driver Strength Control bit 3
BM_PDSC3    EQU (1<<PDSC3)       

PDSC4       EQU 4       ; Pad Driver Strength Control bit 4
BM_PDSC4    EQU (1<<PDSC4)       

STBTEST       EQU 5       ; Standby Test Enable
BM_STBTEST    EQU (1<<STBTEST)       

RSSISEL       EQU 6       ; Testmode Enable for RSSI Measurement
BM_RSSISEL    EQU (1<<RSSISEL)       

ATBSEL       EQU 7       ; Analog Testbus Select
BM_ATBSEL    EQU (1<<ATBSEL)       

; ***** LF_FIFO **********************
; LDFC - LF Data FIFO Configuration Register
LDFFLC0       EQU 0       ; LF Data FIFO Fill-Level Configuration bit 0
BM_LDFFLC0    EQU (1<<LDFFLC0)       

LDFFLC1       EQU 1       ; LF Data FIFO Fill-Level Configuration bit 1
BM_LDFFLC1    EQU (1<<LDFFLC1)       

LDFFLC2       EQU 2       ; LF Data FIFO Fill-Level Configuration bit 2
BM_LDFFLC2    EQU (1<<LDFFLC2)       

LDFFLC3       EQU 3       ; LF Data FIFO Fill-Level Configuration bit 3
BM_LDFFLC3    EQU (1<<LDFFLC3)       

LDFFLC4       EQU 4       ; LF Data FIFO Fill-Level Configuration bit 4
BM_LDFFLC4    EQU (1<<LDFFLC4)       

LDFFLC5       EQU 5       ; LF Data FIFO Fill-Level Configuration bit 5
BM_LDFFLC5    EQU (1<<LDFFLC5)       

LDFMSB       EQU 6       ; LF Data FIFO MSB alignment bit
BM_LDFMSB    EQU (1<<LDFMSB)       

; LDFCKSW - LF Data FIFO Clock Switch Register
LDFSCSW       EQU 0       ; LF Data FIFO Synchronous Clock Switch
BM_LDFSCSW    EQU (1<<LDFSCSW)       

LDFSCKS       EQU 1       ; LF Data FIFO Synchronous Clock Status
BM_LDFSCKS    EQU (1<<LDFSCKS)       

; LDFFL - LF Data FIFO Fill Level Register
LDFFL0       EQU 0       ; LF Data Fifo Fill Level bit 0
BM_LDFFL0    EQU (1<<LDFFL0)       

LDFFL1       EQU 1       ; LF Data Fifo Fill Level bit 1
BM_LDFFL1    EQU (1<<LDFFL1)       

LDFFL2       EQU 2       ; LF Data Fifo Fill Level bit 2
BM_LDFFL2    EQU (1<<LDFFL2)       

LDFFL3       EQU 3       ; LF Data Fifo Fill Level bit 3
BM_LDFFL3    EQU (1<<LDFFL3)       

LDFFL4       EQU 4       ; LF Data Fifo Fill Level bit 4
BM_LDFFL4    EQU (1<<LDFFL4)       

LDFFL5       EQU 5       ; LF Data Fifo Fill Level bit 5
BM_LDFFL5    EQU (1<<LDFFL5)       

LDFCLR       EQU 7       ; LF Data FIFO Clear
BM_LDFCLR    EQU (1<<LDFCLR)       

; LDFIM - LF Data FIFO Interrupt Mask Register
LDFFLIM       EQU 0       ; LF Data FIFO Fill-level Interrupt Mask
BM_LDFFLIM    EQU (1<<LDFFLIM)       

LDFEIM       EQU 1       ; LF Data FIFO Error Interrupt Mask
BM_LDFEIM    EQU (1<<LDFEIM)       

; LDFRP - LF Data FIFO Read Pointer
LDFRP0       EQU 0       ; LF Data FIFO Read Pointer bit 0
BM_LDFRP0    EQU (1<<LDFRP0)       

LDFRP1       EQU 1       ; LF Data FIFO Read Pointer bit 1
BM_LDFRP1    EQU (1<<LDFRP1)       

LDFRP2       EQU 2       ; LF Data FIFO Read Pointer bit 2
BM_LDFRP2    EQU (1<<LDFRP2)       

LDFRP3       EQU 3       ; LF Data FIFO Read Pointer bit 3
BM_LDFRP3    EQU (1<<LDFRP3)       

LDFRP4       EQU 4       ; LF Data FIFO Read Pointer bit 4
BM_LDFRP4    EQU (1<<LDFRP4)       

LDFRP5       EQU 5       ; PH FIFO Read Pointer bit 5
BM_LDFRP5    EQU (1<<LDFRP5)       

; LDFS - LF Data FIFO Status Register
LDFFLR       EQU 0       ; LF Data FIFO Fill-Level Reached flag
BM_LDFFLR    EQU (1<<LDFFLR)       

LDFUF       EQU 1       ; LF Data FIFO UnderFlow flag
BM_LDFUF    EQU (1<<LDFUF)       

LDFOF       EQU 2       ; LF Data FIFO OverFlow flag
BM_LDFOF    EQU (1<<LDFOF)       

; LDFWP - LF Data FIFO Write Pointer
LDFWP0       EQU 0       ; LF Data FIFO Write Pointer bit 0
BM_LDFWP0    EQU (1<<LDFWP0)       

LDFWP1       EQU 1       ; LF Data FIFO Write Pointer bit 1
BM_LDFWP1    EQU (1<<LDFWP1)       

LDFWP2       EQU 2       ; LF Data FIFO Write Pointer bit 2
BM_LDFWP2    EQU (1<<LDFWP2)       

LDFWP3       EQU 3       ; LF Data FIFO Write Pointer bit 3
BM_LDFWP3    EQU (1<<LDFWP3)       

LDFWP4       EQU 4       ; LF Data FIFO Write Pointer bit 4
BM_LDFWP4    EQU (1<<LDFWP4)       

LDFWP5       EQU 5       ; LF Data FIFO Write Pointer bit 5
BM_LDFWP5    EQU (1<<LDFWP5)       

; ***** LF_PROTOCOL_HANDLER **********
; PHBCRR - Protocol Handler Bit Counter Read Register
PHBCR0       EQU 0       ; Protocol Handler Bit Counter Read bit 0
BM_PHBCR0    EQU (1<<PHBCR0)       

PHBCR1       EQU 1       ; Protocol Handler Bit Counter Read bit 1
BM_PHBCR1    EQU (1<<PHBCR1)       

PHBCR2       EQU 2       ; Protocol Handler Bit Counter Read bit 2
BM_PHBCR2    EQU (1<<PHBCR2)       

PHBCR3       EQU 3       ; Protocol Handler Bit Counter Read bit 3
BM_PHBCR3    EQU (1<<PHBCR3)       

PHBCR4       EQU 4       ; Protocol Handler Bit Counter Read bit 4
BM_PHBCR4    EQU (1<<PHBCR4)       

PHBCR5       EQU 5       ; Protocol Handler Bit Counter Read bit 5
BM_PHBCR5    EQU (1<<PHBCR5)       

PHBCR6       EQU 6       ; Protocol Handler Bit Counter Read bit 6
BM_PHBCR6    EQU (1<<PHBCR6)       

PHBCR7       EQU 7       ; Protocol Handler Bit Counter Read bit 7
BM_PHBCR7    EQU (1<<PHBCR7)       

; PHCRCR - Protocol Handler CRC Control Register
CRCFR       EQU 2       ; CRC ID Frame
BM_CRCFR    EQU (1<<CRCFR)       

CRCSE0       EQU 4       ; CRC size bit 0
BM_CRCSE0    EQU (1<<CRCSE0)       

CRCSE1       EQU 5       ; CRC size bit 1
BM_CRCSE1    EQU (1<<CRCSE1)       

CRCEN       EQU 7       ; CRC Enable
BM_CRCEN    EQU (1<<CRCEN)       

; PHCRPH - PH CRC Polynomial High Byte Register
PHCRP8       EQU 0       ; 
BM_PHCRP8    EQU (1<<PHCRP8)       

PHCRP9       EQU 1       ; 
BM_PHCRP9    EQU (1<<PHCRP9)       

PHCRP10       EQU 2       ; 
BM_PHCRP10    EQU (1<<PHCRP10)       

PHCRP11       EQU 3       ; 
BM_PHCRP11    EQU (1<<PHCRP11)       

PHCRP12       EQU 4       ; 
BM_PHCRP12    EQU (1<<PHCRP12)       

PHCRP13       EQU 5       ; 
BM_PHCRP13    EQU (1<<PHCRP13)       

PHCRP14       EQU 6       ; 
BM_PHCRP14    EQU (1<<PHCRP14)       

PHCRP15       EQU 7       ; 
BM_PHCRP15    EQU (1<<PHCRP15)       

; PHCRPL - PH CRC Polynomial Low Byte Register
PHCRP0       EQU 0       ; 
BM_PHCRP0    EQU (1<<PHCRP0)       

PHCRP1       EQU 1       ; 
BM_PHCRP1    EQU (1<<PHCRP1)       

PHCRP2       EQU 2       ; 
BM_PHCRP2    EQU (1<<PHCRP2)       

PHCRP3       EQU 3       ; 
BM_PHCRP3    EQU (1<<PHCRP3)       

PHCRP4       EQU 4       ; 
BM_PHCRP4    EQU (1<<PHCRP4)       

PHCRP5       EQU 5       ; 
BM_PHCRP5    EQU (1<<PHCRP5)       

PHCRP6       EQU 6       ; 
BM_PHCRP6    EQU (1<<PHCRP6)       

PHCRP7       EQU 7       ; 
BM_PHCRP7    EQU (1<<PHCRP7)       

; PHCSRH - PH CRC Checksum High Byte Register
PHCSR8       EQU 0       ; 
BM_PHCSR8    EQU (1<<PHCSR8)       

PHCSR9       EQU 1       ; 
BM_PHCSR9    EQU (1<<PHCSR9)       

PHCSR10       EQU 2       ; 
BM_PHCSR10    EQU (1<<PHCSR10)       

PHCSR11       EQU 3       ; 
BM_PHCSR11    EQU (1<<PHCSR11)       

PHCSR12       EQU 4       ; 
BM_PHCSR12    EQU (1<<PHCSR12)       

PHCSR13       EQU 5       ; 
BM_PHCSR13    EQU (1<<PHCSR13)       

PHCSR14       EQU 6       ; 
BM_PHCSR14    EQU (1<<PHCSR14)       

PHCSR15       EQU 7       ; 
BM_PHCSR15    EQU (1<<PHCSR15)       

; PHCSRL - PH CRC Checksum Low Byte Register
PHCSR0       EQU 0       ; 
BM_PHCSR0    EQU (1<<PHCSR0)       

PHCSR1       EQU 1       ; 
BM_PHCSR1    EQU (1<<PHCSR1)       

PHCSR2       EQU 2       ; 
BM_PHCSR2    EQU (1<<PHCSR2)       

PHCSR3       EQU 3       ; 
BM_PHCSR3    EQU (1<<PHCSR3)       

PHCSR4       EQU 4       ; 
BM_PHCSR4    EQU (1<<PHCSR4)       

PHCSR5       EQU 5       ; 
BM_PHCSR5    EQU (1<<PHCSR5)       

PHCSR6       EQU 6       ; 
BM_PHCSR6    EQU (1<<PHCSR6)       

PHCSR7       EQU 7       ; 
BM_PHCSR7    EQU (1<<PHCSR7)       

; PHCSTH - PH CRC Start Value High Byte Register
PHCST8       EQU 0       ; 
BM_PHCST8    EQU (1<<PHCST8)       

PHCST9       EQU 1       ; 
BM_PHCST9    EQU (1<<PHCST9)       

PHCST10       EQU 2       ; 
BM_PHCST10    EQU (1<<PHCST10)       

PHCST11       EQU 3       ; 
BM_PHCST11    EQU (1<<PHCST11)       

PHCST12       EQU 4       ; 
BM_PHCST12    EQU (1<<PHCST12)       

PHCST13       EQU 5       ; 
BM_PHCST13    EQU (1<<PHCST13)       

PHCST14       EQU 6       ; 
BM_PHCST14    EQU (1<<PHCST14)       

PHCST15       EQU 7       ; 
BM_PHCST15    EQU (1<<PHCST15)       

; PHCSTL - PH CRC Start Value Low Byte Register
PHCST0       EQU 0       ; 
BM_PHCST0    EQU (1<<PHCST0)       

PHCST1       EQU 1       ; 
BM_PHCST1    EQU (1<<PHCST1)       

PHCST2       EQU 2       ; 
BM_PHCST2    EQU (1<<PHCST2)       

PHCST3       EQU 3       ; 
BM_PHCST3    EQU (1<<PHCST3)       

PHCST4       EQU 4       ; 
BM_PHCST4    EQU (1<<PHCST4)       

PHCST5       EQU 5       ; 
BM_PHCST5    EQU (1<<PHCST5)       

PHCST6       EQU 6       ; 
BM_PHCST6    EQU (1<<PHCST6)       

PHCST7       EQU 7       ; 
BM_PHCST7    EQU (1<<PHCST7)       

; PHDFR - Protocol Handler Data Frame end Register
PHDF0       EQU 0       ; Protocol Handler Data Frame end bit 0
BM_PHDF0    EQU (1<<PHDF0)       

PHDF1       EQU 1       ; Protocol Handler Data Frame end bit 1
BM_PHDF1    EQU (1<<PHDF1)       

PHDF2       EQU 2       ; Protocol Handler Data Frame end bit 2
BM_PHDF2    EQU (1<<PHDF2)       

PHDF3       EQU 3       ; Protocol Handler Data Frame end bit 3
BM_PHDF3    EQU (1<<PHDF3)       

PHDF4       EQU 4       ; Protocol Handler Data Frame end bit 4
BM_PHDF4    EQU (1<<PHDF4)       

PHDF5       EQU 5       ; Protocol Handler Data Frame end bit 5
BM_PHDF5    EQU (1<<PHDF5)       

PHDF6       EQU 6       ; Protocol Handler Data Frame end bit 6
BM_PHDF6    EQU (1<<PHDF6)       

PHDF7       EQU 7       ; Protocol Handler Data Frame end bit 7
BM_PHDF7    EQU (1<<PHDF7)       

; PHFR - Protocol Handler Flag Register
CRCEF       EQU 0       ; PH CRC Error Status Flag
BM_CRCEF    EQU (1<<CRCEF)       

PHTBLF       EQU 1       ; Protocol Handler Telegram Bit Length Flag
BM_PHTBLF    EQU (1<<PHTBLF)       

PHDFF       EQU 2       ; Protocol Handler Date Frame Flag
BM_PHDFF    EQU (1<<PHDFF)       

PHIDFF       EQU 3       ; Protocol Handler ID Frame Flag
BM_PHIDFF    EQU (1<<PHIDFF)       

PHID0F       EQU 4       ; Protocol Handler IDentifier 0 Flag
BM_PHID0F    EQU (1<<PHID0F)       

PHID1F       EQU 5       ; Protocol Handler IDentifier 1 Flag
BM_PHID1F    EQU (1<<PHID1F)       

; PHID0L - PH Identifier 0 Length Register
ID0FS0       EQU 0       ; Identifier 0 Frame Select bit 0
BM_ID0FS0    EQU (1<<ID0FS0)       

ID0FS1       EQU 1       ; Identifier 0 Frame Select bit 1
BM_ID0FS1    EQU (1<<ID0FS1)       

ID0FS2       EQU 2       ; Identifier 0 Frame Select bit 2
BM_ID0FS2    EQU (1<<ID0FS2)       

ID0FS3       EQU 3       ; Identifier 0 Frame Select bit 3
BM_ID0FS3    EQU (1<<ID0FS3)       

ID0FS4       EQU 4       ; Identifier 0 Frame Select bit 4
BM_ID0FS4    EQU (1<<ID0FS4)       

ID0FS5       EQU 5       ; Identifier 0 Frame Select bit 5
BM_ID0FS5    EQU (1<<ID0FS5)       

; PHID1L - PH Identifier 1 Length Register
ID1FS0       EQU 0       ; Identifier 0 Frame Select bit 0
BM_ID1FS0    EQU (1<<ID1FS0)       

ID1FS1       EQU 1       ; Identifier 0 Frame Select bit 1
BM_ID1FS1    EQU (1<<ID1FS1)       

ID1FS2       EQU 2       ; Identifier 0 Frame Select bit 2
BM_ID1FS2    EQU (1<<ID1FS2)       

ID1FS3       EQU 3       ; Identifier 0 Frame Select bit 3
BM_ID1FS3    EQU (1<<ID1FS3)       

ID1FS4       EQU 4       ; Identifier 0 Frame Select bit 4
BM_ID1FS4    EQU (1<<ID1FS4)       

ID1FS5       EQU 5       ; Identifier 0 Frame Select bit 5
BM_ID1FS5    EQU (1<<ID1FS5)       

; PHIDFR - Protocol Handler ID Frame Register
RDFS0       EQU 0       ; 
BM_RDFS0    EQU (1<<RDFS0)       

RDFS1       EQU 1       ; 
BM_RDFS1    EQU (1<<RDFS1)       

RDFS2       EQU 2       ; 
BM_RDFS2    EQU (1<<RDFS2)       

RDFS3       EQU 3       ; 
BM_RDFS3    EQU (1<<RDFS3)       

RDFS4       EQU 4       ; 
BM_RDFS4    EQU (1<<RDFS4)       

RDFS5       EQU 5       ; 
BM_RDFS5    EQU (1<<RDFS5)       

RDFS6       EQU 6       ; 
BM_RDFS6    EQU (1<<RDFS6)       

RDFS7       EQU 7       ; 
BM_RDFS7    EQU (1<<RDFS7)       

; PHIMR - Protocol Handler Interrupt Mask Register
PHTBLIM       EQU 1       ; Protocol Handler Telegram Bit Length Interrupt Mask
BM_PHTBLIM    EQU (1<<PHTBLIM)       

PHDFIM       EQU 2       ; Protocol Handler Date Frame Interrupt Mask
BM_PHDFIM    EQU (1<<PHDFIM)       

PHIDFIM       EQU 3       ; Protocol Handler ID Frame Interrupt Mask
BM_PHIDFIM    EQU (1<<PHIDFIM)       

PHID0IM       EQU 4       ; Protocol Handler IDentifier 0 Interrupt Mask
BM_PHID0IM    EQU (1<<PHID0IM)       

PHID1IM       EQU 5       ; Protocol Handler IDentifier 1 Interrupt Mask
BM_PHID1IM    EQU (1<<PHID1IM)       

; PHTBLR - Protocol Handler Telegram Bit Length Register
PHTBL0       EQU 0       ; 
BM_PHTBL0    EQU (1<<PHTBL0)       

PHTBL1       EQU 1       ; 
BM_PHTBL1    EQU (1<<PHTBL1)       

PHTBL2       EQU 2       ; 
BM_PHTBL2    EQU (1<<PHTBL2)       

PHTBL3       EQU 3       ; 
BM_PHTBL3    EQU (1<<PHTBL3)       

PHTBL4       EQU 4       ; 
BM_PHTBL4    EQU (1<<PHTBL4)       

PHTBL5       EQU 5       ; 
BM_PHTBL5    EQU (1<<PHTBL5)       

PHTBL6       EQU 6       ; 
BM_PHTBL6    EQU (1<<PHTBL6)       

PHTBL7       EQU 7       ; 
BM_PHTBL7    EQU (1<<PHTBL7)       

; PHTCR - PH Telegram Configuration Register
FRFIFO       EQU 5       ; ID Frame stored in FIFO
BM_FRFIFO    EQU (1<<FRFIFO)       

CPM       EQU 6       ; Continuous Pattern Mode
BM_CPM    EQU (1<<CPM)       

CSM       EQU 7       ; Continue or Single receive Mode
BM_CSM    EQU (1<<CSM)       

; ***** LF_RECEIVER ******************
; LFCR0 - LF Receiver Control Register 0
LFCE1       EQU 0       ; LF Receiver Channel 1 Enable
BM_LFCE1    EQU (1<<LFCE1)       

LFCE2       EQU 1       ; LF Receiver Channel 2 Enable
BM_LFCE2    EQU (1<<LFCE2)       

LFCE3       EQU 2       ; LF Receiver Channel 3 Enable
BM_LFCE3    EQU (1<<LFCE3)       

LFBR0       EQU 3       ; LF Receiver Bit Rate bit 0
BM_LFBR0    EQU (1<<LFBR0)       

LFBR1       EQU 4       ; LF Receiver Bit Rate bit 1
BM_LFBR1    EQU (1<<LFBR1)       

LFMG       EQU 5       ; LF Receiver Maximum Gain Select
BM_LFMG    EQU (1<<LFMG)       

LFRRT0       EQU 6       ; LF Receiver Reset Time select bit 0
BM_LFRRT0    EQU (1<<LFRRT0)       

LFRRT1       EQU 7       ; LF Receiver Reset Time select bit 1
BM_LFRRT1    EQU (1<<LFRRT1)       

; LFCR1 - LF Receiver Control Register 1
RSST0       EQU 0       ; RSSI Startup TIme bit 0
BM_RSST0    EQU (1<<RSST0)       

RSST1       EQU 1       ; RSSI Startup TIme bit 1
BM_RSST1    EQU (1<<RSST1)       

LFFM1       EQU 2       ; LF Function Mode bit 1
BM_LFFM1    EQU (1<<LFFM1)       

ARMDE       EQU 3       ; Analog Reset Manchester Detector Enable
BM_ARMDE    EQU (1<<ARMDE)       

FLLEN       EQU 4       ; FLL Enable
BM_FLLEN    EQU (1<<FLLEN)       

ADTHEN       EQU 5       ; Adapt Threshold Enable
BM_ADTHEN    EQU (1<<ADTHEN)       

LFPEEN       EQU 6       ; LF Port Event Enable
BM_LFPEEN    EQU (1<<LFPEEN)       

LFRE       EQU 7       ; LF Receiver Enable
BM_LFRE    EQU (1<<LFRE)       

; LFCR2 - LF Receiver Control Register 2
LFSEN0       EQU 0       ; LF Sensitivity Mode bit 0
BM_LFSEN0    EQU (1<<LFSEN0)       

LFSEN1       EQU 1       ; LF Sensitivity Mode bit 1
BM_LFSEN1    EQU (1<<LFSEN1)       

LFDAMP       EQU 2       ; LF coil Damping level select
BM_LFDAMP    EQU (1<<LFDAMP)       

LFVC0       EQU 5       ; LF Velocity Control bit 0
BM_LFVC0    EQU (1<<LFVC0)       

LFVC1       EQU 6       ; LF Velocity Control bit 1
BM_LFVC1    EQU (1<<LFVC1)       

LFVC2       EQU 7       ; LF Velocity Control bit 2
BM_LFVC2    EQU (1<<LFVC2)       

; LFCR3 - LF Receiver Control Register 3
LFRCTEN       EQU 0       ; LF RC Trim Enable
BM_LFRCTEN    EQU (1<<LFRCTEN)       

LFRCPCEN       EQU 1       ; LF RC Pump Continuous Enable
BM_LFRCPCEN    EQU (1<<LFRCPCEN)       

LFRCPM       EQU 2       ; LF RC Pump mode Enable
BM_LFRCPM    EQU (1<<LFRCPM)       

LFTON       EQU 3       ; LF On Time with Activated Standby Mode
BM_LFTON    EQU (1<<LFTON)       

LFTS0       EQU 4       ; LF Standby Time
BM_LFTS0    EQU (1<<LFTS0)       

LFTS1       EQU 5       ; LF Standby Time
BM_LFTS1    EQU (1<<LFTS1)       

LFTS2       EQU 6       ; LF Standby Time
BM_LFTS2    EQU (1<<LFTS2)       

LFSBEN       EQU 7       ; LF Standby mode Enable
BM_LFSBEN    EQU (1<<LFSBEN)       

; LFDSR1 - LF Receiver Decoder Setting Register 1
LOTHA0       EQU 0       ; Low Threshold A bit 0
BM_LOTHA0    EQU (1<<LOTHA0)       

LOTHA1       EQU 1       ; Low Threshold A bit 1
BM_LOTHA1    EQU (1<<LOTHA1)       

HITHA0       EQU 2       ; High Threshold A bit 0
BM_HITHA0    EQU (1<<HITHA0)       

HITHA1       EQU 3       ; High Threshold A bit 1
BM_HITHA1    EQU (1<<HITHA1)       

CTTHA0       EQU 4       ; Count Threshold A bit 0
BM_CTTHA0    EQU (1<<CTTHA0)       

CTTHA1       EQU 5       ; Count Threshold A bit 1
BM_CTTHA1    EQU (1<<CTTHA1)       

; LFDSR11 - Low Frequency Decoder Setting Register 11
TINITA0       EQU 0       ; Time Init A bit 0
BM_TINITA0    EQU (1<<TINITA0)       

TINITA1       EQU 1       ; Time Init A bit 1
BM_TINITA1    EQU (1<<TINITA1)       

TINITA2       EQU 2       ; Time Init A bit 2
BM_TINITA2    EQU (1<<TINITA2)       

TINITA3       EQU 3       ; Time Init A bit 3
BM_TINITA3    EQU (1<<TINITA3)       

TINITB0       EQU 4       ; Time Init B bit 0
BM_TINITB0    EQU (1<<TINITB0)       

TINITB1       EQU 5       ; Time Init B bit 1
BM_TINITB1    EQU (1<<TINITB1)       

TINITB2       EQU 6       ; Time Init B bit 2
BM_TINITB2    EQU (1<<TINITB2)       

TINITB3       EQU 7       ; Time Init B bit 3
BM_TINITB3    EQU (1<<TINITB3)       

; LFDSR2 - LF Receiver Decoder Setting Register 2
LOTHB0       EQU 0       ; Low Threshold B bit 0
BM_LOTHB0    EQU (1<<LOTHB0)       

LOTHB1       EQU 1       ; Low Threshold B bit 1
BM_LOTHB1    EQU (1<<LOTHB1)       

HITHB0       EQU 2       ; High Threshold B bit 0
BM_HITHB0    EQU (1<<HITHB0)       

HITHB1       EQU 3       ; High Threshold B bit 1
BM_HITHB1    EQU (1<<HITHB1)       

CTTHB0       EQU 4       ; Count Threshold B bit 0
BM_CTTHB0    EQU (1<<CTTHB0)       

CTTHB1       EQU 5       ; Count Threshold B bit 1
BM_CTTHB1    EQU (1<<CTTHB1)       

; LFDSR3 - LF Receiver Decoder Setting Register 3
PBDTH0       EQU 0       ; Preburst Detection Threshold bit 0
BM_PBDTH0    EQU (1<<PBDTH0)       

PBDTH1       EQU 1       ; Preburst Detection Threshold bit 1
BM_PBDTH1    EQU (1<<PBDTH1)       

QCTH0       EQU 3       ; Quality Check Threshold bit 0
BM_QCTH0    EQU (1<<QCTH0)       

QCTH1       EQU 4       ; Quality Check Threshold bit 1
BM_QCTH1    EQU (1<<QCTH1)       

QCTH2       EQU 5       ; Quality Check Threshold bit 2
BM_QCTH2    EQU (1<<QCTH2)       

; LFDSR4 - LF Receiver Decoder Setting Register 4
SDTHA0       EQU 0       ; Sync Detection Threshold A bit 0
BM_SDTHA0    EQU (1<<SDTHA0)       

SDTHA1       EQU 1       ; Sync Detection Threshold A bit 1
BM_SDTHA1    EQU (1<<SDTHA1)       

SDTHA2       EQU 2       ; Sync Detection Threshold A bit 2
BM_SDTHA2    EQU (1<<SDTHA2)       

SCTHA0       EQU 3       ; Sync Count Threshold A bit 0
BM_SCTHA0    EQU (1<<SCTHA0)       

SCTHA1       EQU 4       ; Sync Count Threshold A bit 1
BM_SCTHA1    EQU (1<<SCTHA1)       

SCTHA2       EQU 5       ; Sync Count Threshold A bit 2
BM_SCTHA2    EQU (1<<SCTHA2)       

SRSTC0       EQU 6       ; Sync Restart Control bit 0
BM_SRSTC0    EQU (1<<SRSTC0)       

SRSTC1       EQU 7       ; Sync Restart Control bit 1
BM_SRSTC1    EQU (1<<SRSTC1)       

; LFFR - LF Receiver Flag Register
LFSYDF       EQU 0       ; LF Synchronization Detect Flag
BM_LFSYDF    EQU (1<<LFSYDF)       

LFDEF       EQU 1       ; LF Decoder Error Flag
BM_LFDEF    EQU (1<<LFDEF)       

LFEOF       EQU 2       ; LF End of Telegram Flag
BM_LFEOF    EQU (1<<LFEOF)       

LFTOF       EQU 3       ; LF telegram Time Out Flag
BM_LFTOF    EQU (1<<LFTOF)       

LFSD       EQU 6       ; LF Signal Detect
BM_LFSD    EQU (1<<LFSD)       

LFES       EQU 7       ; LF Envelope Signal
BM_LFES    EQU (1<<LFES)       

; LFIMR - LF Receiver Interrupt Mask Register
LFSYDIM       EQU 0       ; LF Synchronization Detected Interrupt Mask
BM_LFSYDIM    EQU (1<<LFSYDIM)       

LFDEIM       EQU 1       ; LF Decoder Error Interrupt Mask
BM_LFDEIM    EQU (1<<LFDEIM)       

LFEOIM       EQU 2       ; LF End Of telegram Interrupt Mask
BM_LFEOIM    EQU (1<<LFEOIM)       

; LFQC1 - LF Receiver Channel 1 Quality Faktor Register
LFQS10       EQU 0       ; LF resistor select channel 1 bit 0
BM_LFQS10    EQU (1<<LFQS10)       

LFQS11       EQU 1       ; LF resistor select channel 1 bit 1
BM_LFQS11    EQU (1<<LFQS11)       

LFQS12       EQU 2       ; LF resistor select channel 1 bit 2
BM_LFQS12    EQU (1<<LFQS12)       

LFQS13       EQU 3       ; LF resistor select channel 1 bit 3
BM_LFQS13    EQU (1<<LFQS13)       

LFCS10       EQU 4       ; LF capacitor select channel 1 bit 0
BM_LFCS10    EQU (1<<LFCS10)       

LFCS11       EQU 5       ; LF capacitor select channel 1 bit 1
BM_LFCS11    EQU (1<<LFCS11)       

LFCS12       EQU 6       ; LF capacitor select channel 1 bit 2
BM_LFCS12    EQU (1<<LFCS12)       

LFCS13       EQU 7       ; LF capacitor select channel 1 bit 3
BM_LFCS13    EQU (1<<LFCS13)       

; LFQC2 - LF Receiver Channel 2 Quality Faktor Register
LFQS20       EQU 0       ; LF resistor select channel 2 bit 0
BM_LFQS20    EQU (1<<LFQS20)       

LFQS21       EQU 1       ; LF resistor select channel 2 bit 1
BM_LFQS21    EQU (1<<LFQS21)       

LFQS22       EQU 2       ; LF resistor select channel 2 bit 2
BM_LFQS22    EQU (1<<LFQS22)       

LFQS23       EQU 3       ; LF resistor select channel 2 bit 3
BM_LFQS23    EQU (1<<LFQS23)       

LFCS20       EQU 4       ; LF capacitor select channel 2 bit 0
BM_LFCS20    EQU (1<<LFCS20)       

LFCS21       EQU 5       ; LF capacitor select channel 2 bit 1
BM_LFCS21    EQU (1<<LFCS21)       

LFCS22       EQU 6       ; LF capacitor select channel 2 bit 2
BM_LFCS22    EQU (1<<LFCS22)       

LFCS23       EQU 7       ; LF capacitor select channel 2 bit 3
BM_LFCS23    EQU (1<<LFCS23)       

; LFQC3 - LF Receiver Channel 3 Quality Faktor Register
LFQS30       EQU 0       ; LF resistor select channel 3 bit 0
BM_LFQS30    EQU (1<<LFQS30)       

LFQS31       EQU 1       ; LF resistor select channel 3 bit 1
BM_LFQS31    EQU (1<<LFQS31)       

LFQS32       EQU 2       ; LF resistor select channel 3 bit 2
BM_LFQS32    EQU (1<<LFQS32)       

LFQS33       EQU 3       ; LF resistor select channel 3 bit 3
BM_LFQS33    EQU (1<<LFQS33)       

LFCS30       EQU 4       ; LF capacitor select channel 3 bit 0
BM_LFCS30    EQU (1<<LFCS30)       

LFCS31       EQU 5       ; LF capacitor select channel 3 bit 1
BM_LFCS31    EQU (1<<LFCS31)       

LFCS32       EQU 6       ; LF capacitor select channel 3 bit 2
BM_LFCS32    EQU (1<<LFCS32)       

LFCS33       EQU 7       ; LF capacitor select channel 3 bit 3
BM_LFCS33    EQU (1<<LFCS33)       

; LFSTOP - LF Receiver Stop Bit Register
LFSTSY0       EQU 0       ; LF Stop Bits Symbols bit 0
BM_LFSTSY0    EQU (1<<LFSTSY0)       

LFSTSY1       EQU 1       ; LF Stop Bits Symbols bit 1
BM_LFSTSY1    EQU (1<<LFSTSY1)       

LFSTSY2       EQU 2       ; LF Stop Bits Symbols bit 2
BM_LFSTSY2    EQU (1<<LFSTSY2)       

LFSTSY3       EQU 3       ; LF Stop Bits Symbols bit 3
BM_LFSTSY3    EQU (1<<LFSTSY3)       

LFSTL0       EQU 4       ; LF Stop Bits Length bit 0
BM_LFSTL0    EQU (1<<LFSTL0)       

LFSTL1       EQU 5       ; LF Stop Bits Length bit 1
BM_LFSTL1    EQU (1<<LFSTL1)       

LFSTL2       EQU 6       ; LF Stop Bits Length bit 2
BM_LFSTL2    EQU (1<<LFSTL2)       

; LFSYLE - LF Receiver Synchronization Length Register
LFSYLE0       EQU 0       ; 
BM_LFSYLE0    EQU (1<<LFSYLE0)       

LFSYLE1       EQU 1       ; 
BM_LFSYLE1    EQU (1<<LFSYLE1)       

LFSYLE2       EQU 2       ; 
BM_LFSYLE2    EQU (1<<LFSYLE2)       

LFSYLE3       EQU 3       ; 
BM_LFSYLE3    EQU (1<<LFSYLE3)       

LFSYLE4       EQU 4       ; 
BM_LFSYLE4    EQU (1<<LFSYLE4)       

LFSYLE5       EQU 5       ; 
BM_LFSYLE5    EQU (1<<LFSYLE5)       

; ***** LF_RSSI **********************
; RSCALIB - RSSI Calibration Register
RSCALIB0       EQU 0       ; RSSI Calibration Bit 0
BM_RSCALIB0    EQU (1<<RSCALIB0)       

RSCALIB1       EQU 1       ; RSSI Calibration Bit 1
BM_RSCALIB1    EQU (1<<RSCALIB1)       

RSCALIB2       EQU 2       ; RSSI Calibration Bit 2
BM_RSCALIB2    EQU (1<<RSCALIB2)       

RSCALIB3       EQU 3       ; RSSI Calibration Bit 3
BM_RSCALIB3    EQU (1<<RSCALIB3)       

RSCALIB4       EQU 4       ; RSSI Calibration Bit 4
BM_RSCALIB4    EQU (1<<RSCALIB4)       

RSCALIB5       EQU 5       ; RSSI Calibration Bit 5
BM_RSCALIB5    EQU (1<<RSCALIB5)       

RSCALIB6       EQU 6       ; RSSI Calibration Bit 6
BM_RSCALIB6    EQU (1<<RSCALIB6)       

RSCALIB7       EQU 7       ; RSSI Calibration Bit 7
BM_RSCALIB7    EQU (1<<RSCALIB7)       

; RSCR - RSSI Control Register
RSSDEN       EQU 0       ; RSSI Sign Detection Enable Bit
BM_RSSDEN    EQU (1<<RSSDEN)       

RSOS       EQU 1       ; RSSI Operation Start Bit
BM_RSOS    EQU (1<<RSOS)       

RSEOR       EQU 2       ; RSSI End Operation Request Bit
BM_RSEOR    EQU (1<<RSEOR)       

RSOFM       EQU 3       ; RSSI Operation Flag Mask Bit
BM_RSOFM    EQU (1<<RSOFM)       

RSMODE0       EQU 4       ; RSSI Mode Bit 0
BM_RSMODE0    EQU (1<<RSMODE0)       

RSMODE1       EQU 5       ; RSSI Mode Bit 1
BM_RSMODE1    EQU (1<<RSMODE1)       

RSRES       EQU 7       ; RSSI Reset Bit
BM_RSRES    EQU (1<<RSRES)       

; RSDBGR - RSSI Debug Register
RSSANA       EQU 0       ; RSSI Set Analog Bit
BM_RSSANA    EQU (1<<RSSANA)       

RSINFM       EQU 2       ; RSSI Infinity Measurement Bit
BM_RSINFM    EQU (1<<RSINFM)       

RSFPD       EQU 3       ; RSSI Fix Pre-Divider Bit
BM_RSFPD    EQU (1<<RSFPD)       

RSHOME       EQU 4       ; RSSI Hold Measurement Bit
BM_RSHOME    EQU (1<<RSHOME)       

RSDBGS0       EQU 5       ; RSSI Debug Select Bit 0
BM_RSDBGS0    EQU (1<<RSDBGS0)       

RSDBGS1       EQU 6       ; RSSI Debug Select Bit 1
BM_RSDBGS1    EQU (1<<RSDBGS1)       

RSDBGEN       EQU 7       ; RSSI Debug Enable Bit
BM_RSDBGEN    EQU (1<<RSDBGEN)       

; RSDLYR - RSSI Delay Register
RSTRD0       EQU 0       ; RSSI Tracking Delay Bit 0
BM_RSTRD0    EQU (1<<RSTRD0)       

RSTRD1       EQU 1       ; RSSI Tracking Delay Bit 1
BM_RSTRD1    EQU (1<<RSTRD1)       

RSTRD2       EQU 2       ; RSSI Tracking Delay Bit 2
BM_RSTRD2    EQU (1<<RSTRD2)       

RSTRD3       EQU 3       ; RSSI Tracking Delay Bit 3
BM_RSTRD3    EQU (1<<RSTRD3)       

RSTRD4       EQU 4       ; RSSI Tracking Delay Bit 4
BM_RSTRD4    EQU (1<<RSTRD4)       

RSTRD5       EQU 5       ; RSSI Tracking Delay Bit 5
BM_RSTRD5    EQU (1<<RSTRD5)       

RSRD0       EQU 6       ; RSSI Range Delay Bit 0
BM_RSRD0    EQU (1<<RSRD0)       

RSRD1       EQU 7       ; RSSI Range Delay Bit 1
BM_RSRD1    EQU (1<<RSRD1)       

; RSFR - RSSI Flag Register
RSOOR1       EQU 0       ; RSSI Out of Range Channel 1 Bit
BM_RSOOR1    EQU (1<<RSOOR1)       

RSOOR2       EQU 1       ; RSSI Out of Range Channel 2 Bit
BM_RSOOR2    EQU (1<<RSOOR2)       

RSOOR3       EQU 2       ; RSSI Out of Range Channel 3 Bit
BM_RSOOR3    EQU (1<<RSOOR3)       

RSOFF       EQU 3       ; RSSI Operation Finish Flag Bit
BM_RSOFF    EQU (1<<RSOFF)       

RSAOOR1       EQU 5       ; RSSI Averaging Out of Range Channel 1 Bit
BM_RSAOOR1    EQU (1<<RSAOOR1)       

RSAOOR2       EQU 6       ; RSSI Averaging Out of Range Channel 2 Bit
BM_RSAOOR2    EQU (1<<RSAOOR2)       

RSAOOR3       EQU 7       ; RSSI Averaging Out of Range Channel 3 Bit
BM_RSAOOR3    EQU (1<<RSAOOR3)       

; RSMS1R - RSSI Measurement Setting 1 Register
RSCH1E       EQU 0       ; RSSI Channel 1 Enable Bit
BM_RSCH1E    EQU (1<<RSCH1E)       

RSCH2E       EQU 1       ; RSSI Channel 2 Enable Bit
BM_RSCH2E    EQU (1<<RSCH2E)       

RSCH3E       EQU 2       ; RSSI Channel 3 Enable Bit
BM_RSCH3E    EQU (1<<RSCH3E)       

RSINTM       EQU 3       ; RSSI internal Measurement Bit
BM_RSINTM    EQU (1<<RSINTM)       

RSSTIM       EQU 4       ; RSSI Stimulus Bit
BM_RSSTIM    EQU (1<<RSSTIM)       

RSCMS       EQU 5       ; RSSI Current Mode Select Bit
BM_RSCMS    EQU (1<<RSCMS)       

RSSSV       EQU 6       ; RSSI Show Sample Value Bit
BM_RSSSV    EQU (1<<RSSSV)       

RSSCAL       EQU 7       ; RSSI Set Calibration Bit
BM_RSSCAL    EQU (1<<RSSCAL)       

; RSMS2R - RSSI Measurement Setting 2 Register
RSSADR0       EQU 0       ; RSSI Sample Address Bit 0
BM_RSSADR0    EQU (1<<RSSADR0)       

RSSADR1       EQU 1       ; RSSI Sample Address Bit 1
BM_RSSADR1    EQU (1<<RSSADR1)       

RSSADR2       EQU 2       ; RSSI Sample Address Bit 2
BM_RSSADR2    EQU (1<<RSSADR2)       

RSSADR3       EQU 3       ; RSSI Sample Address Bit 3
BM_RSSADR3    EQU (1<<RSSADR3)       

RSAVGS0       EQU 4       ; RSSI Average Select Bit 0
BM_RSAVGS0    EQU (1<<RSAVGS0)       

RSAVGS1       EQU 5       ; RSSI Average Select Bit 1
BM_RSAVGS1    EQU (1<<RSAVGS1)       

RSAVGS2       EQU 6       ; RSSI Average Select Bit 2
BM_RSAVGS2    EQU (1<<RSAVGS2)       

RSAVGS3       EQU 7       ; RSSI Average Select Bit 3
BM_RSAVGS3    EQU (1<<RSAVGS3)       

; RSRES1H - RSSI Result 1 High Byte Register
RSRES1H0       EQU 0       ; RSSI Result 1 High Byte Bit 0
BM_RSRES1H0    EQU (1<<RSRES1H0)       

RSRES1H1       EQU 1       ; RSSI Result 1 High Byte Bit 1
BM_RSRES1H1    EQU (1<<RSRES1H1)       

RSRES1H2       EQU 2       ; RSSI Result 1 High Byte Bit 2
BM_RSRES1H2    EQU (1<<RSRES1H2)       

RSRES1H3       EQU 3       ; RSSI Result 1 High Byte Bit 3
BM_RSRES1H3    EQU (1<<RSRES1H3)       

RSRES1H4       EQU 4       ; RSSI Result 1 High Byte Bit 4
BM_RSRES1H4    EQU (1<<RSRES1H4)       

RSRES1H5       EQU 5       ; RSSI Result 1 High Byte Bit 5
BM_RSRES1H5    EQU (1<<RSRES1H5)       

RSRES1H6       EQU 6       ; RSSI Result 1 High Byte Bit 6
BM_RSRES1H6    EQU (1<<RSRES1H6)       

RSRES1H7       EQU 7       ; RSSI Result 1 High Byte Bit 7
BM_RSRES1H7    EQU (1<<RSRES1H7)       

; RSRES1L - RSSI Result 1 Low Byte Register
RSRES1L0       EQU 0       ; RSSI Result 1 Low Byte Bit 0
BM_RSRES1L0    EQU (1<<RSRES1L0)       

RSRES1L1       EQU 1       ; RSSI Result 1 Low Byte Bit 1
BM_RSRES1L1    EQU (1<<RSRES1L1)       

RSRES1L2       EQU 2       ; RSSI Result 1 Low Byte Bit 2
BM_RSRES1L2    EQU (1<<RSRES1L2)       

RSRES1L3       EQU 3       ; RSSI Result 1 Low Byte Bit 3
BM_RSRES1L3    EQU (1<<RSRES1L3)       

RSRES1L4       EQU 4       ; RSSI Result 1 Low Byte Bit 4
BM_RSRES1L4    EQU (1<<RSRES1L4)       

RSRES1L5       EQU 5       ; RSSI Result 1 Low Byte Bit 5
BM_RSRES1L5    EQU (1<<RSRES1L5)       

RSRES1L6       EQU 6       ; RSSI Result 1 Low Byte Bit 6
BM_RSRES1L6    EQU (1<<RSRES1L6)       

RSRES1L7       EQU 7       ; RSSI Result 1 Low Byte Bit 7
BM_RSRES1L7    EQU (1<<RSRES1L7)       

; RSRES2H - RSSI Result 2 High Byte Register
RSRES2H0       EQU 0       ; RSSI Result 2 High Byte Bit 0
BM_RSRES2H0    EQU (1<<RSRES2H0)       

RSRES2H1       EQU 1       ; RSSI Result 2 High Byte Bit 1
BM_RSRES2H1    EQU (1<<RSRES2H1)       

RSRES2H2       EQU 2       ; RSSI Result 2 High Byte Bit 2
BM_RSRES2H2    EQU (1<<RSRES2H2)       

RSRES2H3       EQU 3       ; RSSI Result 2 High Byte Bit 3
BM_RSRES2H3    EQU (1<<RSRES2H3)       

RSRES2H4       EQU 4       ; RSSI Result 2 High Byte Bit 4
BM_RSRES2H4    EQU (1<<RSRES2H4)       

RSRES2H5       EQU 5       ; RSSI Result 2 High Byte Bit 5
BM_RSRES2H5    EQU (1<<RSRES2H5)       

RSRES2H6       EQU 6       ; RSSI Result 2 High Byte Bit 6
BM_RSRES2H6    EQU (1<<RSRES2H6)       

RSRES2H7       EQU 7       ; RSSI Result 2 High Byte Bit 7
BM_RSRES2H7    EQU (1<<RSRES2H7)       

; RSRES2L - RSSI Result 2 Low Byte Register
RSRES2L0       EQU 0       ; RSSI Result 2 Low Byte Bit 0
BM_RSRES2L0    EQU (1<<RSRES2L0)       

RSRES2L1       EQU 1       ; RSSI Result 2 Low Byte Bit 1
BM_RSRES2L1    EQU (1<<RSRES2L1)       

RSRES2L2       EQU 2       ; RSSI Result 2 Low Byte Bit 2
BM_RSRES2L2    EQU (1<<RSRES2L2)       

RSRES2L3       EQU 3       ; RSSI Result 2 Low Byte Bit 3
BM_RSRES2L3    EQU (1<<RSRES2L3)       

RSRES2L4       EQU 4       ; RSSI Result 2 Low Byte Bit 4
BM_RSRES2L4    EQU (1<<RSRES2L4)       

RSRES2L5       EQU 5       ; RSSI Result 2 Low Byte Bit 5
BM_RSRES2L5    EQU (1<<RSRES2L5)       

RSRES2L6       EQU 6       ; RSSI Result 2 Low Byte Bit 6
BM_RSRES2L6    EQU (1<<RSRES2L6)       

RSRES2L7       EQU 7       ; RSSI Result 2 Low Byte Bit 7
BM_RSRES2L7    EQU (1<<RSRES2L7)       

; RSRES3H - RSSI Result 3 High Byte Register
RSRES3H0       EQU 0       ; RSSI Result 3 High Byte Bit 0
BM_RSRES3H0    EQU (1<<RSRES3H0)       

RSRES3H1       EQU 1       ; RSSI Result 3 High Byte Bit 1
BM_RSRES3H1    EQU (1<<RSRES3H1)       

RSRES3H2       EQU 2       ; RSSI Result 3 High Byte Bit 2
BM_RSRES3H2    EQU (1<<RSRES3H2)       

RSRES3H3       EQU 3       ; RSSI Result 3 High Byte Bit 3
BM_RSRES3H3    EQU (1<<RSRES3H3)       

RSRES3H4       EQU 4       ; RSSI Result 3 High Byte Bit 4
BM_RSRES3H4    EQU (1<<RSRES3H4)       

RSRES3H5       EQU 5       ; RSSI Result 3 High Byte Bit 5
BM_RSRES3H5    EQU (1<<RSRES3H5)       

RSRES3H6       EQU 6       ; RSSI Result 3 High Byte Bit 6
BM_RSRES3H6    EQU (1<<RSRES3H6)       

RSRES3H7       EQU 7       ; RSSI Result 3 High Byte Bit 7
BM_RSRES3H7    EQU (1<<RSRES3H7)       

; RSRES3L - RSSI Result 3 Low Byte Register
RSRES3L0       EQU 0       ; RSSI Result 3 Low Byte Bit 0
BM_RSRES3L0    EQU (1<<RSRES3L0)       

RSRES3L1       EQU 1       ; RSSI Result 3 Low Byte Bit 1
BM_RSRES3L1    EQU (1<<RSRES3L1)       

RSRES3L2       EQU 2       ; RSSI Result 3 Low Byte Bit 2
BM_RSRES3L2    EQU (1<<RSRES3L2)       

RSRES3L3       EQU 3       ; RSSI Result 3 Low Byte Bit 3
BM_RSRES3L3    EQU (1<<RSRES3L3)       

RSRES3L4       EQU 4       ; RSSI Result 3 Low Byte Bit 4
BM_RSRES3L4    EQU (1<<RSRES3L4)       

RSRES3L5       EQU 5       ; RSSI Result 3 Low Byte Bit 5
BM_RSRES3L5    EQU (1<<RSRES3L5)       

RSRES3L6       EQU 6       ; RSSI Result 3 Low Byte Bit 6
BM_RSRES3L6    EQU (1<<RSRES3L6)       

RSRES3L7       EQU 7       ; RSSI Result 3 Low Byte Bit 7
BM_RSRES3L7    EQU (1<<RSRES3L7)       

; RSRES4H - RSSI Result 4 High Byte Register
RSRES4H0       EQU 0       ; RSSI Result 4 High Byte Bit 0
BM_RSRES4H0    EQU (1<<RSRES4H0)       

RSRES4H1       EQU 1       ; RSSI Result 4 High Byte Bit 1
BM_RSRES4H1    EQU (1<<RSRES4H1)       

RSRES4H2       EQU 2       ; RSSI Result 4 High Byte Bit 2
BM_RSRES4H2    EQU (1<<RSRES4H2)       

RSRES4H3       EQU 3       ; RSSI Result 4 High Byte Bit 3
BM_RSRES4H3    EQU (1<<RSRES4H3)       

RSRES4H4       EQU 4       ; RSSI Result 4 High Byte Bit 4
BM_RSRES4H4    EQU (1<<RSRES4H4)       

RSRES4H5       EQU 5       ; RSSI Result 4 High Byte Bit 5
BM_RSRES4H5    EQU (1<<RSRES4H5)       

RSRES4H6       EQU 6       ; RSSI Result 4 High Byte Bit 6
BM_RSRES4H6    EQU (1<<RSRES4H6)       

RSRES4H7       EQU 7       ; RSSI Result 4 High Byte Bit 7
BM_RSRES4H7    EQU (1<<RSRES4H7)       

; RSRES4L - RSSI Result 4 Low Byte Register
RSRES4L0       EQU 0       ; RSSI Result 4 Low Byte Bit 0
BM_RSRES4L0    EQU (1<<RSRES4L0)       

RSRES4L1       EQU 1       ; RSSI Result 4 Low Byte Bit 1
BM_RSRES4L1    EQU (1<<RSRES4L1)       

RSRES4L2       EQU 2       ; RSSI Result 4 Low Byte Bit 2
BM_RSRES4L2    EQU (1<<RSRES4L2)       

RSRES4L3       EQU 3       ; RSSI Result 4 Low Byte Bit 3
BM_RSRES4L3    EQU (1<<RSRES4L3)       

RSRES4L4       EQU 4       ; RSSI Result 4 Low Byte Bit 4
BM_RSRES4L4    EQU (1<<RSRES4L4)       

RSRES4L5       EQU 5       ; RSSI Result 4 Low Byte Bit 5
BM_RSRES4L5    EQU (1<<RSRES4L5)       

RSRES4L6       EQU 6       ; RSSI Result 4 Low Byte Bit 6
BM_RSRES4L6    EQU (1<<RSRES4L6)       

RSRES4L7       EQU 7       ; RSSI Result 4 Low Byte Bit 7
BM_RSRES4L7    EQU (1<<RSRES4L7)       

; RSSR - RSSI Status Register
RSRDY       EQU 0       ; RSSI Ready Bit
BM_RSRDY    EQU (1<<RSRDY)       

RSSVLD       EQU 1       ; RSSI Sample Valid Bit
BM_RSSVLD    EQU (1<<RSSVLD)       

; RSSRCR - RSSI SRC Calibration Register
SRCMODE0       EQU 0       ; RSSI SRC Calibration Mode Bit 0
BM_SRCMODE0    EQU (1<<SRCMODE0)       

SRCMODE1       EQU 1       ; RSSI SRC Calibration Mode Bit 1
BM_SRCMODE1    EQU (1<<SRCMODE1)       

SRCMIN0       EQU 2       ; RSSI SRC Minimum Value Bit 0
BM_SRCMIN0    EQU (1<<SRCMIN0)       

SRCMIN1       EQU 3       ; RSSI SRC Minimum Value Bit 1
BM_SRCMIN1    EQU (1<<SRCMIN1)       

SRCCLR       EQU 4       ; RSSI SRC Clear Bit
BM_SRCCLR    EQU (1<<SRCCLR)       

SRCSTEP0       EQU 6       ; RSSI SRC Step Bit 0
BM_SRCSTEP0    EQU (1<<SRCSTEP0)       

SRCSTEP1       EQU 7       ; RSSI SRC Step Bit 1
BM_SRCSTEP1    EQU (1<<SRCSTEP1)       

; SD12RR - Sign Detection Channel 1 vs 2 Result Register
SD12RR0       EQU 0       ; Sign Detection Channel 1 vs 2 Result Bit 0
BM_SD12RR0    EQU (1<<SD12RR0)       

SD12RR1       EQU 1       ; Sign Detection Channel 1 vs 2 Result Bit 1
BM_SD12RR1    EQU (1<<SD12RR1)       

SD12RR2       EQU 2       ; Sign Detection Channel 1 vs 2 Result Bit 2
BM_SD12RR2    EQU (1<<SD12RR2)       

SD12RR3       EQU 3       ; Sign Detection Channel 1 vs 2 Result Bit 3
BM_SD12RR3    EQU (1<<SD12RR3)       

SD12RR4       EQU 4       ; Sign Detection Channel 1 vs 2 Result Bit 4
BM_SD12RR4    EQU (1<<SD12RR4)       

SD12RR5       EQU 5       ; Sign Detection Channel 1 vs 2 Result Bit 5
BM_SD12RR5    EQU (1<<SD12RR5)       

SD12RR6       EQU 6       ; Sign Detection Channel 1 vs 2 Result Bit 6
BM_SD12RR6    EQU (1<<SD12RR6)       

SD12RR7       EQU 7       ; Sign Detection Channel 1 vs 2 Result Bit 7
BM_SD12RR7    EQU (1<<SD12RR7)       

; SD13RR - Sign Detection Channel 1 vs 3 Result Register
SD13RR0       EQU 0       ; Sign Detection Channel 1 vs 3 Result Bit 0
BM_SD13RR0    EQU (1<<SD13RR0)       

SD13RR1       EQU 1       ; Sign Detection Channel 1 vs 3 Result Bit 1
BM_SD13RR1    EQU (1<<SD13RR1)       

SD13RR2       EQU 2       ; Sign Detection Channel 1 vs 3 Result Bit 2
BM_SD13RR2    EQU (1<<SD13RR2)       

SD13RR3       EQU 3       ; Sign Detection Channel 1 vs 3 Result Bit 3
BM_SD13RR3    EQU (1<<SD13RR3)       

SD13RR4       EQU 4       ; Sign Detection Channel 1 vs 3 Result Bit 4
BM_SD13RR4    EQU (1<<SD13RR4)       

SD13RR5       EQU 5       ; Sign Detection Channel 1 vs 3 Result Bit 5
BM_SD13RR5    EQU (1<<SD13RR5)       

SD13RR6       EQU 6       ; Sign Detection Channel 1 vs 3 Result Bit 6
BM_SD13RR6    EQU (1<<SD13RR6)       

SD13RR7       EQU 7       ; Sign Detection Channel 1 vs 3 Result Bit 7
BM_SD13RR7    EQU (1<<SD13RR7)       

; SD23RR - Sign Detection Channel 2 vs 3 Result Register
SD23RR0       EQU 0       ; Sign Detection Channel 2 vs 3 Result Bit 0
BM_SD23RR0    EQU (1<<SD23RR0)       

SD23RR1       EQU 1       ; Sign Detection Channel 2 vs 3 Result Bit 1
BM_SD23RR1    EQU (1<<SD23RR1)       

SD23RR2       EQU 2       ; Sign Detection Channel 2 vs 3 Result Bit 2
BM_SD23RR2    EQU (1<<SD23RR2)       

SD23RR3       EQU 3       ; Sign Detection Channel 2 vs 3 Result Bit 3
BM_SD23RR3    EQU (1<<SD23RR3)       

SD23RR4       EQU 4       ; Sign Detection Channel 2 vs 3 Result Bit 4
BM_SD23RR4    EQU (1<<SD23RR4)       

SD23RR5       EQU 5       ; Sign Detection Channel 2 vs 3 Result Bit 5
BM_SD23RR5    EQU (1<<SD23RR5)       

SD23RR6       EQU 6       ; Sign Detection Channel 2 vs 3 Result Bit 6
BM_SD23RR6    EQU (1<<SD23RR6)       

SD23RR7       EQU 7       ; Sign Detection Channel 2 vs 3 Result Bit 7
BM_SD23RR7    EQU (1<<SD23RR7)       

; SD360R - Sign Detection 360 Degree Result Register
SD360R0       EQU 0       ; Sign Detection 360 Degree Result Bit 0
BM_SD360R0    EQU (1<<SD360R0)       

SD360R1       EQU 1       ; Sign Detection 360 Degree Result Bit 1
BM_SD360R1    EQU (1<<SD360R1)       

SD360R2       EQU 2       ; Sign Detection 360 Degree Result Bit 2
BM_SD360R2    EQU (1<<SD360R2)       

SD360R3       EQU 3       ; Sign Detection 360 Degree Result Bit 3
BM_SD360R3    EQU (1<<SD360R3)       

SD360R4       EQU 4       ; Sign Detection 360 Degree Result Bit 4
BM_SD360R4    EQU (1<<SD360R4)       

SD360R5       EQU 5       ; Sign Detection 360 Degree Result Bit 5
BM_SD360R5    EQU (1<<SD360R5)       

SD360R6       EQU 6       ; Sign Detection 360 Degree Result Bit 6
BM_SD360R6    EQU (1<<SD360R6)       

SD360R7       EQU 7       ; Sign Detection 360 Degree Result Bit 7
BM_SD360R7    EQU (1<<SD360R7)       

; ***** LF_TIMER *********************
; LTCMR - LF Timer Control Mode Register
LTPS0       EQU 0       ; LF Timer Prescaler Select bit 0
BM_LTPS0    EQU (1<<LTPS0)       

LTPS1       EQU 1       ; LF Timer Prescaler Select bit 1
BM_LTPS1    EQU (1<<LTPS1)       

LTPS2       EQU 2       ; LF Timer Prescaler Select bit 2
BM_LTPS2    EQU (1<<LTPS2)       

LTCRM       EQU 3       ; LF Timer Compare Reset Mask
BM_LTCRM    EQU (1<<LTCRM)       

LTCIM       EQU 4       ; LF Timer Compare Interrupt Mask
BM_LTCIM    EQU (1<<LTCIM)       

LTCM       EQU 5       ; LF Timer Continuous Mode
BM_LTCM    EQU (1<<LTCM)       

LTSM       EQU 6       ; LF Timer Start Mode
BM_LTSM    EQU (1<<LTSM)       

LTENA       EQU 7       ; LF Timer Enable
BM_LTENA    EQU (1<<LTENA)       

; LTCOR - LF Timer Compare Register
LTCOR0       EQU 0       ; 
BM_LTCOR0    EQU (1<<LTCOR0)       

LTCOR1       EQU 1       ; 
BM_LTCOR1    EQU (1<<LTCOR1)       

LTCOR2       EQU 2       ; 
BM_LTCOR2    EQU (1<<LTCOR2)       

LTCOR3       EQU 3       ; 
BM_LTCOR3    EQU (1<<LTCOR3)       

LTCOR4       EQU 4       ; 
BM_LTCOR4    EQU (1<<LTCOR4)       

LTCOR5       EQU 5       ; 
BM_LTCOR5    EQU (1<<LTCOR5)       

LTCOR6       EQU 6       ; 
BM_LTCOR6    EQU (1<<LTCOR6)       

LTCOR7       EQU 7       ; 
BM_LTCOR7    EQU (1<<LTCOR7)       

; LTEMR - LF Timer Event Mask Register
ID0EM       EQU 0       ; ID0 Event Mask
BM_ID0EM    EQU (1<<ID0EM)       

ID1EM       EQU 1       ; ID1 Event Mask
BM_ID1EM    EQU (1<<ID1EM)       

IDFEM       EQU 2       ; Identifier Frame Event Mask
BM_IDFEM    EQU (1<<IDFEM)       

DFEM       EQU 3       ; Data Frame end reached Event Mask
BM_DFEM    EQU (1<<DFEM)       

TBLEM       EQU 4       ; Telegram Bit Length reached Event Mask
BM_TBLEM    EQU (1<<TBLEM)       

FLEM       EQU 5       ; Fill Level reached Event Mask
BM_FLEM    EQU (1<<FLEM)       

EOFEM       EQU 6       ; End OF telegram Event Mask
BM_EOFEM    EQU (1<<EOFEM)       

LTCOF       EQU 7       ; LF Timer Compare Flag
BM_LTCOF    EQU (1<<LTCOF)       

; ***** LF_TRANSPONDER ***************
; TPCR1 - Transponder Control Register 1
TPQPLM       EQU 2       ; Transponder Quadrature Pulse Length Operation Modulation
BM_TPQPLM    EQU (1<<TPQPLM)       

TPBR       EQU 4       ; Transponder Bit Rate
BM_TPBR    EQU (1<<TPBR)       

TPDFCP0       EQU 5       ; Transponder Decoder Field Clock Prescaler value bit 0
BM_TPDFCP0    EQU (1<<TPDFCP0)       

TPDFCP1       EQU 6       ; Transponder Decoder Field Clock Prescaler value bit 1
BM_TPDFCP1    EQU (1<<TPDFCP1)       

TPMODE       EQU 7       ; Transponder Mode
BM_TPMODE    EQU (1<<TPMODE)       

; TPCR2 - Transponder Control 2 Register
TPMA       EQU 0       ; Transponder Mode Acknowledge
BM_TPMA    EQU (1<<TPMA)       

TPMOD       EQU 1       ; Transponder Modulation
BM_TPMOD    EQU (1<<TPMOD)       

TPPSD       EQU 2       ; Transponder Power Switch Disable
BM_TPPSD    EQU (1<<TPPSD)       

TPD       EQU 3       ; Transponder Disable
BM_TPD    EQU (1<<TPD)       

TPNFTO       EQU 4       ; Transponder No Field Timeout enable
BM_TPNFTO    EQU (1<<TPNFTO)       

TPWDLV0       EQU 5       ; Transponder watch dog value for No Field timeout (with SRC clk) bit 0
BM_TPWDLV0    EQU (1<<TPWDLV0)       

TPWDLV1       EQU 6       ; Transponder watch dog value for No Field timeout (with SRC clk) bit 1
BM_TPWDLV1    EQU (1<<TPWDLV1)       

; TPCR3 - Transponder Control Register 3
TPTD       EQU 0       ; Transponder Transmit Data
BM_TPTD    EQU (1<<TPTD)       

TPRD       EQU 1       ; Transponder Receive Data
BM_TPRD    EQU (1<<TPRD)       

TPTLIW       EQU 2       ; Transponder Transmit Listen Window
BM_TPTLIW    EQU (1<<TPTLIW)       

TPRCD       EQU 5       ; Transponder Reception Continuous Damping
BM_TPRCD    EQU (1<<TPRCD)       

; TPCR4 - Transponder Control Register 4
TPBCCS0       EQU 0       ; Transponder Battery Charge Current Select bit 0
BM_TPBCCS0    EQU (1<<TPBCCS0)       

TPBCCS1       EQU 1       ; Transponder Battery Charge Current Select bit 1
BM_TPBCCS1    EQU (1<<TPBCCS1)       

TPBCCS2       EQU 2       ; Transponder Battery Charge Current Select bit 2
BM_TPBCCS2    EQU (1<<TPBCCS2)       

TPBCCS3       EQU 3       ; Transponder Battery Charge Current Select bit 3
BM_TPBCCS3    EQU (1<<TPBCCS3)       

TPBCM       EQU 4       ; Transponder Battery Charge Mode
BM_TPBCM    EQU (1<<TPBCM)       

; TPCR5 - Transponder Control Register 5
TPMUD0       EQU 0       ; Transponder Modulation Undamped Level Select bit 0
BM_TPMUD0    EQU (1<<TPMUD0)       

TPMUD1       EQU 1       ; Transponder Modulation Undamped Level Select bit 1
BM_TPMUD1    EQU (1<<TPMUD1)       

TPMUD2       EQU 2       ; Transponder Modulation Undamped Level Select bit 2
BM_TPMUD2    EQU (1<<TPMUD2)       

TPMD0       EQU 4       ; Transponder Modulation Damping Select bit 0
BM_TPMD0    EQU (1<<TPMD0)       

TPMD1       EQU 5       ; Transponder Modulation Damping Select bit 1
BM_TPMD1    EQU (1<<TPMD1)       

TPMD2       EQU 6       ; Transponder Modulation Damping Select bit 2
BM_TPMD2    EQU (1<<TPMD2)       

; TPECMR - Transponder Encoder Mode Register
TPECM10       EQU 0       ; Transponder Encoder Mode 1 bit 0
BM_TPECM10    EQU (1<<TPECM10)       

TPECM11       EQU 1       ; Transponder Encoder Mode 1 bit 1
BM_TPECM11    EQU (1<<TPECM11)       

TPECM20       EQU 2       ; Transponder Encoder Mode 2 bit 0
BM_TPECM20    EQU (1<<TPECM20)       

TPECM21       EQU 3       ; Transponder Encoder Mode 2 bit 1
BM_TPECM21    EQU (1<<TPECM21)       

TPECM30       EQU 4       ; Transponder Encoder Mode 3 bit 0
BM_TPECM30    EQU (1<<TPECM30)       

TPECM31       EQU 5       ; Transponder Encoder Mode 3 bit 1
BM_TPECM31    EQU (1<<TPECM31)       

TPECM40       EQU 6       ; Transponder Encoder Mode 4 bit 0
BM_TPECM40    EQU (1<<TPECM40)       

TPECM41       EQU 7       ; Transponder Encoder Mode 4 bit 1
BM_TPECM41    EQU (1<<TPECM41)       

; TPFR - Transponder Flag Register
TPF       EQU 0       ; Transponder Flag
BM_TPF    EQU (1<<TPF)       

TPFTF       EQU 1       ; Transponder Field Timeout Flag
BM_TPFTF    EQU (1<<TPFTF)       

TPNFTF       EQU 2       ; Transponder No Field Timeout Flag
BM_TPNFTF    EQU (1<<TPNFTF)       

TPBERF       EQU 3       ; Transponder Bit Error Flag
BM_TPBERF    EQU (1<<TPBERF)       

; TPIMR - Transponder Interrupt Mask Register
TPIM       EQU 0       ; Transponder Interrupt Mask
BM_TPIM    EQU (1<<TPIM)       

TPFTIM       EQU 1       ; Transponder Field Timeout Interrupt Mask
BM_TPFTIM    EQU (1<<TPFTIM)       

TPNFTIM       EQU 2       ; Transponder No Field Timeout Interrupt Mask
BM_TPNFTIM    EQU (1<<TPNFTIM)       

TPBERIM       EQU 3       ; Transponder Bit Error Interrupt Mask
BM_TPBERIM    EQU (1<<TPBERIM)       

; TPSR - Transponder Status Register
TPA       EQU 0       ; Transponder Active
BM_TPA    EQU (1<<TPA)       

TPGAP       EQU 1       ; Transponder GAP Signal
BM_TPGAP    EQU (1<<TPGAP)       

TPPSW       EQU 2       ; Transponder Power Switch
BM_TPPSW    EQU (1<<TPPSW)       

TPBCOK       EQU 3       ; Transponder Battery Charge OK
BM_TPBCOK    EQU (1<<TPBCOK)       

; ***** MEM **************************
; EEST - EEPROM Status Register
EESYN0       EQU 0       ; EEPROM Syndrome bit 0
BM_EESYN0    EQU (1<<EESYN0)       

EESYN1       EQU 1       ; EEPROM Syndrome bit 1
BM_EESYN1    EQU (1<<EESYN1)       

EESYN2       EQU 2       ; EEPROM Syndrome bit 2
BM_EESYN2    EQU (1<<EESYN2)       

EESYN3       EQU 3       ; EEPROM Syndrome bit 3
BM_EESYN3    EQU (1<<EESYN3)       

; PGMST - Program Memory Status Register
PGMSYN0       EQU 0       ; Program Memory Syndrome bit 0
BM_PGMSYN0    EQU (1<<PGMSYN0)       

PGMSYN1       EQU 1       ; Program Memory Syndrome bit 1
BM_PGMSYN1    EQU (1<<PGMSYN1)       

PGMSYN2       EQU 2       ; Program Memory Syndrome bit 2
BM_PGMSYN2    EQU (1<<PGMSYN2)       

PGMSYN3       EQU 3       ; Program Memory Syndrome bit 3
BM_PGMSYN3    EQU (1<<PGMSYN3)       

PGMSYN4       EQU 4       ; Program Memory Syndrome bit 4
BM_PGMSYN4    EQU (1<<PGMSYN4)       

; ***** PORTB ************************
; DDRB - Port B Data Direction Register
DDRB0       EQU 0       ; Port B Data Direction Register bit 0
BM_DDRB0    EQU (1<<DDRB0)       

DDRB1       EQU 1       ; Port B Data Direction Register bit 1
BM_DDRB1    EQU (1<<DDRB1)       

DDRB2       EQU 2       ; Port B Data Direction Register bit 2
BM_DDRB2    EQU (1<<DDRB2)       

DDRB3       EQU 3       ; Port B Data Direction Register bit 3
BM_DDRB3    EQU (1<<DDRB3)       

DDRB4       EQU 4       ; Port B Data Direction Register bit 4
BM_DDRB4    EQU (1<<DDRB4)       

DDRB5       EQU 5       ; Port B Data Direction Register bit 5
BM_DDRB5    EQU (1<<DDRB5)       

DDRB6       EQU 6       ; Port B Data Direction Register bit 6
BM_DDRB6    EQU (1<<DDRB6)       

DDRB7       EQU 7       ; Port B Data Direction Register bit 7
BM_DDRB7    EQU (1<<DDRB7)       

; PINB - Port B Input Pins
PINB0       EQU 0       ; Port B Input Pins bit 0
BM_PINB0    EQU (1<<PINB0)       

PINB1       EQU 1       ; Port B Input Pins bit 1
BM_PINB1    EQU (1<<PINB1)       

PINB2       EQU 2       ; Port B Input Pins bit 2
BM_PINB2    EQU (1<<PINB2)       

PINB3       EQU 3       ; Port B Input Pins bit 3
BM_PINB3    EQU (1<<PINB3)       

PINB4       EQU 4       ; Port B Input Pins bit 4
BM_PINB4    EQU (1<<PINB4)       

PINB5       EQU 5       ; Port B Input Pins bit 5
BM_PINB5    EQU (1<<PINB5)       

PINB6       EQU 6       ; Port B Input Pins bit 6
BM_PINB6    EQU (1<<PINB6)       

PINB7       EQU 7       ; Port B Input Pins bit 7
BM_PINB7    EQU (1<<PINB7)       

; PORTB - Port B Data Register
PORTB0       EQU 0       ; Port B Data Register bit 0
BM_PORTB0    EQU (1<<PORTB0)       

PB0       EQU 0       ; For compatibility
BM_PB0    EQU (1<<PB0)       

PORTB1       EQU 1       ; Port B Data Register bit 1
BM_PORTB1    EQU (1<<PORTB1)       

PB1       EQU 1       ; For compatibility
BM_PB1    EQU (1<<PB1)       

PORTB2       EQU 2       ; Port B Data Register bit 2
BM_PORTB2    EQU (1<<PORTB2)       

PB2       EQU 2       ; For compatibility
BM_PB2    EQU (1<<PB2)       

PORTB3       EQU 3       ; Port B Data Register bit 3
BM_PORTB3    EQU (1<<PORTB3)       

PB3       EQU 3       ; For compatibility
BM_PB3    EQU (1<<PB3)       

PORTB4       EQU 4       ; Port B Data Register bit 4
BM_PORTB4    EQU (1<<PORTB4)       

PB4       EQU 4       ; For compatibility
BM_PB4    EQU (1<<PB4)       

PORTB5       EQU 5       ; Port B Data Register bit 5
BM_PORTB5    EQU (1<<PORTB5)       

PB5       EQU 5       ; For compatibility
BM_PB5    EQU (1<<PB5)       

PORTB6       EQU 6       ; Port B Data Register bit 6
BM_PORTB6    EQU (1<<PORTB6)       

PB6       EQU 6       ; For compatibility
BM_PB6    EQU (1<<PB6)       

PORTB7       EQU 7       ; Port B Data Register bit 7
BM_PORTB7    EQU (1<<PORTB7)       

PB7       EQU 7       ; For compatibility
BM_PB7    EQU (1<<PB7)       

; ***** PORTC ************************
; DDRC - Port C Data Direction Register
DDRC0       EQU 0       ; Port C Data Direction Register bit 0
BM_DDRC0    EQU (1<<DDRC0)       

DDRC1       EQU 1       ; Port C Data Direction Register bit 1
BM_DDRC1    EQU (1<<DDRC1)       

DDRC2       EQU 2       ; Port C Data Direction Register bit 2
BM_DDRC2    EQU (1<<DDRC2)       

; PINC - Port C Input Pins
PINC0       EQU 0       ; Port C Input Pins bit 0
BM_PINC0    EQU (1<<PINC0)       

PINC1       EQU 1       ; Port C Input Pins bit 1
BM_PINC1    EQU (1<<PINC1)       

PINC2       EQU 2       ; Port C Input Pins bit 2
BM_PINC2    EQU (1<<PINC2)       

; PORTC - Port C Data Register
PORTC0       EQU 0       ; Port C Data Register bit 0
BM_PORTC0    EQU (1<<PORTC0)       

PC0       EQU 0       ; For compatibility
BM_PC0    EQU (1<<PC0)       

PORTC1       EQU 1       ; Port C Data Register bit 1
BM_PORTC1    EQU (1<<PORTC1)       

PC1       EQU 1       ; For compatibility
BM_PC1    EQU (1<<PC1)       

PORTC2       EQU 2       ; Port C Data Register bit 2
BM_PORTC2    EQU (1<<PORTC2)       

PC2       EQU 2       ; For compatibility
BM_PC2    EQU (1<<PC2)       

; ***** PORTD ************************
; DDRD - Port D Data Direction Register
DDRD0       EQU 0       ; Port D Data Direction Register bit 0
BM_DDRD0    EQU (1<<DDRD0)       

DDRD1       EQU 1       ; Port D Data Direction Register bit 1
BM_DDRD1    EQU (1<<DDRD1)       

DDRD2       EQU 2       ; Port D Data Direction Register bit 2
BM_DDRD2    EQU (1<<DDRD2)       

DDRD3       EQU 3       ; Port D Data Direction Register bit 3
BM_DDRD3    EQU (1<<DDRD3)       

DDRD4       EQU 4       ; Port D Data Direction Register bit 4
BM_DDRD4    EQU (1<<DDRD4)       

DDRD5       EQU 5       ; Port D Data Direction Register bit 5
BM_DDRD5    EQU (1<<DDRD5)       

DDRD6       EQU 6       ; Port D Data Direction Register bit 6
BM_DDRD6    EQU (1<<DDRD6)       

DDRD7       EQU 7       ; Port D Data Direction Register bit 7
BM_DDRD7    EQU (1<<DDRD7)       

; PIND - Port D Input Pins
PIND0       EQU 0       ; Port D Input Pins bit 0
BM_PIND0    EQU (1<<PIND0)       

PIND1       EQU 1       ; Port D Input Pins bit 1
BM_PIND1    EQU (1<<PIND1)       

PIND2       EQU 2       ; Port D Input Pins bit 2
BM_PIND2    EQU (1<<PIND2)       

PIND3       EQU 3       ; Port D Input Pins bit 3
BM_PIND3    EQU (1<<PIND3)       

PIND4       EQU 4       ; Port D Input Pins bit 4
BM_PIND4    EQU (1<<PIND4)       

PIND5       EQU 5       ; Port D Input Pins bit 5
BM_PIND5    EQU (1<<PIND5)       

PIND6       EQU 6       ; Port D Input Pins bit 6
BM_PIND6    EQU (1<<PIND6)       

PIND7       EQU 7       ; Port D Input Pins bit 7
BM_PIND7    EQU (1<<PIND7)       

; PORTD - Port D Data Register
PORTD0       EQU 0       ; Port D Data Register bit 0
BM_PORTD0    EQU (1<<PORTD0)       

PD0       EQU 0       ; For compatibility
BM_PD0    EQU (1<<PD0)       

PORTD1       EQU 1       ; Port D Data Register bit 1
BM_PORTD1    EQU (1<<PORTD1)       

PD1       EQU 1       ; For compatibility
BM_PD1    EQU (1<<PD1)       

PORTD2       EQU 2       ; Port D Data Register bit 2
BM_PORTD2    EQU (1<<PORTD2)       

PD2       EQU 2       ; For compatibility
BM_PD2    EQU (1<<PD2)       

PORTD3       EQU 3       ; Port D Data Register bit 3
BM_PORTD3    EQU (1<<PORTD3)       

PD3       EQU 3       ; For compatibility
BM_PD3    EQU (1<<PD3)       

PORTD4       EQU 4       ; Port D Data Register bit 4
BM_PORTD4    EQU (1<<PORTD4)       

PD4       EQU 4       ; For compatibility
BM_PD4    EQU (1<<PD4)       

PORTD5       EQU 5       ; Port D Data Register bit 5
BM_PORTD5    EQU (1<<PORTD5)       

PD5       EQU 5       ; For compatibility
BM_PD5    EQU (1<<PD5)       

PORTD6       EQU 6       ; Port D Data Register bit 6
BM_PORTD6    EQU (1<<PORTD6)       

PD6       EQU 6       ; For compatibility
BM_PD6    EQU (1<<PD6)       

PORTD7       EQU 7       ; Port D Data Register bit 7
BM_PORTD7    EQU (1<<PORTD7)       

PD7       EQU 7       ; For compatibility
BM_PD7    EQU (1<<PD7)       

; ***** SFIFO ************************
; SFC - Support FIFO Configuration Register
SFFLC0       EQU 0       ; Support FIFO Fill-Level Configuration bit 0
BM_SFFLC0    EQU (1<<SFFLC0)       

SFFLC1       EQU 1       ; Support FIFO Fill-Level Configuration bit 1
BM_SFFLC1    EQU (1<<SFFLC1)       

SFFLC2       EQU 2       ; Support FIFO Fill-Level Configuration bit 2
BM_SFFLC2    EQU (1<<SFFLC2)       

SFFLC3       EQU 3       ; Support FIFO Fill-Level Configuration bit 3
BM_SFFLC3    EQU (1<<SFFLC3)       

SFFLC4       EQU 4       ; Support FIFO Fill-Level Configuration bit 4
BM_SFFLC4    EQU (1<<SFFLC4)       

SFDRA       EQU 7       ; Support FIFO Direct Read Access Operational Mode
BM_SFDRA    EQU (1<<SFDRA)       

; SFI - Support FIFO Interrupt Mask Register
SFFLIM       EQU 0       ; Support FIFO Fill-level Interrupt Mask
BM_SFFLIM    EQU (1<<SFFLIM)       

SFERIM       EQU 1       ; Support FIFO Error Interrupt Mask
BM_SFERIM    EQU (1<<SFERIM)       

; SFL - Support FIFO Fill Level Register
SFFLS0       EQU 0       ; Support FIFO Fill Level Status bit 0
BM_SFFLS0    EQU (1<<SFFLS0)       

SFFLS1       EQU 1       ; Support FIFO Fill Level Status bit 1
BM_SFFLS1    EQU (1<<SFFLS1)       

SFFLS2       EQU 2       ; Support FIFO Fill Level Status bit 2
BM_SFFLS2    EQU (1<<SFFLS2)       

SFFLS3       EQU 3       ; Support FIFO Fill Level Status bit 3
BM_SFFLS3    EQU (1<<SFFLS3)       

SFFLS4       EQU 4       ; Support FIFO Fill Level Status bit 4
BM_SFFLS4    EQU (1<<SFFLS4)       

SFCLR       EQU 7       ; Support FIFO Clear
BM_SFCLR    EQU (1<<SFCLR)       

; SFRP - Support FIFO Read Pointer
SFRP0       EQU 0       ; Support FIFO Read Pointer bit 0
BM_SFRP0    EQU (1<<SFRP0)       

SFRP1       EQU 1       ; Support FIFO Read Pointer bit 1
BM_SFRP1    EQU (1<<SFRP1)       

SFRP2       EQU 2       ; Support FIFO Read Pointer bit 2
BM_SFRP2    EQU (1<<SFRP2)       

SFRP3       EQU 3       ; Support FIFO Read Pointer bit 3
BM_SFRP3    EQU (1<<SFRP3)       

SFRP4       EQU 4       ; Support FIFO Read Pointer bit 4
BM_SFRP4    EQU (1<<SFRP4)       

; SFS - Support FIFO Status Register
SFFLRF       EQU 0       ; Support FIFO Fill-Level Reached Status Flag
BM_SFFLRF    EQU (1<<SFFLRF)       

SFUFL       EQU 1       ; Support FIFO Underflow Flag
BM_SFUFL    EQU (1<<SFUFL)       

SFOFL       EQU 2       ; Support FIFO Overflow Flag
BM_SFOFL    EQU (1<<SFOFL)       

; SFWP - Support FIFO Write Pointer
SFWP0       EQU 0       ; Support FIFO Write Pointer bit 0
BM_SFWP0    EQU (1<<SFWP0)       

SFWP1       EQU 1       ; Support FIFO Write Pointer bit 1
BM_SFWP1    EQU (1<<SFWP1)       

SFWP2       EQU 2       ; Support FIFO Write Pointer bit 2
BM_SFWP2    EQU (1<<SFWP2)       

SFWP3       EQU 3       ; Support FIFO Write Pointer bit 3
BM_SFWP3    EQU (1<<SFWP3)       

SFWP4       EQU 4       ; Support FIFO Write Pointer bit 4
BM_SFWP4    EQU (1<<SFWP4)       

; ***** SPI **************************
; SFFR - SPI FIFO Fill Status Register
RFL0       EQU 0       ; Receive Buffer Fill Level bit 0
BM_RFL0    EQU (1<<RFL0)       

RFL1       EQU 1       ; Receive Buffer Fill Level bit 1
BM_RFL1    EQU (1<<RFL1)       

RFL2       EQU 2       ; Receive Buffer Fill Level bit 2
BM_RFL2    EQU (1<<RFL2)       

RFC       EQU 3       ; Rx Buffer Clear
BM_RFC    EQU (1<<RFC)       

TFL0       EQU 4       ; Transmit Buffer Fill Level bit 0
BM_TFL0    EQU (1<<TFL0)       

TFL1       EQU 5       ; Transmit Buffer Fill Level bit 1
BM_TFL1    EQU (1<<TFL1)       

TFL2       EQU 6       ; Transmit Buffer Fill Level bit 2
BM_TFL2    EQU (1<<TFL2)       

TFC       EQU 7       ; SPI Tx Buffer Clear
BM_TFC    EQU (1<<TFC)       

; SFIR - SPI FIFO Interrupt Register
RIL0       EQU 0       ; Receive Buffer Interrupt Level bit 0
BM_RIL0    EQU (1<<RIL0)       

RIL1       EQU 1       ; Receive Buffer Interrupt Level bit 1
BM_RIL1    EQU (1<<RIL1)       

RIL2       EQU 2       ; Receive Buffer Interrupt Level bit 2
BM_RIL2    EQU (1<<RIL2)       

SRIE       EQU 3       ; Rx Buffer Interrupt Enable
BM_SRIE    EQU (1<<SRIE)       

TIL0       EQU 4       ; Transmit Buffer Interrupt Level bit 0
BM_TIL0    EQU (1<<TIL0)       

TIL1       EQU 5       ; Transmit Buffer Interrupt Level bit 1
BM_TIL1    EQU (1<<TIL1)       

TIL2       EQU 6       ; Transmit Buffer Interrupt Level bit 2
BM_TIL2    EQU (1<<TIL2)       

STIE       EQU 7       ; SPI Rx Buffer Interrupt Enable
BM_STIE    EQU (1<<STIE)       

; SPCR - SPI control Register
SPR0       EQU 0       ; SPI Clock Rate Select 0
BM_SPR0    EQU (1<<SPR0)       

SPR1       EQU 1       ; SPI Clock Rate Select 1
BM_SPR1    EQU (1<<SPR1)       

CPHA       EQU 2       ; Clock Phase
BM_CPHA    EQU (1<<CPHA)       

CPOL       EQU 3       ; Clock Polarity
BM_CPOL    EQU (1<<CPOL)       

MSTR       EQU 4       ; Master/Slave Select
BM_MSTR    EQU (1<<MSTR)       

DORD       EQU 5       ; Data Order
BM_DORD    EQU (1<<DORD)       

SPE       EQU 6       ; SPI Enable
BM_SPE    EQU (1<<SPE)       

SPIE       EQU 7       ; Spe Interrupt Enable
BM_SPIE    EQU (1<<SPIE)       

; SPSR - SPI Status Register
SPI2X       EQU 0       ; Double SPI Speed Bit
BM_SPI2X    EQU (1<<SPI2X)       

RXIF       EQU 4       ; Rx Buffer Interrupt Flag
BM_RXIF    EQU (1<<RXIF)       

TXIF       EQU 5       ; Tx Buffer Interrupt Flag
BM_TXIF    EQU (1<<TXIF)       

SPIF       EQU 7       ; SPI Interrupt Flag
BM_SPIF    EQU (1<<SPIF)       

; ***** SPI2 *************************
; SP2CR - SPI2 control Register
SP2R0       EQU 0       ; SPI2 Clock Rate Select 0
BM_SP2R0    EQU (1<<SP2R0)       

SP2R1       EQU 1       ; SPI2 Clock Rate Select 1
BM_SP2R1    EQU (1<<SP2R1)       

CPHA2       EQU 2       ; Clock Phase
BM_CPHA2    EQU (1<<CPHA2)       

CPOL2       EQU 3       ; Clock Polarity
BM_CPOL2    EQU (1<<CPOL2)       

MSTR2       EQU 4       ; Master/Slave Select
BM_MSTR2    EQU (1<<MSTR2)       

DORD2       EQU 5       ; Data Order
BM_DORD2    EQU (1<<DORD2)       

SP2E       EQU 6       ; SPI2 Enable
BM_SP2E    EQU (1<<SP2E)       

SP2IE       EQU 7       ; Spi2 Interrupt Enable
BM_SP2IE    EQU (1<<SP2IE)       

; SP2SR - SPI2 Status Register
SPI22X       EQU 0       ; Double SPI2 Speed Bit
BM_SPI22X    EQU (1<<SPI22X)       

WCOL2       EQU 6       ; Write Colliion Flag
BM_WCOL2    EQU (1<<WCOL2)       

SP2IF       EQU 7       ; SPI2 Interrupt Flag
BM_SP2IF    EQU (1<<SP2IF)       

; ***** SSM **************************
; MSMCR1 - Master State Machine Control Register 1
MSMSM00       EQU 0       ; Master State Machine SubState Machine Select 0 bit 0
BM_MSMSM00    EQU (1<<MSMSM00)       

MSMSM01       EQU 1       ; Master State Machine SubState Machine Select 0 bit 1
BM_MSMSM01    EQU (1<<MSMSM01)       

MSMSM02       EQU 2       ; Master State Machine SubState Machine Select 0 bit 2
BM_MSMSM02    EQU (1<<MSMSM02)       

MSMSM03       EQU 3       ; Master State Machine SubState Machine Select 0 bit 3
BM_MSMSM03    EQU (1<<MSMSM03)       

MSMSM10       EQU 4       ; Master State Machine SubState Machine Select 1 bit 0
BM_MSMSM10    EQU (1<<MSMSM10)       

MSMSM11       EQU 5       ; Master State Machine SubState Machine Select 1 bit 1
BM_MSMSM11    EQU (1<<MSMSM11)       

MSMSM12       EQU 6       ; Master State Machine SubState Machine Select 1 bit 2
BM_MSMSM12    EQU (1<<MSMSM12)       

MSMSM13       EQU 7       ; Master State Machine SubState Machine Select 1 bit 3
BM_MSMSM13    EQU (1<<MSMSM13)       

; MSMCR2 - Master State Machine Control Register 2
MSMSM20       EQU 0       ; Master State Machine SubState Machine Select 2 bit 0
BM_MSMSM20    EQU (1<<MSMSM20)       

MSMSM21       EQU 1       ; Master State Machine SubState Machine Select 2 bit 1
BM_MSMSM21    EQU (1<<MSMSM21)       

MSMSM22       EQU 2       ; Master State Machine SubState Machine Select 2 bit 2
BM_MSMSM22    EQU (1<<MSMSM22)       

MSMSM23       EQU 3       ; Master State Machine SubState Machine Select 2 bit 3
BM_MSMSM23    EQU (1<<MSMSM23)       

MSMSM30       EQU 4       ; Master State Machine SubState Machine Select 3 bit 0
BM_MSMSM30    EQU (1<<MSMSM30)       

MSMSM31       EQU 5       ; Master State Machine SubState Machine Select 3 bit 1
BM_MSMSM31    EQU (1<<MSMSM31)       

MSMSM32       EQU 6       ; Master State Machine SubState Machine Select 3 bit 2
BM_MSMSM32    EQU (1<<MSMSM32)       

MSMSM33       EQU 7       ; Master State Machine SubState Machine Select 3 bit 3
BM_MSMSM33    EQU (1<<MSMSM33)       

; MSMCR3 - Master State Machine Control Register 3
MSMSM40       EQU 0       ; Master State Machine SubState Machine Select 4 bit 0
BM_MSMSM40    EQU (1<<MSMSM40)       

MSMSM41       EQU 1       ; Master State Machine SubState Machine Select 4 bit 1
BM_MSMSM41    EQU (1<<MSMSM41)       

MSMSM42       EQU 2       ; Master State Machine SubState Machine Select 4 bit 2
BM_MSMSM42    EQU (1<<MSMSM42)       

MSMSM43       EQU 3       ; Master State Machine SubState Machine Select 4 bit 3
BM_MSMSM43    EQU (1<<MSMSM43)       

MSMSM50       EQU 4       ; Master State Machine SubState Machine Select 5 bit 0
BM_MSMSM50    EQU (1<<MSMSM50)       

MSMSM51       EQU 5       ; Master State Machine SubState Machine Select 5 bit 1
BM_MSMSM51    EQU (1<<MSMSM51)       

MSMSM52       EQU 6       ; Master State Machine SubState Machine Select 5 bit 2
BM_MSMSM52    EQU (1<<MSMSM52)       

MSMSM53       EQU 7       ; Master State Machine SubState Machine Select 5 bit 3
BM_MSMSM53    EQU (1<<MSMSM53)       

; MSMCR4 - Master State Machine Control Register 4
MSMSM60       EQU 0       ; Master State Machine SubState Machine Select 6 bit 0
BM_MSMSM60    EQU (1<<MSMSM60)       

MSMSM61       EQU 1       ; Master State Machine SubState Machine Select 6 bit 1
BM_MSMSM61    EQU (1<<MSMSM61)       

MSMSM62       EQU 2       ; Master State Machine SubState Machine Select 6 bit 2
BM_MSMSM62    EQU (1<<MSMSM62)       

MSMSM63       EQU 3       ; Master State Machine SubState Machine Select 6 bit 3
BM_MSMSM63    EQU (1<<MSMSM63)       

MSMSM70       EQU 4       ; Master State Machine SubState Machine Select 7 bit 0
BM_MSMSM70    EQU (1<<MSMSM70)       

MSMSM71       EQU 5       ; Master State Machine SubState Machine Select 7 bit 1
BM_MSMSM71    EQU (1<<MSMSM71)       

MSMSM72       EQU 6       ; Master State Machine SubState Machine Select 7 bit 2
BM_MSMSM72    EQU (1<<MSMSM72)       

MSMSM73       EQU 7       ; Master State Machine SubState Machine Select 7 bit 3
BM_MSMSM73    EQU (1<<MSMSM73)       

; MSMSTR - Master State Machine state register
SSMMST0       EQU 0       ; Sequencer State Machine Master State bit 0
BM_SSMMST0    EQU (1<<SSMMST0)       

SSMMST1       EQU 1       ; Sequencer State Machine Master State bit 1
BM_SSMMST1    EQU (1<<SSMMST1)       

SSMMST2       EQU 2       ; Sequencer State Machine Master State bit 2
BM_SSMMST2    EQU (1<<SSMMST2)       

SSMMST3       EQU 3       ; Sequencer State Machine Master State bit 3
BM_SSMMST3    EQU (1<<SSMMST3)       

SSMMST4       EQU 4       ; Sequencer State Machine Master State bit 4
BM_SSMMST4    EQU (1<<SSMMST4)       

; SSMCR - SSM Control Register
SSMTGE       EQU 2       ; Sequencer State Machine Tx Gauss Enable
BM_SSMTGE    EQU (1<<SSMTGE)       

SSMTPE       EQU 3       ; Sequencer State Machine Tx Preemphasis Enable
BM_SSMTPE    EQU (1<<SSMTPE)       

SSMPVE       EQU 4       ; Sequencer State Machine PV Enable
BM_SSMPVE    EQU (1<<SSMPVE)       

SSMTAE       EQU 5       ; Sequencer State Machine Tx Ask-Shaping Enable
BM_SSMTAE    EQU (1<<SSMTAE)       

; SSMFBR - SSM Filter Bandwidth Register
SSMPLDT       EQU 5       ; Sequencer State Machine PLL Lock Delay Time
BM_SSMPLDT    EQU (1<<SSMPLDT)       

; SSMIFR - SSM Interrupt Flag Register
SSMIF       EQU 0       ; Sequencer State Machine Interrupt Flag
BM_SSMIF    EQU (1<<SSMIF)       

; SSMIMR - SSM interrupt mask register
SSMIM       EQU 0       ; Sequencer State Machine Interrupt Mask
BM_SSMIM    EQU (1<<SSMIM)       

; SSMRR - SSM Run Register
SSMR       EQU 0       ; Sequencer State Machine Run
BM_SSMR    EQU (1<<SSMR)       

SSMST       EQU 1       ; Sequencer State Machine Stop
BM_SSMST    EQU (1<<SSMST)       

; SSMSR - SSM Status Register
SSMESM0       EQU 0       ; Sequencer State Machine Error State Machine bit 0
BM_SSMESM0    EQU (1<<SSMESM0)       

SSMESM1       EQU 1       ; Sequencer State Machine Error State Machine bit 1
BM_SSMESM1    EQU (1<<SSMESM1)       

SSMESM2       EQU 2       ; Sequencer State Machine Error State Machine bit 2
BM_SSMESM2    EQU (1<<SSMESM2)       

SSMESM3       EQU 3       ; Sequencer State Machine Error State Machine bit 3
BM_SSMESM3    EQU (1<<SSMESM3)       

SSMERR       EQU 7       ; Sequencer State Machine Error
BM_SSMERR    EQU (1<<SSMERR)       

; SSMSTR - SSM State Register
SSMSTA0       EQU 0       ; Sequencer State Machine State A bit 0
BM_SSMSTA0    EQU (1<<SSMSTA0)       

SSMSTA1       EQU 1       ; Sequencer State Machine State A bit 1
BM_SSMSTA1    EQU (1<<SSMSTA1)       

SSMSTA2       EQU 2       ; Sequencer State Machine State A bit 2
BM_SSMSTA2    EQU (1<<SSMSTA2)       

SSMSTA3       EQU 3       ; Sequencer State Machine State A bit 3
BM_SSMSTA3    EQU (1<<SSMSTA3)       

SSMSTA4       EQU 4       ; Sequencer State Machine State A bit 4
BM_SSMSTA4    EQU (1<<SSMSTA4)       

SSMSTA5       EQU 5       ; Sequencer State Machine State A bit 5
BM_SSMSTA5    EQU (1<<SSMSTA5)       

; ***** SUP **************************
; CALRDY - Calibration ready signature
CALRDY0       EQU 0       ; Calibration ready signature bit 0
BM_CALRDY0    EQU (1<<CALRDY0)       

CALRDY1       EQU 1       ; Calibration ready signature bit 1
BM_CALRDY1    EQU (1<<CALRDY1)       

CALRDY2       EQU 2       ; Calibration ready signature bit 2
BM_CALRDY2    EQU (1<<CALRDY2)       

CALRDY3       EQU 3       ; Calibration ready signature bit 3
BM_CALRDY3    EQU (1<<CALRDY3)       

CALRDY4       EQU 4       ; Calibration ready signature bit 4
BM_CALRDY4    EQU (1<<CALRDY4)       

CALRDY5       EQU 5       ; Calibration ready signature bit 5
BM_CALRDY5    EQU (1<<CALRDY5)       

CALRDY6       EQU 6       ; Calibration ready signature bit 6
BM_CALRDY6    EQU (1<<CALRDY6)       

CALRDY7       EQU 7       ; Calibration ready signature bit 7
BM_CALRDY7    EQU (1<<CALRDY7)       

; CALRDYLF - Calibration ready signature LFVCC
CALRDYLF0       EQU 0       ; Calibration ready signature LFVCC bit 0
BM_CALRDYLF0    EQU (1<<CALRDYLF0)       

CALRDYLF1       EQU 1       ; Calibration ready signature LFVCC bit 1
BM_CALRDYLF1    EQU (1<<CALRDYLF1)       

CALRDYLF2       EQU 2       ; Calibration ready signature LFVCC bit 2
BM_CALRDYLF2    EQU (1<<CALRDYLF2)       

CALRDYLF3       EQU 3       ; Calibration ready signature LFVCC bit 3
BM_CALRDYLF3    EQU (1<<CALRDYLF3)       

CALRDYLF4       EQU 4       ; Calibration ready signature LFVCC bit 4
BM_CALRDYLF4    EQU (1<<CALRDYLF4)       

CALRDYLF5       EQU 5       ; Calibration ready signature LFVCC bit 5
BM_CALRDYLF5    EQU (1<<CALRDYLF5)       

CALRDYLF6       EQU 6       ; Calibration ready signature LFVCC bit 6
BM_CALRDYLF6    EQU (1<<CALRDYLF6)       

CALRDYLF7       EQU 7       ; Calibration ready signature LFVCC bit 7
BM_CALRDYLF7    EQU (1<<CALRDYLF7)       

; PMTER - Power Management Test Enable Register
PMTE0       EQU 0       ; Power Management Test Enable bit 0
BM_PMTE0    EQU (1<<PMTE0)       

PMTE1       EQU 1       ; Power Management Test Enable bit 1
BM_PMTE1    EQU (1<<PMTE1)       

PMTE2       EQU 2       ; Power Management Test Enable bit 2
BM_PMTE2    EQU (1<<PMTE2)       

PMTE3       EQU 3       ; Power Management Test Enable bit 3
BM_PMTE3    EQU (1<<PMTE3)       

PMTE4       EQU 4       ; Power Management Test Enable bit 4
BM_PMTE4    EQU (1<<PMTE4)       

PMTE5       EQU 5       ; Power Management Test Enable bit 5
BM_PMTE5    EQU (1<<PMTE5)       

PMTE6       EQU 6       ; Power Management Test Enable bit 6
BM_PMTE6    EQU (1<<PMTE6)       

PMTE7       EQU 7       ; Power Management Test Enable bit 7
BM_PMTE7    EQU (1<<PMTE7)       

; RCTCAL - RC oscillator Temperature Compensation register
FRCTC       EQU 0       ; FRC Oscillator Temperature Compensation bit
BM_FRCTC    EQU (1<<FRCTC)       

MRCTC0       EQU 1       ; Medium frequency RC Oscillator Temperature Compensation bit 0
BM_MRCTC0    EQU (1<<MRCTC0)       

MRCTC1       EQU 2       ; Medium frequency RC Oscillator Temperature Compensation bit 1
BM_MRCTC1    EQU (1<<MRCTC1)       

MRCTC2       EQU 3       ; Medium frequency RC Oscillator Temperature Compensation bit 2
BM_MRCTC2    EQU (1<<MRCTC2)       

DI_MRCBG       EQU 4       ; Disable MRC Oscillator Bandgap
BM_DI_MRCBG    EQU (1<<DI_MRCBG)       

; SUPCA1 - Supply calibration register 1
PV22       EQU 2       ; Power Amplifier Voltage 2.2V
BM_PV22    EQU (1<<PV22)       

PVDIC       EQU 3       ; Power Amplifier Regulator Double Internal Current
BM_PVDIC    EQU (1<<PVDIC)       

PVCAL0       EQU 4       ; Power Amplifier Regulator Calibration bit 0
BM_PVCAL0    EQU (1<<PVCAL0)       

PVCAL1       EQU 5       ; Power Amplifier Regulator Calibration bit 1
BM_PVCAL1    EQU (1<<PVCAL1)       

PVCAL2       EQU 6       ; Power Amplifier Regulator Calibration bit 2
BM_PVCAL2    EQU (1<<PVCAL2)       

PVCAL3       EQU 7       ; Power Amplifier Regulator Calibration bit 3
BM_PVCAL3    EQU (1<<PVCAL3)       

; SUPCA10 - Supply calibration register 10
POWMAN0       EQU 0       ; Power Management Spare bits bit 0
BM_POWMAN0    EQU (1<<POWMAN0)       

POWMAN1       EQU 1       ; Power Management Spare bits bit 1
BM_POWMAN1    EQU (1<<POWMAN1)       

POWMAN2       EQU 2       ; Power Management Spare bits bit 2
BM_POWMAN2    EQU (1<<POWMAN2)       

POWMAN3       EQU 3       ; Power Management Spare bits bit 3
BM_POWMAN3    EQU (1<<POWMAN3)       

POWMAN4       EQU 4       ; Power Management Spare bits bit 4
BM_POWMAN4    EQU (1<<POWMAN4)       

; SUPCA2 - Supply calibration register 2
BGCAL0       EQU 0       ; Band Gap Calibration bit 0
BM_BGCAL0    EQU (1<<BGCAL0)       

BGCAL1       EQU 1       ; Band Gap Calibration bit 1
BM_BGCAL1    EQU (1<<BGCAL1)       

BGCAL2       EQU 2       ; Band Gap Calibration bit 2
BM_BGCAL2    EQU (1<<BGCAL2)       

BGCAL3       EQU 3       ; Band Gap Calibration bit 3
BM_BGCAL3    EQU (1<<BGCAL3)       

; SUPCA3 - Supply calibration register 3
ACAL0       EQU 0       ; AVCC Regulator Output Voltage Calibration bit 0
BM_ACAL0    EQU (1<<ACAL0)       

ACAL1       EQU 1       ; AVCC Regulator Output Voltage Calibration bit 1
BM_ACAL1    EQU (1<<ACAL1)       

ACAL2       EQU 2       ; AVCC Regulator Output Voltage Calibration bit 2
BM_ACAL2    EQU (1<<ACAL2)       

ACAL3       EQU 3       ; AVCC Regulator Output Voltage Calibration bit 3
BM_ACAL3    EQU (1<<ACAL3)       

ACAL4       EQU 4       ; AVCC Regulator Output Voltage Calibration bit 4
BM_ACAL4    EQU (1<<ACAL4)       

ACAL5       EQU 5       ; AVCC Regulator Output Voltage Calibration bit 5
BM_ACAL5    EQU (1<<ACAL5)       

ACAL6       EQU 6       ; AVCC Regulator Output Voltage Calibration bit 6
BM_ACAL6    EQU (1<<ACAL6)       

ACAL7       EQU 7       ; AVCC Regulator Output Voltage Calibration bit 7
BM_ACAL7    EQU (1<<ACAL7)       

; SUPCA4 - Supply calibration register 4
ICONST0       EQU 0       ; ICONST Constant current of bandgap bit 0
BM_ICONST0    EQU (1<<ICONST0)       

ICONST1       EQU 1       ; ICONST Constant current of bandgap bit 1
BM_ICONST1    EQU (1<<ICONST1)       

ICONST2       EQU 2       ; ICONST Constant current of bandgap bit 2
BM_ICONST2    EQU (1<<ICONST2)       

ICONST3       EQU 3       ; ICONST Constant current of bandgap bit 3
BM_ICONST3    EQU (1<<ICONST3)       

ICONST4       EQU 4       ; ICONST Constant current of bandgap bit 4
BM_ICONST4    EQU (1<<ICONST4)       

ICONST5       EQU 5       ; ICONST Constant current of bandgap bit 5
BM_ICONST5    EQU (1<<ICONST5)       

; SUPCA5 - Supply calibration register 5
IPTAT0       EQU 0       ; IPTAT current of bandgap bit 0
BM_IPTAT0    EQU (1<<IPTAT0)       

IPTAT1       EQU 1       ; IPTAT current of bandgap bit 1
BM_IPTAT1    EQU (1<<IPTAT1)       

IPTAT2       EQU 2       ; IPTAT current of bandgap bit 2
BM_IPTAT2    EQU (1<<IPTAT2)       

IPTAT3       EQU 3       ; IPTAT current of bandgap bit 3
BM_IPTAT3    EQU (1<<IPTAT3)       

IPTAT4       EQU 4       ; IPTAT current of bandgap bit 4
BM_IPTAT4    EQU (1<<IPTAT4)       

IPTAT5       EQU 5       ; IPTAT current of bandgap bit 5
BM_IPTAT5    EQU (1<<IPTAT5)       

; SUPCA6 - Supply calibration register 6
VBGTR0       EQU 0       ; Threshold voltage of bandgap bit 0
BM_VBGTR0    EQU (1<<VBGTR0)       

VBGTR1       EQU 1       ; Threshold voltage of bandgap bit 1
BM_VBGTR1    EQU (1<<VBGTR1)       

VBGTR2       EQU 2       ; Threshold voltage of bandgap bit 2
BM_VBGTR2    EQU (1<<VBGTR2)       

VBGTR3       EQU 3       ; Threshold voltage of bandgap bit 3
BM_VBGTR3    EQU (1<<VBGTR3)       

VBGTR4       EQU 4       ; Threshold voltage of bandgap bit 4
BM_VBGTR4    EQU (1<<VBGTR4)       

VBGTR5       EQU 5       ; Threshold voltage of bandgap bit 5
BM_VBGTR5    EQU (1<<VBGTR5)       

VBGTR6       EQU 6       ; Threshold voltage of bandgap bit 6
BM_VBGTR6    EQU (1<<VBGTR6)       

VBGTR7       EQU 7       ; Threshold voltage of bandgap bit 7
BM_VBGTR7    EQU (1<<VBGTR7)       

; SUPCA7 - Supply calibration register 7
VCCCAL0       EQU 0       ; LFVCC and DVCC Output Voltage Calibration bit 0
BM_VCCCAL0    EQU (1<<VCCCAL0)       

VCCCAL1       EQU 1       ; LFVCC and DVCC Output Voltage Calibration bit 1
BM_VCCCAL1    EQU (1<<VCCCAL1)       

VCCCAL2       EQU 2       ; LFVCC and DVCC Output Voltage Calibration bit 2
BM_VCCCAL2    EQU (1<<VCCCAL2)       

LFVCCBD0       EQU 3       ; VREF voltage for LFVCC Brown-out-Detector bit 0
BM_LFVCCBD0    EQU (1<<LFVCCBD0)       

LFVCCBD1       EQU 4       ; VREF voltage for LFVCC Brown-out-Detector bit 1
BM_LFVCCBD1    EQU (1<<LFVCCBD1)       

LFVCCBD2       EQU 5       ; VREF voltage for LFVCC Brown-out-Detector bit 2
BM_LFVCCBD2    EQU (1<<LFVCCBD2)       

; SUPCA8 - Supply calibration register 8
VSWBD0       EQU 0       ; VSIG voltage for LFVCC Brown-out-Detector bit 0
BM_VSWBD0    EQU (1<<VSWBD0)       

VSWBD1       EQU 1       ; VSIG voltage for LFVCC Brown-out-Detector bit 1
BM_VSWBD1    EQU (1<<VSWBD1)       

VSWBD2       EQU 2       ; VSIG voltage for LFVCC Brown-out-Detector bit 2
BM_VSWBD2    EQU (1<<VSWBD2)       

DVCCBD0       EQU 3       ; VREF voltage for DVCC Brown-out-Detector bit 0
BM_DVCCBD0    EQU (1<<DVCCBD0)       

DVCCBD1       EQU 4       ; VREF voltage for DVCC Brown-out-Detector bit 1
BM_DVCCBD1    EQU (1<<DVCCBD1)       

DVCCBD2       EQU 5       ; VREF voltage for DVCC Brown-out-Detector bit 2
BM_DVCCBD2    EQU (1<<DVCCBD2)       

; SUPCA9 - Supply calibration register 9
VMEM0       EQU 0       ; VMEM ldo voltage bit 0
BM_VMEM0    EQU (1<<VMEM0)       

VMEM1       EQU 1       ; VMEM ldo voltage bit 1
BM_VMEM1    EQU (1<<VMEM1)       

VMEM2       EQU 2       ; VMEM ldo voltage bit 2
BM_VMEM2    EQU (1<<VMEM2)       

VMEM3       EQU 3       ; VMEM ldo voltage bit 3
BM_VMEM3    EQU (1<<VMEM3)       

VMEM4       EQU 4       ; VMEM ldo voltage bit 4
BM_VMEM4    EQU (1<<VMEM4)       

VMEM5       EQU 5       ; VMEM ldo voltage bit 5
BM_VMEM5    EQU (1<<VMEM5)       

VMEM6       EQU 6       ; VMEM ldo voltage bit 6
BM_VMEM6    EQU (1<<VMEM6)       

VMEM7       EQU 7       ; VMEM ldo voltage bit 7
BM_VMEM7    EQU (1<<VMEM7)       

; SUPCR - Supply Control Register
AVCCRM       EQU 0       ; AVCC Reset Interrupt Mask
BM_AVCCRM    EQU (1<<AVCCRM)       

AVCCLM       EQU 1       ; AVCC Low Interrupt Mask
BM_AVCCLM    EQU (1<<AVCCLM)       

PVEN       EQU 2       ; Power amplifier Voltage supply Enable
BM_PVEN    EQU (1<<PVEN)       

AVDIC       EQU 3       ; AVCC Double Internal Current
BM_AVDIC    EQU (1<<AVDIC)       

AVEN       EQU 4       ; AVCC Enable
BM_AVEN    EQU (1<<AVEN)       

DVHEN       EQU 5       ; DVCC High Current Mode Enable
BM_DVHEN    EQU (1<<DVHEN)       

VMRESM       EQU 6       ; VMEM Reset Mask
BM_VMRESM    EQU (1<<VMRESM)       

VMEMEN       EQU 7       ; Memory Voltage Regulator Enable
BM_VMEMEN    EQU (1<<VMEMEN)       

; SUPFR - Supply Interrupt Flag Register
AVCCRF       EQU 0       ; AVCC reset interrupt flag
BM_AVCCRF    EQU (1<<AVCCRF)       

AVCCLF       EQU 1       ; AVCC low interrupt flag
BM_AVCCLF    EQU (1<<AVCCLF)       

; VMCR - Voltage Monitor Control Register
VMLS0       EQU 0       ; Voltage Monitor Level Select bit 0
BM_VMLS0    EQU (1<<VMLS0)       

VMLS1       EQU 1       ; Voltage Monitor Level Select bit 1
BM_VMLS1    EQU (1<<VMLS1)       

VMLS2       EQU 2       ; Voltage Monitor Level Select bit 2
BM_VMLS2    EQU (1<<VMLS2)       

VMLS3       EQU 3       ; Voltage Monitor Level Select bit 3
BM_VMLS3    EQU (1<<VMLS3)       

VMIM       EQU 4       ; Voltage Monitor Interrupt Mask
BM_VMIM    EQU (1<<VMIM)       

VMPS0       EQU 5       ; Voltage Monitor Power Supply Select 0
BM_VMPS0    EQU (1<<VMPS0)       

VMPS1       EQU 6       ; Voltage Monitor Power Supply Select 1
BM_VMPS1    EQU (1<<VMPS1)       

VMRS       EQU 7       ; Voltage Monitor Range Select
BM_VMRS    EQU (1<<VMRS)       

; VMSCR - Voltage Monitor Status and Control Register
VMF       EQU 0       ; Voltage Monitor Flag
BM_VMF    EQU (1<<VMF)       

VMDIH       EQU 1       ; Voltage Monitor Disable Hysteresis
BM_VMDIH    EQU (1<<VMDIH)       

; ***** TIMER0_WDT *******************
; T0CR - Timer0 Control Register
T0PS0       EQU 0       ; Timer0 Prescaler Select bit 0
BM_T0PS0    EQU (1<<T0PS0)       

T0PS1       EQU 1       ; Timer0 Prescaler Select bit 1
BM_T0PS1    EQU (1<<T0PS1)       

T0PS2       EQU 2       ; Timer0 Prescaler Select bit 2
BM_T0PS2    EQU (1<<T0PS2)       

T0IE       EQU 3       ; Timer0 Interrupt Enable
BM_T0IE    EQU (1<<T0IE)       

T0PR       EQU 4       ; Timer0 Prescaler Reset
BM_T0PR    EQU (1<<T0PR)       

; T0IFR - Timer0 Interrupt Flag Register
T0F       EQU 0       ; Timer0 Flag
BM_T0F    EQU (1<<T0F)       

; WDTCR - Watchdog Timer0 control Register
WDPS0       EQU 0       ; Watchdog Prescaler Select bit 0
BM_WDPS0    EQU (1<<WDPS0)       

WDPS1       EQU 1       ; Watchdog Prescaler Select bit 1
BM_WDPS1    EQU (1<<WDPS1)       

WDPS2       EQU 2       ; Watchdog Prescaler Select bit 2
BM_WDPS2    EQU (1<<WDPS2)       

WDE       EQU 3       ; Watchdog System Reset Enable
BM_WDE    EQU (1<<WDE)       

WDCE       EQU 4       ; Watchdog Change Enable
BM_WDCE    EQU (1<<WDCE)       

; ***** TIMER1 ***********************
; T1CNT - Timer1 Counter Register
T1CNT0       EQU 0       ; Timer1 Counter bit 0
BM_T1CNT0    EQU (1<<T1CNT0)       

T1CNT1       EQU 1       ; Timer1 Counter bit 1
BM_T1CNT1    EQU (1<<T1CNT1)       

T1CNT2       EQU 2       ; Timer1 Counter bit 2
BM_T1CNT2    EQU (1<<T1CNT2)       

T1CNT3       EQU 3       ; Timer1 Counter bit 3
BM_T1CNT3    EQU (1<<T1CNT3)       

T1CNT4       EQU 4       ; Timer1 Counter bit 4
BM_T1CNT4    EQU (1<<T1CNT4)       

T1CNT5       EQU 5       ; Timer1 Counter bit 5
BM_T1CNT5    EQU (1<<T1CNT5)       

T1CNT6       EQU 6       ; Timer1 Counter bit 6
BM_T1CNT6    EQU (1<<T1CNT6)       

T1CNT7       EQU 7       ; Timer1 Counter bit 7
BM_T1CNT7    EQU (1<<T1CNT7)       

; T1COR - Timer1 Compare Register
T1COR0       EQU 0       ; Timer1 Compare bit 0
BM_T1COR0    EQU (1<<T1COR0)       

T1COR1       EQU 1       ; Timer1 Compare bit 1
BM_T1COR1    EQU (1<<T1COR1)       

T1COR2       EQU 2       ; Timer1 Compare bit 2
BM_T1COR2    EQU (1<<T1COR2)       

T1COR3       EQU 3       ; Timer1 Compare bit 3
BM_T1COR3    EQU (1<<T1COR3)       

T1COR4       EQU 4       ; Timer1 Compare bit 4
BM_T1COR4    EQU (1<<T1COR4)       

T1COR5       EQU 5       ; Timer1 Compare bit 5
BM_T1COR5    EQU (1<<T1COR5)       

T1COR6       EQU 6       ; Timer1 Compare bit 6
BM_T1COR6    EQU (1<<T1COR6)       

T1COR7       EQU 7       ; Timer1 Compare bit 7
BM_T1COR7    EQU (1<<T1COR7)       

; T1CR - Timer1 control Register
T1OTM       EQU 0       ; Timer1 Overflow Toggle Mask
BM_T1OTM    EQU (1<<T1OTM)       

T1CTM       EQU 1       ; Timer1 Compare Toggle Mask
BM_T1CTM    EQU (1<<T1CTM)       

T1CRM       EQU 2       ; Timer1 Compare Reset Mask
BM_T1CRM    EQU (1<<T1CRM)       

T1TOP       EQU 4       ; Timer1 Toggle Output Preset
BM_T1TOP    EQU (1<<T1TOP)       

T1RES       EQU 5       ; Timer1 Reset
BM_T1RES    EQU (1<<T1RES)       

T1TOS       EQU 6       ; Timer1 Toggle with Start
BM_T1TOS    EQU (1<<T1TOS)       

T1ENA       EQU 7       ; Timer1 Enable
BM_T1ENA    EQU (1<<T1ENA)       

; T1IFR - Timer1 Interrupt Flag Register
T1OFF       EQU 0       ; Timer1 Overflow Flag
BM_T1OFF    EQU (1<<T1OFF)       

T1COF       EQU 1       ; Timer1 Compare Flag
BM_T1COF    EQU (1<<T1COF)       

; T1IMR - Timer1 Interrupt Mask Register
T1OIM       EQU 0       ; Timer1 Overflow Interrupt Mask
BM_T1OIM    EQU (1<<T1OIM)       

T1CIM       EQU 1       ; Timer1 Compare Interrupt Mask
BM_T1CIM    EQU (1<<T1CIM)       

; T1MR - Timer1 Mode Register
T1CS0       EQU 0       ; Timer1 Clock Select bit 0
BM_T1CS0    EQU (1<<T1CS0)       

T1CS1       EQU 1       ; Timer1 Clock Select bit 1
BM_T1CS1    EQU (1<<T1CS1)       

T1PS0       EQU 2       ; Timer1 Prescaler Select bit 0
BM_T1PS0    EQU (1<<T1PS0)       

T1PS1       EQU 3       ; Timer1 Prescaler Select bit 1
BM_T1PS1    EQU (1<<T1PS1)       

T1PS2       EQU 4       ; Timer1 Prescaler Select bit 2
BM_T1PS2    EQU (1<<T1PS2)       

T1PS3       EQU 5       ; Timer1 Prescaler Select bit 3
BM_T1PS3    EQU (1<<T1PS3)       

T1DC0       EQU 6       ; Timer1 Duty Cycle bit 0
BM_T1DC0    EQU (1<<T1DC0)       

T1DC1       EQU 7       ; Timer1 Duty Cycle bit 1
BM_T1DC1    EQU (1<<T1DC1)       

; ***** TIMER2 ***********************
; T2CNT - Timer2 Counter Register
T2CNT0       EQU 0       ; Timer2 Counter bit 0
BM_T2CNT0    EQU (1<<T2CNT0)       

T2CNT1       EQU 1       ; Timer2 Counter bit 1
BM_T2CNT1    EQU (1<<T2CNT1)       

T2CNT2       EQU 2       ; Timer2 Counter bit 2
BM_T2CNT2    EQU (1<<T2CNT2)       

T2CNT3       EQU 3       ; Timer2 Counter bit 3
BM_T2CNT3    EQU (1<<T2CNT3)       

T2CNT4       EQU 4       ; Timer2 Counter bit 4
BM_T2CNT4    EQU (1<<T2CNT4)       

T2CNT5       EQU 5       ; Timer2 Counter bit 5
BM_T2CNT5    EQU (1<<T2CNT5)       

T2CNT6       EQU 6       ; Timer2 Counter bit 6
BM_T2CNT6    EQU (1<<T2CNT6)       

T2CNT7       EQU 7       ; Timer2 Counter bit 7
BM_T2CNT7    EQU (1<<T2CNT7)       

; T2COR - Timer2 Compare Register
T2COR0       EQU 0       ; Timer2 Compare bit 0
BM_T2COR0    EQU (1<<T2COR0)       

T2COR1       EQU 1       ; Timer2 Compare bit 1
BM_T2COR1    EQU (1<<T2COR1)       

T2COR2       EQU 2       ; Timer2 Compare bit 2
BM_T2COR2    EQU (1<<T2COR2)       

T2COR3       EQU 3       ; Timer2 Compare bit 3
BM_T2COR3    EQU (1<<T2COR3)       

T2COR4       EQU 4       ; Timer2 Compare bit 4
BM_T2COR4    EQU (1<<T2COR4)       

T2COR5       EQU 5       ; Timer2 Compare bit 5
BM_T2COR5    EQU (1<<T2COR5)       

T2COR6       EQU 6       ; Timer2 Compare bit 6
BM_T2COR6    EQU (1<<T2COR6)       

T2COR7       EQU 7       ; Timer2 Compare bit 7
BM_T2COR7    EQU (1<<T2COR7)       

; T2CR - Timer2 Control Register
T2OTM       EQU 0       ; Timer2 Overflow Toggle Mask
BM_T2OTM    EQU (1<<T2OTM)       

T2CTM       EQU 1       ; Timer2 Compare Toggle Mask
BM_T2CTM    EQU (1<<T2CTM)       

T2CRM       EQU 2       ; Timer2 Compare Reset Mask
BM_T2CRM    EQU (1<<T2CRM)       

T2TOP       EQU 4       ; Timer2 Toggle Output Preset
BM_T2TOP    EQU (1<<T2TOP)       

T2RES       EQU 5       ; Timer2 Reset
BM_T2RES    EQU (1<<T2RES)       

T2TOS       EQU 6       ; Timer2 Toggle with Start
BM_T2TOS    EQU (1<<T2TOS)       

T2ENA       EQU 7       ; Timer2 Enable
BM_T2ENA    EQU (1<<T2ENA)       

; T2IFR - Timer2 Interrupt Flag Register
T2OFF       EQU 0       ; Timer2 Overflow Flag
BM_T2OFF    EQU (1<<T2OFF)       

T2COF       EQU 1       ; Timer2 Compare Flag
BM_T2COF    EQU (1<<T2COF)       

; T2IMR - Timer2 Interrupt Mask Register
T2OIM       EQU 0       ; Timer2 Overflow Interrupt Mask
BM_T2OIM    EQU (1<<T2OIM)       

T2CIM       EQU 1       ; Timer2 Compare Interrupt Mask
BM_T2CIM    EQU (1<<T2CIM)       

; T2MR - Timer2 Mode Register
T2CS0       EQU 0       ; Timer2 Clock Select bit 0
BM_T2CS0    EQU (1<<T2CS0)       

T2CS1       EQU 1       ; Timer2 Clock Select bit 1
BM_T2CS1    EQU (1<<T2CS1)       

T2PS0       EQU 2       ; Timer2 Prescaler Select bit 0
BM_T2PS0    EQU (1<<T2PS0)       

T2PS1       EQU 3       ; Timer2 Prescaler Select bit 1
BM_T2PS1    EQU (1<<T2PS1)       

T2PS2       EQU 4       ; Timer2 Prescaler Select bit 2
BM_T2PS2    EQU (1<<T2PS2)       

T2PS3       EQU 5       ; Timer2 Prescaler Select bit 3
BM_T2PS3    EQU (1<<T2PS3)       

T2DC0       EQU 6       ; Timer2 Duty Cycle bit 0
BM_T2DC0    EQU (1<<T2DC0)       

T2DC1       EQU 7       ; Timer2 Duty Cycle bit 1
BM_T2DC1    EQU (1<<T2DC1)       

; ***** TIMER3 ***********************
; T3CNTH - Timer3 counter High Byte Register
T3CNT8       EQU 0       ; Timer3 counter bit 8
BM_T3CNT8    EQU (1<<T3CNT8)       

T3CNT9       EQU 1       ; Timer3 counter bit 9
BM_T3CNT9    EQU (1<<T3CNT9)       

T3CNT10       EQU 2       ; Timer3 counter bit 10
BM_T3CNT10    EQU (1<<T3CNT10)       

T3CNT11       EQU 3       ; Timer3 counter bit 11
BM_T3CNT11    EQU (1<<T3CNT11)       

T3CNT12       EQU 4       ; Timer3 counter bit 12
BM_T3CNT12    EQU (1<<T3CNT12)       

T3CNT13       EQU 5       ; Timer3 counter bit 13
BM_T3CNT13    EQU (1<<T3CNT13)       

T3CNT14       EQU 6       ; Timer3 counter bit 14
BM_T3CNT14    EQU (1<<T3CNT14)       

T3CNT15       EQU 7       ; Timer3 counter bit 15
BM_T3CNT15    EQU (1<<T3CNT15)       

; T3CNTL - Timer3 counter Low Byte Register
T3CNT0       EQU 0       ; Timer3 counter bit 0
BM_T3CNT0    EQU (1<<T3CNT0)       

T3CNT1       EQU 1       ; Timer3 counter bit 1
BM_T3CNT1    EQU (1<<T3CNT1)       

T3CNT2       EQU 2       ; Timer3 counter bit 2
BM_T3CNT2    EQU (1<<T3CNT2)       

T3CNT3       EQU 3       ; Timer3 counter bit 3
BM_T3CNT3    EQU (1<<T3CNT3)       

T3CNT4       EQU 4       ; Timer3 counter bit 4
BM_T3CNT4    EQU (1<<T3CNT4)       

T3CNT5       EQU 5       ; Timer3 counter bit 5
BM_T3CNT5    EQU (1<<T3CNT5)       

T3CNT6       EQU 6       ; Timer3 counter bit 6
BM_T3CNT6    EQU (1<<T3CNT6)       

T3CNT7       EQU 7       ; Timer3 counter bit 7
BM_T3CNT7    EQU (1<<T3CNT7)       

; T3CORH - Timer3 compare High Byte Register
T3COR8       EQU 0       ; Timer3 compare bit 8
BM_T3COR8    EQU (1<<T3COR8)       

T3COR9       EQU 1       ; Timer3 compare bit 9
BM_T3COR9    EQU (1<<T3COR9)       

T3COR10       EQU 2       ; Timer3 compare bit 10
BM_T3COR10    EQU (1<<T3COR10)       

T3COR11       EQU 3       ; Timer3 compare bit 11
BM_T3COR11    EQU (1<<T3COR11)       

T3COR12       EQU 4       ; Timer3 compare bit 12
BM_T3COR12    EQU (1<<T3COR12)       

T3COR13       EQU 5       ; Timer3 compare bit 13
BM_T3COR13    EQU (1<<T3COR13)       

T3COR14       EQU 6       ; Timer3 compare bit 14
BM_T3COR14    EQU (1<<T3COR14)       

T3COR15       EQU 7       ; Timer3 compare bit 15
BM_T3COR15    EQU (1<<T3COR15)       

; T3CORL - Timer3 compare Low Byte Register
T3COR0       EQU 0       ; Timer3 compare bit 0
BM_T3COR0    EQU (1<<T3COR0)       

T3COR1       EQU 1       ; Timer3 compare bit 1
BM_T3COR1    EQU (1<<T3COR1)       

T3COR2       EQU 2       ; Timer3 compare bit 2
BM_T3COR2    EQU (1<<T3COR2)       

T3COR3       EQU 3       ; Timer3 compare bit 3
BM_T3COR3    EQU (1<<T3COR3)       

T3COR4       EQU 4       ; Timer3 compare bit 4
BM_T3COR4    EQU (1<<T3COR4)       

T3COR5       EQU 5       ; Timer3 compare bit 5
BM_T3COR5    EQU (1<<T3COR5)       

T3COR6       EQU 6       ; Timer3 compare bit 6
BM_T3COR6    EQU (1<<T3COR6)       

T3COR7       EQU 7       ; Timer3 compare bit 7
BM_T3COR7    EQU (1<<T3COR7)       

; T3CR - Timer3 control Register
T3OTM       EQU 0       ; Timer3 Overflow Toggle Mask
BM_T3OTM    EQU (1<<T3OTM)       

T3CTM       EQU 1       ; Timer3 Compare Toggle Mask
BM_T3CTM    EQU (1<<T3CTM)       

T3CRM       EQU 2       ; Timer3 Compare Reset Mask
BM_T3CRM    EQU (1<<T3CRM)       

T3CPRM       EQU 3       ; Timer3 CaPture Reset Mask
BM_T3CPRM    EQU (1<<T3CPRM)       

T3TOP       EQU 4       ; Timer3 Toggle Output Preset
BM_T3TOP    EQU (1<<T3TOP)       

T3RES       EQU 5       ; Timer3 Reset
BM_T3RES    EQU (1<<T3RES)       

T3TOS       EQU 6       ; Timer3 Toggle with Start
BM_T3TOS    EQU (1<<T3TOS)       

T3ENA       EQU 7       ; Timer3 Enable
BM_T3ENA    EQU (1<<T3ENA)       

; T3ICRH - Timer3 input capture High Byte Register
T3ICR8       EQU 0       ; Timer3 input capture bit 8
BM_T3ICR8    EQU (1<<T3ICR8)       

T3ICR9       EQU 1       ; Timer3 input capture bit 9
BM_T3ICR9    EQU (1<<T3ICR9)       

T3ICR10       EQU 2       ; Timer3 input capture bit 10
BM_T3ICR10    EQU (1<<T3ICR10)       

T3ICR11       EQU 3       ; Timer3 input capture bit 11
BM_T3ICR11    EQU (1<<T3ICR11)       

T3ICR12       EQU 4       ; Timer3 input capture bit 12
BM_T3ICR12    EQU (1<<T3ICR12)       

T3ICR13       EQU 5       ; Timer3 input capture bit 13
BM_T3ICR13    EQU (1<<T3ICR13)       

T3ICR14       EQU 6       ; Timer3 input capture bit 14
BM_T3ICR14    EQU (1<<T3ICR14)       

T3ICR15       EQU 7       ; Timer3 input capture bit 15
BM_T3ICR15    EQU (1<<T3ICR15)       

; T3ICRL - Timer3 input capture Low Byte Register
T3ICR0       EQU 0       ; Timer3 input capture bit 0
BM_T3ICR0    EQU (1<<T3ICR0)       

T3ICR1       EQU 1       ; Timer3 input capture bit 1
BM_T3ICR1    EQU (1<<T3ICR1)       

T3ICR2       EQU 2       ; Timer3 input capture bit 2
BM_T3ICR2    EQU (1<<T3ICR2)       

T3ICR3       EQU 3       ; Timer3 input capture bit 3
BM_T3ICR3    EQU (1<<T3ICR3)       

T3ICR4       EQU 4       ; Timer3 input capture bit 4
BM_T3ICR4    EQU (1<<T3ICR4)       

T3ICR5       EQU 5       ; Timer3 input capture bit 5
BM_T3ICR5    EQU (1<<T3ICR5)       

T3ICR6       EQU 6       ; Timer3 input capture bit 6
BM_T3ICR6    EQU (1<<T3ICR6)       

T3ICR7       EQU 7       ; Timer3 input capture bit 7
BM_T3ICR7    EQU (1<<T3ICR7)       

; T3IFR - Timer3 interrupt flag Register
T3OFF       EQU 0       ; Timer3 OverFlow Flag
BM_T3OFF    EQU (1<<T3OFF)       

T3COF       EQU 1       ; Timer3 Compare Flag
BM_T3COF    EQU (1<<T3COF)       

T3ICF       EQU 2       ; Timer3 Input Capture Flag
BM_T3ICF    EQU (1<<T3ICF)       

; T3IMR - Timer3 interrupt mask Register
T3OIM       EQU 0       ; Timer3 Overflow Interrupt Mask
BM_T3OIM    EQU (1<<T3OIM)       

T3CIM       EQU 1       ; Timer3 Compare Interrupt Mask
BM_T3CIM    EQU (1<<T3CIM)       

T3CPIM       EQU 2       ; Timer3 Capture Interrupt Mask
BM_T3CPIM    EQU (1<<T3CPIM)       

; T3MRA - Timer3 mode Register
T3CS0       EQU 0       ; Timer3 Clock Select bit 0
BM_T3CS0    EQU (1<<T3CS0)       

T3CS1       EQU 1       ; Timer3 Clock Select bit 1
BM_T3CS1    EQU (1<<T3CS1)       

T3PS0       EQU 2       ; Timer3 Prescaler Select bit 0
BM_T3PS0    EQU (1<<T3PS0)       

T3PS1       EQU 3       ; Timer3 Prescaler Select bit 1
BM_T3PS1    EQU (1<<T3PS1)       

T3PS2       EQU 4       ; Timer3 Prescaler Select bit 2
BM_T3PS2    EQU (1<<T3PS2)       

; T3MRB - Timer3 mode Register
T3SCE       EQU 1       ; Timer3 Software Capture Enable
BM_T3SCE    EQU (1<<T3SCE)       

T3CNC       EQU 2       ; Timer3 input Capture Noise Canceller
BM_T3CNC    EQU (1<<T3CNC)       

T3CE0       EQU 3       ; Timer3 Capture Edge select bit 0
BM_T3CE0    EQU (1<<T3CE0)       

T3CE1       EQU 4       ; Timer3 Capture Edge select bit 1
BM_T3CE1    EQU (1<<T3CE1)       

T3ICS0       EQU 5       ; Timer3 Input Capture Select bit 0
BM_T3ICS0    EQU (1<<T3ICS0)       

T3ICS1       EQU 6       ; Timer3 Input Capture Select bit 1
BM_T3ICS1    EQU (1<<T3ICS1)       

T3ICS2       EQU 7       ; Timer3 Input Capture Select bit 2
BM_T3ICS2    EQU (1<<T3ICS2)       

; ***** TIMER4 ***********************
; T4CNTH - Timer4 counter High Byte Register
T4CNT8       EQU 0       ; Timer4 counter bit 8
BM_T4CNT8    EQU (1<<T4CNT8)       

T4CNT9       EQU 1       ; Timer4 counter bit 9
BM_T4CNT9    EQU (1<<T4CNT9)       

T4CNT10       EQU 2       ; Timer4 counter bit 10
BM_T4CNT10    EQU (1<<T4CNT10)       

T4CNT11       EQU 3       ; Timer4 counter bit 11
BM_T4CNT11    EQU (1<<T4CNT11)       

T4CNT12       EQU 4       ; Timer4 counter bit 12
BM_T4CNT12    EQU (1<<T4CNT12)       

T4CNT13       EQU 5       ; Timer4 counter bit 13
BM_T4CNT13    EQU (1<<T4CNT13)       

T4CNT14       EQU 6       ; Timer4 counter bit 14
BM_T4CNT14    EQU (1<<T4CNT14)       

T4CNT15       EQU 7       ; Timer4 counter bit 15
BM_T4CNT15    EQU (1<<T4CNT15)       

; T4CNTL - Timer4 counter Low Byte Register
T4CNT0       EQU 0       ; Timer4 counter bit 0
BM_T4CNT0    EQU (1<<T4CNT0)       

T4CNT1       EQU 1       ; Timer4 counter bit 1
BM_T4CNT1    EQU (1<<T4CNT1)       

T4CNT2       EQU 2       ; Timer4 counter bit 2
BM_T4CNT2    EQU (1<<T4CNT2)       

T4CNT3       EQU 3       ; Timer4 counter bit 3
BM_T4CNT3    EQU (1<<T4CNT3)       

T4CNT4       EQU 4       ; Timer4 counter bit 4
BM_T4CNT4    EQU (1<<T4CNT4)       

T4CNT5       EQU 5       ; Timer4 counter bit 5
BM_T4CNT5    EQU (1<<T4CNT5)       

T4CNT6       EQU 6       ; Timer4 counter bit 6
BM_T4CNT6    EQU (1<<T4CNT6)       

T4CNT7       EQU 7       ; Timer4 counter bit 7
BM_T4CNT7    EQU (1<<T4CNT7)       

; T4CORH - Timer4 compare high Byte Register
T4COR8       EQU 0       ; Timer4 compare bit 8
BM_T4COR8    EQU (1<<T4COR8)       

T4COR9       EQU 1       ; Timer4 compare bit 9
BM_T4COR9    EQU (1<<T4COR9)       

T4COR10       EQU 2       ; Timer4 compare bit 10
BM_T4COR10    EQU (1<<T4COR10)       

T4COR11       EQU 3       ; Timer4 compare bit 11
BM_T4COR11    EQU (1<<T4COR11)       

T4COR12       EQU 4       ; Timer4 compare bit 12
BM_T4COR12    EQU (1<<T4COR12)       

T4COR13       EQU 5       ; Timer4 compare bit 13
BM_T4COR13    EQU (1<<T4COR13)       

T4COR14       EQU 6       ; Timer4 compare bit 14
BM_T4COR14    EQU (1<<T4COR14)       

T4COR15       EQU 7       ; Timer4 compare bit 15
BM_T4COR15    EQU (1<<T4COR15)       

; T4CORL - Timer4 compare Low Byte Register
T4COR0       EQU 0       ; Timer4 compare bit 0
BM_T4COR0    EQU (1<<T4COR0)       

T4COR1       EQU 1       ; Timer4 compare bit 1
BM_T4COR1    EQU (1<<T4COR1)       

T4COR2       EQU 2       ; Timer4 compare bit 2
BM_T4COR2    EQU (1<<T4COR2)       

T4COR3       EQU 3       ; Timer4 compare bit 3
BM_T4COR3    EQU (1<<T4COR3)       

T4COR4       EQU 4       ; Timer4 compare bit 4
BM_T4COR4    EQU (1<<T4COR4)       

T4COR5       EQU 5       ; Timer4 compare bit 5
BM_T4COR5    EQU (1<<T4COR5)       

T4COR6       EQU 6       ; Timer4 compare bit 6
BM_T4COR6    EQU (1<<T4COR6)       

T4COR7       EQU 7       ; Timer4 compare bit 7
BM_T4COR7    EQU (1<<T4COR7)       

; T4CR - Timer4 control Register
T4OTM       EQU 0       ; Timer4 Overflow Toggle Mask
BM_T4OTM    EQU (1<<T4OTM)       

T4CTM       EQU 1       ; Timer4 Compare Toggle Mask
BM_T4CTM    EQU (1<<T4CTM)       

T4CRM       EQU 2       ; Timer4 Compare Reset Mask
BM_T4CRM    EQU (1<<T4CRM)       

T4CPRM       EQU 3       ; Timer4 CaPture Reset Mask
BM_T4CPRM    EQU (1<<T4CPRM)       

T4TOP       EQU 4       ; Timer4 Toggle Output Preset
BM_T4TOP    EQU (1<<T4TOP)       

T4RES       EQU 5       ; Timer4 Reset
BM_T4RES    EQU (1<<T4RES)       

T4TOS       EQU 6       ; Timer4 Toggle with Start
BM_T4TOS    EQU (1<<T4TOS)       

T4ENA       EQU 7       ; Timer4 Enable
BM_T4ENA    EQU (1<<T4ENA)       

; T4ICRH - Timer4 input capture High Byte Register
T4ICR8       EQU 0       ; Timer4 input capture bit 8
BM_T4ICR8    EQU (1<<T4ICR8)       

T4ICR9       EQU 1       ; Timer4 input capture bit 9
BM_T4ICR9    EQU (1<<T4ICR9)       

T4ICR10       EQU 2       ; Timer4 input capture bit 10
BM_T4ICR10    EQU (1<<T4ICR10)       

T4ICR11       EQU 3       ; Timer4 input capture bit 11
BM_T4ICR11    EQU (1<<T4ICR11)       

T4ICR12       EQU 4       ; Timer4 input capture bit 12
BM_T4ICR12    EQU (1<<T4ICR12)       

T4ICR13       EQU 5       ; Timer4 input capture bit 13
BM_T4ICR13    EQU (1<<T4ICR13)       

T4ICR14       EQU 6       ; Timer4 input capture bit 14
BM_T4ICR14    EQU (1<<T4ICR14)       

T4ICR15       EQU 7       ; Timer4 input capture bit 15
BM_T4ICR15    EQU (1<<T4ICR15)       

; T4ICRL - Timer4 input capture Low Byte Register
T4ICR0       EQU 0       ; Timer4 input capture bit 0
BM_T4ICR0    EQU (1<<T4ICR0)       

T4ICR1       EQU 1       ; Timer4 input capture bit 1
BM_T4ICR1    EQU (1<<T4ICR1)       

T4ICR2       EQU 2       ; Timer4 input capture bit 2
BM_T4ICR2    EQU (1<<T4ICR2)       

T4ICR3       EQU 3       ; Timer4 input capture bit 3
BM_T4ICR3    EQU (1<<T4ICR3)       

T4ICR4       EQU 4       ; Timer4 input capture bit 4
BM_T4ICR4    EQU (1<<T4ICR4)       

T4ICR5       EQU 5       ; Timer4 input capture bit 5
BM_T4ICR5    EQU (1<<T4ICR5)       

T4ICR6       EQU 6       ; Timer4 input capture bit 6
BM_T4ICR6    EQU (1<<T4ICR6)       

T4ICR7       EQU 7       ; Timer4 input capture bit 7
BM_T4ICR7    EQU (1<<T4ICR7)       

; T4IFR - Timer4 interrupt flag Register
T4OFF       EQU 0       ; Timer4 OverFlow Flag
BM_T4OFF    EQU (1<<T4OFF)       

T4COF       EQU 1       ; Timer4 Compare Flag
BM_T4COF    EQU (1<<T4COF)       

T4ICF       EQU 2       ; Timer4 Input Capture Flag
BM_T4ICF    EQU (1<<T4ICF)       

; T4IMR - Timer4 interrupt mask Register
T4OIM       EQU 0       ; Timer4 Overflow Interrupt Mask
BM_T4OIM    EQU (1<<T4OIM)       

T4CIM       EQU 1       ; Timer4 Compare Interrupt Mask
BM_T4CIM    EQU (1<<T4CIM)       

T4CPIM       EQU 2       ; Timer4 Capture Interrupt Mask
BM_T4CPIM    EQU (1<<T4CPIM)       

; T4MRA - Timer4 mode Register
T4CS0       EQU 0       ; Timer4 Clock Select bit 0
BM_T4CS0    EQU (1<<T4CS0)       

T4CS1       EQU 1       ; Timer4 Clock Select bit 1
BM_T4CS1    EQU (1<<T4CS1)       

T4PS0       EQU 2       ; Timer4 Prescaler Select bit 0
BM_T4PS0    EQU (1<<T4PS0)       

T4PS1       EQU 3       ; Timer4 Prescaler Select bit 1
BM_T4PS1    EQU (1<<T4PS1)       

T4PS2       EQU 4       ; Timer4 Prescaler Select bit 2
BM_T4PS2    EQU (1<<T4PS2)       

; T4MRB - Timer4 mode Register
T4SCE       EQU 1       ; Timer4 Software Capture Enable
BM_T4SCE    EQU (1<<T4SCE)       

T4CNC       EQU 2       ; Timer4 input Capture Noise Canceller
BM_T4CNC    EQU (1<<T4CNC)       

T4CE0       EQU 3       ; Timer4 Capture Edge select bit 0
BM_T4CE0    EQU (1<<T4CE0)       

T4CE1       EQU 4       ; Timer4 Capture Edge select bit 1
BM_T4CE1    EQU (1<<T4CE1)       

T4ICS0       EQU 5       ; Timer4 Input Capture Select bit 0
BM_T4ICS0    EQU (1<<T4ICS0)       

T4ICS1       EQU 6       ; Timer4 Input Capture Select bit 1
BM_T4ICS1    EQU (1<<T4ICS1)       

T4ICS2       EQU 7       ; Timer4 Input Capture Select bit 2
BM_T4ICS2    EQU (1<<T4ICS2)       

; ***** TIMER5 ***********************
; GTCCR - General Timer/Counter Control Register
PSR10       EQU 0       ; PreScaler Reset
BM_PSR10    EQU (1<<PSR10)       

TSM       EQU 7       ; Timer/Counter Synchronization Mode
BM_TSM    EQU (1<<TSM)       

; T5CCR - Timer5 Control Register
T5CS0       EQU 0       ; Timer5 Clock Select bit 0
BM_T5CS0    EQU (1<<T5CS0)       

T5CS1       EQU 1       ; Timer5 Clock Select bit 1
BM_T5CS1    EQU (1<<T5CS1)       

T5CS2       EQU 2       ; Timer5 Clock Select bit 2
BM_T5CS2    EQU (1<<T5CS2)       

T5CTC       EQU 3       ; Clear Timer5 on Compare Match
BM_T5CTC    EQU (1<<T5CTC)       

; T5CNTH - Timer5 Counter High byte
T5CNT8       EQU 0       ; Timer5 Counter bit 8
BM_T5CNT8    EQU (1<<T5CNT8)       

T5CNT9       EQU 1       ; Timer5 Counter bit 9
BM_T5CNT9    EQU (1<<T5CNT9)       

T5CNT10       EQU 2       ; Timer5 Counter bit 10
BM_T5CNT10    EQU (1<<T5CNT10)       

T5CNT11       EQU 3       ; Timer5 Counter bit 11
BM_T5CNT11    EQU (1<<T5CNT11)       

T5CNT12       EQU 4       ; Timer5 Counter bit 12
BM_T5CNT12    EQU (1<<T5CNT12)       

T5CNT13       EQU 5       ; Timer5 Counter bit 13
BM_T5CNT13    EQU (1<<T5CNT13)       

T5CNT14       EQU 6       ; Timer5 Counter bit 14
BM_T5CNT14    EQU (1<<T5CNT14)       

T5CNT15       EQU 7       ; Timer5 Counter bit 15
BM_T5CNT15    EQU (1<<T5CNT15)       

; T5CNTL - Timer5 Counter Low byte
T5CNT0       EQU 0       ; Timer5 Counter bit 0
BM_T5CNT0    EQU (1<<T5CNT0)       

T5CNT1       EQU 1       ; Timer5 Counter bit 1
BM_T5CNT1    EQU (1<<T5CNT1)       

T5CNT2       EQU 2       ; Timer5 Counter bit 2
BM_T5CNT2    EQU (1<<T5CNT2)       

T5CNT3       EQU 3       ; Timer5 Counter bit 3
BM_T5CNT3    EQU (1<<T5CNT3)       

T5CNT4       EQU 4       ; Timer5 Counter bit 4
BM_T5CNT4    EQU (1<<T5CNT4)       

T5CNT5       EQU 5       ; Timer5 Counter bit 5
BM_T5CNT5    EQU (1<<T5CNT5)       

T5CNT6       EQU 6       ; Timer5 Counter bit 6
BM_T5CNT6    EQU (1<<T5CNT6)       

T5CNT7       EQU 7       ; Timer5 Counter bit 7
BM_T5CNT7    EQU (1<<T5CNT7)       

; T5IFR - Timer5 Interrupt Flag Register
T5OFF       EQU 0       ; Timer5 Output Overflow Flag
BM_T5OFF    EQU (1<<T5OFF)       

T5COF       EQU 1       ; Timer5 Output Compare Output Match Flag
BM_T5COF    EQU (1<<T5COF)       

; T5IMR - Timer5 Interrupt Mask Register
T5OIM       EQU 0       ; Timer5 Output Overflow Interrupt Mask
BM_T5OIM    EQU (1<<T5OIM)       

T5CIM       EQU 1       ; Timer5 Output Compare Interrupt Mask
BM_T5CIM    EQU (1<<T5CIM)       

; T5OCRH - Timer5 Output Compare Register High byte
T5OCR8       EQU 0       ; Timer5 Output Compare Register bit 8
BM_T5OCR8    EQU (1<<T5OCR8)       

T5OCR9       EQU 1       ; Timer5 Output Compare Register bit 9
BM_T5OCR9    EQU (1<<T5OCR9)       

T5OCR10       EQU 2       ; Timer5 Output Compare Register bit 10
BM_T5OCR10    EQU (1<<T5OCR10)       

T5OCR11       EQU 3       ; Timer5 Output Compare Register bit 11
BM_T5OCR11    EQU (1<<T5OCR11)       

T5OCR12       EQU 4       ; Timer5 Output Compare Register bit 12
BM_T5OCR12    EQU (1<<T5OCR12)       

T5OCR13       EQU 5       ; Timer5 Output Compare Register bit 13
BM_T5OCR13    EQU (1<<T5OCR13)       

T5OCR14       EQU 6       ; Timer5 Output Compare Register bit 14
BM_T5OCR14    EQU (1<<T5OCR14)       

T5OCR15       EQU 7       ; Timer5 Output Compare Register bit 15
BM_T5OCR15    EQU (1<<T5OCR15)       

; T5OCRL - Timer5 Output Compare Register Low byte
T5OCR0       EQU 0       ; Timer5 Output Compare Register bit 0
BM_T5OCR0    EQU (1<<T5OCR0)       

T5OCR1       EQU 1       ; Timer5 Output Compare Register bit 1
BM_T5OCR1    EQU (1<<T5OCR1)       

T5OCR2       EQU 2       ; Timer5 Output Compare Register bit 2
BM_T5OCR2    EQU (1<<T5OCR2)       

T5OCR3       EQU 3       ; Timer5 Output Compare Register bit 3
BM_T5OCR3    EQU (1<<T5OCR3)       

T5OCR4       EQU 4       ; Timer5 Output Compare Register bit 4
BM_T5OCR4    EQU (1<<T5OCR4)       

T5OCR5       EQU 5       ; Timer5 Output Compare Register bit 5
BM_T5OCR5    EQU (1<<T5OCR5)       

T5OCR6       EQU 6       ; Timer5 Output Compare Register bit 6
BM_T5OCR6    EQU (1<<T5OCR6)       

T5OCR7       EQU 7       ; Timer5 Output Compare Register bit 7
BM_T5OCR7    EQU (1<<T5OCR7)       

; T5TEMP - Timer5 Temp Register
T5TEMP0       EQU 0       ; Timer5 Temp bit 0
BM_T5TEMP0    EQU (1<<T5TEMP0)       

T5TEMP1       EQU 1       ; Timer5 Temp bit 1
BM_T5TEMP1    EQU (1<<T5TEMP1)       

T5TEMP2       EQU 2       ; Timer5 Temp bit 2
BM_T5TEMP2    EQU (1<<T5TEMP2)       

T5TEMP3       EQU 3       ; Timer5 Temp bit 3
BM_T5TEMP3    EQU (1<<T5TEMP3)       

T5TEMP4       EQU 4       ; Timer5 Temp bit 4
BM_T5TEMP4    EQU (1<<T5TEMP4)       

T5TEMP5       EQU 5       ; Timer5 Temp bit 5
BM_T5TEMP5    EQU (1<<T5TEMP5)       

T5TEMP6       EQU 6       ; Timer5 Temp bit 6
BM_T5TEMP6    EQU (1<<T5TEMP6)       

T5TEMP7       EQU 7       ; Timer5 Temp bit 7
BM_T5TEMP7    EQU (1<<T5TEMP7)       

; ***** TMO **************************
; TMOCR - Timer Modulator Output Control Register
TO1PIS0       EQU 0       ; Timer modulator Output 1 Port Interface Select bit 0
BM_TO1PIS0    EQU (1<<TO1PIS0)       

TO1PIS1       EQU 1       ; Timer modulator Output 1 Port Interface Select bit 1
BM_TO1PIS1    EQU (1<<TO1PIS1)       

TO2PIS0       EQU 2       ; Timer modulator Output 2 Port Interface Select bit 0
BM_TO2PIS0    EQU (1<<TO2PIS0)       

TO2PIS1       EQU 3       ; Timer modulator Output 2 Port Interface Select bit 1
BM_TO2PIS1    EQU (1<<TO2PIS1)       

TO3PIS0       EQU 4       ; Timer modulator Output 3 Port Interface Select bit 0
BM_TO3PIS0    EQU (1<<TO3PIS0)       

TO3PIS1       EQU 5       ; Timer modulator Output 3 Port Interface Select bit 1
BM_TO3PIS1    EQU (1<<TO3PIS1)       

TO4PIS0       EQU 6       ; Timer modulator Output 4 Port Interface Select bit 0
BM_TO4PIS0    EQU (1<<TO4PIS0)       

TO4PIS1       EQU 7       ; Timer modulator Output 4 Port Interface Select bit 1
BM_TO4PIS1    EQU (1<<TO4PIS1)       

; ***** TPLF_CAL *********************
; LFCPR - LF Receiver Calibration Protect Register
LFCALP       EQU 0       ; LF Calibration Protection
BM_LFCALP    EQU (1<<LFCALP)       

LFCALRY       EQU 1       ; LF Calibration Ready
BM_LFCALRY    EQU (1<<LFCALRY)       

TPCD       EQU 6       ; Transponder Constant Disable
BM_TPCD    EQU (1<<TPCD)       

LFCPCE       EQU 7       ; LF Calibration Protect Change Enable
BM_LFCPCE    EQU (1<<LFCPCE)       

; SRCCAL - Slow RC oscillator calibration
SRCCAL1       EQU 0       ; Slow RC oscillator calibration bit 1
BM_SRCCAL1    EQU (1<<SRCCAL1)       

SRCCAL2       EQU 1       ; Slow RC oscillator calibration bit 2
BM_SRCCAL2    EQU (1<<SRCCAL2)       

SRCCAL3       EQU 2       ; Slow RC oscillator calibration bit 3
BM_SRCCAL3    EQU (1<<SRCCAL3)       

SRCCAL4       EQU 3       ; Slow RC oscillator calibration bit 4
BM_SRCCAL4    EQU (1<<SRCCAL4)       

SRCCAL5       EQU 4       ; Slow RC oscillator calibration bit 5
BM_SRCCAL5    EQU (1<<SRCCAL5)       

SRCCAL6       EQU 5       ; Slow RC oscillator calibration bit 6
BM_SRCCAL6    EQU (1<<SRCCAL6)       

SRCCAL7       EQU 6       ; Slow RC oscillator calibration bit 7
BM_SRCCAL7    EQU (1<<SRCCAL7)       

SRCCAL8       EQU 7       ; Slow RC oscillator calibration bit 8
BM_SRCCAL8    EQU (1<<SRCCAL8)       

; SRCCALL - Slow RC oscillator calibration LSB
SRCCAL0       EQU 0       ; Slow RC oscillator calibration bit 0
BM_SRCCAL0    EQU (1<<SRCCAL0)       

; SRCTCAL - SRC oscillator Temperature Compensation register
SRCTC0       EQU 0       ; SRC Oscillator Temperature Compensation bit 0
BM_SRCTC0    EQU (1<<SRCTC0)       

SRCTC1       EQU 1       ; SRC Oscillator Temperature Compensation bit 1
BM_SRCTC1    EQU (1<<SRCTC1)       

SRCTC2       EQU 2       ; SRC Oscillator Temperature Compensation bit 2
BM_SRCTC2    EQU (1<<SRCTC2)       

SRCS0       EQU 3       ; SRC Oscillator Select delta current bit 0
BM_SRCS0    EQU (1<<SRCS0)       

SRCS1       EQU 4       ; SRC Oscillator Select delta current bit 1
BM_SRCS1    EQU (1<<SRCS1)       

DIS_SRC       EQU 6       ; Disable SRC Bandgap
BM_DIS_SRC    EQU (1<<DIS_SRC)       

HOLD_SRC       EQU 7       ; Stop SRC-Osccillator output
BM_HOLD_SRC    EQU (1<<HOLD_SRC)       

; ***** TWI1 *************************
; TW1AMR - TWI1 Address Mask Register
TW1AM0       EQU 1       ; TWI1 Address Mask bit 0
BM_TW1AM0    EQU (1<<TW1AM0)       

TW1AM1       EQU 2       ; TWI1 Address Mask bit 1
BM_TW1AM1    EQU (1<<TW1AM1)       

TW1AM2       EQU 3       ; TWI1 Address Mask bit 2
BM_TW1AM2    EQU (1<<TW1AM2)       

TW1AM3       EQU 4       ; TWI1 Address Mask bit 3
BM_TW1AM3    EQU (1<<TW1AM3)       

TW1AM4       EQU 5       ; TWI1 Address Mask bit 4
BM_TW1AM4    EQU (1<<TW1AM4)       

TW1AM5       EQU 6       ; TWI1 Address Mask bit 5
BM_TW1AM5    EQU (1<<TW1AM5)       

TW1AM6       EQU 7       ; TWI1 Address Mask bit 6
BM_TW1AM6    EQU (1<<TW1AM6)       

; TW1AR - TWI1 (Slave) Address Register
TW1GCE       EQU 0       ; TWI1 General Call Recognition Enable
BM_TW1GCE    EQU (1<<TW1GCE)       

TW1A0       EQU 1       ; TWI1 (Slave) Address bit 0
BM_TW1A0    EQU (1<<TW1A0)       

TW1A1       EQU 2       ; TWI1 (Slave) Address bit 1
BM_TW1A1    EQU (1<<TW1A1)       

TW1A2       EQU 3       ; TWI1 (Slave) Address bit 2
BM_TW1A2    EQU (1<<TW1A2)       

TW1A3       EQU 4       ; TWI1 (Slave) Address bit 3
BM_TW1A3    EQU (1<<TW1A3)       

TW1A4       EQU 5       ; TWI1 (Slave) Address bit 4
BM_TW1A4    EQU (1<<TW1A4)       

TW1A5       EQU 6       ; TWI1 (Slave) Address bit 5
BM_TW1A5    EQU (1<<TW1A5)       

TW1A6       EQU 7       ; TWI1 (Slave) Address bit 6
BM_TW1A6    EQU (1<<TW1A6)       

; TW1BR - TWI1 Bit Rate Register
TW1BR0       EQU 0       ; TWI1 Bit Rate bit 0
BM_TW1BR0    EQU (1<<TW1BR0)       

TW1BR1       EQU 1       ; TWI1 Bit Rate bit 1
BM_TW1BR1    EQU (1<<TW1BR1)       

TW1BR2       EQU 2       ; TWI1 Bit Rate bit 2
BM_TW1BR2    EQU (1<<TW1BR2)       

TW1BR3       EQU 3       ; TWI1 Bit Rate bit 3
BM_TW1BR3    EQU (1<<TW1BR3)       

TW1BR4       EQU 4       ; TWI1 Bit Rate bit 4
BM_TW1BR4    EQU (1<<TW1BR4)       

TW1BR5       EQU 5       ; TWI1 Bit Rate bit 5
BM_TW1BR5    EQU (1<<TW1BR5)       

TW1BR6       EQU 6       ; TWI1 Bit Rate bit 6
BM_TW1BR6    EQU (1<<TW1BR6)       

TW1BR7       EQU 7       ; TWI1 Bit Rate bit 7
BM_TW1BR7    EQU (1<<TW1BR7)       

; TW1CR - TWI1 Control Register
TW1IE       EQU 0       ; TWI1 Interrupt Enable
BM_TW1IE    EQU (1<<TW1IE)       

TW1EN       EQU 2       ; TWI1 Enable
BM_TW1EN    EQU (1<<TW1EN)       

TW1WC       EQU 3       ; TWI1 Write Collision Flag
BM_TW1WC    EQU (1<<TW1WC)       

TW1STO       EQU 4       ; TWI1 STOP Condition
BM_TW1STO    EQU (1<<TW1STO)       

TW1STA       EQU 5       ; TWI1 START Condition
BM_TW1STA    EQU (1<<TW1STA)       

TW1EA       EQU 6       ; TWI1 Enable Acknowledge
BM_TW1EA    EQU (1<<TW1EA)       

TW1INT       EQU 7       ; TWI1 Interrupt Flag
BM_TW1INT    EQU (1<<TW1INT)       

; TW1DR - TWI1 Data Register
TW1D0       EQU 0       ; TWI1 Data bit 0
BM_TW1D0    EQU (1<<TW1D0)       

TW1D1       EQU 1       ; TWI1 Data bit 1
BM_TW1D1    EQU (1<<TW1D1)       

TW1D2       EQU 2       ; TWI1 Data bit 2
BM_TW1D2    EQU (1<<TW1D2)       

TW1D3       EQU 3       ; TWI1 Data bit 3
BM_TW1D3    EQU (1<<TW1D3)       

TW1D4       EQU 4       ; TWI1 Data bit 4
BM_TW1D4    EQU (1<<TW1D4)       

TW1D5       EQU 5       ; TWI1 Data bit 5
BM_TW1D5    EQU (1<<TW1D5)       

TW1D6       EQU 6       ; TWI1 Data bit 6
BM_TW1D6    EQU (1<<TW1D6)       

TW1D7       EQU 7       ; TWI1 Data bit 7
BM_TW1D7    EQU (1<<TW1D7)       

; TW1SR - TWI1 Status Register
TW1PS0       EQU 0       ; TWI1 Prescaler bit 0
BM_TW1PS0    EQU (1<<TW1PS0)       

TW1PS1       EQU 1       ; TWI1 Prescaler bit 1
BM_TW1PS1    EQU (1<<TW1PS1)       

TW1S0       EQU 3       ; TWI1 Status bit 0
BM_TW1S0    EQU (1<<TW1S0)       

TW1S1       EQU 4       ; TWI1 Status bit 1
BM_TW1S1    EQU (1<<TW1S1)       

TW1S2       EQU 5       ; TWI1 Status bit 2
BM_TW1S2    EQU (1<<TW1S2)       

TW1S3       EQU 6       ; TWI1 Status bit 3
BM_TW1S3    EQU (1<<TW1S3)       

TW1S4       EQU 7       ; TWI1 Status bit 4
BM_TW1S4    EQU (1<<TW1S4)       

; ***** TWI2 *************************
; TW2AMR - TWI2 Address Mask Register
TW2AM0       EQU 1       ; TWI2 Address Mask bit 0
BM_TW2AM0    EQU (1<<TW2AM0)       

TW2AM1       EQU 2       ; TWI2 Address Mask bit 1
BM_TW2AM1    EQU (1<<TW2AM1)       

TW2AM2       EQU 3       ; TWI2 Address Mask bit 2
BM_TW2AM2    EQU (1<<TW2AM2)       

TW2AM3       EQU 4       ; TWI2 Address Mask bit 3
BM_TW2AM3    EQU (1<<TW2AM3)       

TW2AM4       EQU 5       ; TWI2 Address Mask bit 4
BM_TW2AM4    EQU (1<<TW2AM4)       

TW2AM5       EQU 6       ; TWI2 Address Mask bit 5
BM_TW2AM5    EQU (1<<TW2AM5)       

TW2AM6       EQU 7       ; TWI2 Address Mask bit 6
BM_TW2AM6    EQU (1<<TW2AM6)       

; TW2AR - TWI2 (Slave) Address Register
TW2GCE       EQU 0       ; TWI2 General Call Recognition Enable
BM_TW2GCE    EQU (1<<TW2GCE)       

TW2A0       EQU 1       ; TWI2 (Slave) Address bit 0
BM_TW2A0    EQU (1<<TW2A0)       

TW2A1       EQU 2       ; TWI2 (Slave) Address bit 1
BM_TW2A1    EQU (1<<TW2A1)       

TW2A2       EQU 3       ; TWI2 (Slave) Address bit 2
BM_TW2A2    EQU (1<<TW2A2)       

TW2A3       EQU 4       ; TWI2 (Slave) Address bit 3
BM_TW2A3    EQU (1<<TW2A3)       

TW2A4       EQU 5       ; TWI2 (Slave) Address bit 4
BM_TW2A4    EQU (1<<TW2A4)       

TW2A5       EQU 6       ; TWI2 (Slave) Address bit 5
BM_TW2A5    EQU (1<<TW2A5)       

TW2A6       EQU 7       ; TWI2 (Slave) Address bit 6
BM_TW2A6    EQU (1<<TW2A6)       

; TW2BR - TWI2 Bit Rate Register
TW2BR0       EQU 0       ; TWI2 Bit Rate bit 0
BM_TW2BR0    EQU (1<<TW2BR0)       

TW2BR1       EQU 1       ; TWI2 Bit Rate bit 1
BM_TW2BR1    EQU (1<<TW2BR1)       

TW2BR2       EQU 2       ; TWI2 Bit Rate bit 2
BM_TW2BR2    EQU (1<<TW2BR2)       

TW2BR3       EQU 3       ; TWI2 Bit Rate bit 3
BM_TW2BR3    EQU (1<<TW2BR3)       

TW2BR4       EQU 4       ; TWI2 Bit Rate bit 4
BM_TW2BR4    EQU (1<<TW2BR4)       

TW2BR5       EQU 5       ; TWI2 Bit Rate bit 5
BM_TW2BR5    EQU (1<<TW2BR5)       

TW2BR6       EQU 6       ; TWI2 Bit Rate bit 6
BM_TW2BR6    EQU (1<<TW2BR6)       

TW2BR7       EQU 7       ; TWI2 Bit Rate bit 7
BM_TW2BR7    EQU (1<<TW2BR7)       

; TW2CR - TWI2 Control Register
TW2IE       EQU 0       ; TWI2 Interrupt Enable
BM_TW2IE    EQU (1<<TW2IE)       

TW2EN       EQU 2       ; TWI2 Enable
BM_TW2EN    EQU (1<<TW2EN)       

TW2WC       EQU 3       ; TWI2 Write Collision Flag
BM_TW2WC    EQU (1<<TW2WC)       

TW2STO       EQU 4       ; TWI2 STOP Condition
BM_TW2STO    EQU (1<<TW2STO)       

TW2STA       EQU 5       ; TWI2 START Condition
BM_TW2STA    EQU (1<<TW2STA)       

TW2EA       EQU 6       ; TWI2 Enable Acknowledge
BM_TW2EA    EQU (1<<TW2EA)       

TW2INT       EQU 7       ; TWI2 Interrupt Flag
BM_TW2INT    EQU (1<<TW2INT)       

; TW2DR - TWI2 Data Register
TW2D0       EQU 0       ; TWI2 Data bit 0
BM_TW2D0    EQU (1<<TW2D0)       

TW2D1       EQU 1       ; TWI2 Data bit 1
BM_TW2D1    EQU (1<<TW2D1)       

TW2D2       EQU 2       ; TWI2 Data bit 2
BM_TW2D2    EQU (1<<TW2D2)       

TW2D3       EQU 3       ; TWI2 Data bit 3
BM_TW2D3    EQU (1<<TW2D3)       

TW2D4       EQU 4       ; TWI2 Data bit 4
BM_TW2D4    EQU (1<<TW2D4)       

TW2D5       EQU 5       ; TWI2 Data bit 5
BM_TW2D5    EQU (1<<TW2D5)       

TW2D6       EQU 6       ; TWI2 Data bit 6
BM_TW2D6    EQU (1<<TW2D6)       

TW2D7       EQU 7       ; TWI2 Data bit 7
BM_TW2D7    EQU (1<<TW2D7)       

; TW2SR - TWI2 Status Register
TW2PS0       EQU 0       ; TWI2 Prescaler bit 0
BM_TW2PS0    EQU (1<<TW2PS0)       

TW2PS1       EQU 1       ; TWI2 Prescaler bit 1
BM_TW2PS1    EQU (1<<TW2PS1)       

TW2S0       EQU 3       ; TWI2 Status bit 0
BM_TW2S0    EQU (1<<TW2S0)       

TW2S1       EQU 4       ; TWI2 Status bit 1
BM_TW2S1    EQU (1<<TW2S1)       

TW2S2       EQU 5       ; TWI2 Status bit 2
BM_TW2S2    EQU (1<<TW2S2)       

TW2S3       EQU 6       ; TWI2 Status bit 3
BM_TW2S3    EQU (1<<TW2S3)       

TW2S4       EQU 7       ; TWI2 Status bit 4
BM_TW2S4    EQU (1<<TW2S4)       

; ***** TXDSP ************************
; BBTE2 - Base Band Test Enable 2
TDEPO       EQU 0       ; Tx DSP External Pin Output
BM_TDEPO    EQU (1<<TDEPO)       

DITDIS       EQU 1       ; Dither Disable on Sigma Delta Modulator
BM_DITDIS    EQU (1<<DITDIS)       

; FFREQ1H - Fractional Frequency 1 Setting, High Byte
FFREQ1H0       EQU 0       ; Fractional Frequency 1 High Byte bit 0
BM_FFREQ1H0    EQU (1<<FFREQ1H0)       

FFREQ1H1       EQU 1       ; Fractional Frequency 1 High Byte bit 1
BM_FFREQ1H1    EQU (1<<FFREQ1H1)       

; FFREQ1L - Fractional Frequency 1 Setting, Low Byte
FFREQ1L0       EQU 0       ; Fractional Frequency 1 Low Byte bit 0
BM_FFREQ1L0    EQU (1<<FFREQ1L0)       

FFREQ1L1       EQU 1       ; Fractional Frequency 1 Low Byte bit 1
BM_FFREQ1L1    EQU (1<<FFREQ1L1)       

FFREQ1L2       EQU 2       ; Fractional Frequency 1 Low Byte bit 2
BM_FFREQ1L2    EQU (1<<FFREQ1L2)       

FFREQ1L3       EQU 3       ; Fractional Frequency 1 Low Byte bit 3
BM_FFREQ1L3    EQU (1<<FFREQ1L3)       

FFREQ1L4       EQU 4       ; Fractional Frequency 1 Low Byte bit 4
BM_FFREQ1L4    EQU (1<<FFREQ1L4)       

FFREQ1L5       EQU 5       ; Fractional Frequency 1 Low Byte bit 5
BM_FFREQ1L5    EQU (1<<FFREQ1L5)       

FFREQ1L6       EQU 6       ; Fractional Frequency 1 Low Byte bit 6
BM_FFREQ1L6    EQU (1<<FFREQ1L6)       

FFREQ1L7       EQU 7       ; Fractional Frequency 1 Low Byte bit 7
BM_FFREQ1L7    EQU (1<<FFREQ1L7)       

; FFREQ1M - Fractional Frequency 1 Setting, Middle Byte
FFREQ1M0       EQU 0       ; Fractional Frequency 1 Middle Byte bit 0
BM_FFREQ1M0    EQU (1<<FFREQ1M0)       

FFREQ1M1       EQU 1       ; Fractional Frequency 1 Middle Byte bit 1
BM_FFREQ1M1    EQU (1<<FFREQ1M1)       

FFREQ1M2       EQU 2       ; Fractional Frequency 1 Middle Byte bit 2
BM_FFREQ1M2    EQU (1<<FFREQ1M2)       

FFREQ1M3       EQU 3       ; Fractional Frequency 1 Middle Byte bit 3
BM_FFREQ1M3    EQU (1<<FFREQ1M3)       

FFREQ1M4       EQU 4       ; Fractional Frequency 1 Middle Byte bit 4
BM_FFREQ1M4    EQU (1<<FFREQ1M4)       

FFREQ1M5       EQU 5       ; Fractional Frequency 1 Middle Byte bit 5
BM_FFREQ1M5    EQU (1<<FFREQ1M5)       

FFREQ1M6       EQU 6       ; Fractional Frequency 1 Middle Byte bit 6
BM_FFREQ1M6    EQU (1<<FFREQ1M6)       

FFREQ1M7       EQU 7       ; Fractional Frequency 1 Middle Byte bit 7
BM_FFREQ1M7    EQU (1<<FFREQ1M7)       

; FFREQ2H - Fractional Frequency 2 Setting, High Byte
FFREQ2H0       EQU 0       ; Fractional Frequency 2 High Byte bit 0
BM_FFREQ2H0    EQU (1<<FFREQ2H0)       

FFREQ2H1       EQU 1       ; Fractional Frequency 2 High Byte bit 1
BM_FFREQ2H1    EQU (1<<FFREQ2H1)       

; FFREQ2L - Fractional Frequency 2 Setting, Low Byte
FFREQ2L0       EQU 0       ; Fractional Frequency 2 Low Byte bit 0
BM_FFREQ2L0    EQU (1<<FFREQ2L0)       

FFREQ2L1       EQU 1       ; Fractional Frequency 2 Low Byte bit 1
BM_FFREQ2L1    EQU (1<<FFREQ2L1)       

FFREQ2L2       EQU 2       ; Fractional Frequency 2 Low Byte bit 2
BM_FFREQ2L2    EQU (1<<FFREQ2L2)       

FFREQ2L3       EQU 3       ; Fractional Frequency 2 Low Byte bit 3
BM_FFREQ2L3    EQU (1<<FFREQ2L3)       

FFREQ2L4       EQU 4       ; Fractional Frequency 2 Low Byte bit 4
BM_FFREQ2L4    EQU (1<<FFREQ2L4)       

FFREQ2L5       EQU 5       ; Fractional Frequency 2 Low Byte bit 5
BM_FFREQ2L5    EQU (1<<FFREQ2L5)       

FFREQ2L6       EQU 6       ; Fractional Frequency 2 Low Byte bit 6
BM_FFREQ2L6    EQU (1<<FFREQ2L6)       

FFREQ2L7       EQU 7       ; Fractional Frequency 2 Low Byte bit 7
BM_FFREQ2L7    EQU (1<<FFREQ2L7)       

; FFREQ2M - Fractional Frequency 2 Setting, Middle Byte
FFREQ2M0       EQU 0       ; Fractional Frequency 2 Middle Byte bit 0
BM_FFREQ2M0    EQU (1<<FFREQ2M0)       

FFREQ2M1       EQU 1       ; Fractional Frequency 2 Middle Byte bit 1
BM_FFREQ2M1    EQU (1<<FFREQ2M1)       

FFREQ2M2       EQU 2       ; Fractional Frequency 2 Middle Byte bit 2
BM_FFREQ2M2    EQU (1<<FFREQ2M2)       

FFREQ2M3       EQU 3       ; Fractional Frequency 2 Middle Byte bit 3
BM_FFREQ2M3    EQU (1<<FFREQ2M3)       

FFREQ2M4       EQU 4       ; Fractional Frequency 2 Middle Byte bit 4
BM_FFREQ2M4    EQU (1<<FFREQ2M4)       

FFREQ2M5       EQU 5       ; Fractional Frequency 2 Middle Byte bit 5
BM_FFREQ2M5    EQU (1<<FFREQ2M5)       

FFREQ2M6       EQU 6       ; Fractional Frequency 2 Middle Byte bit 6
BM_FFREQ2M6    EQU (1<<FFREQ2M6)       

FFREQ2M7       EQU 7       ; Fractional Frequency 2 Middle Byte bit 7
BM_FFREQ2M7    EQU (1<<FFREQ2M7)       

; FSCR - Frequency Synthesizer Control Register
TXMOD       EQU 0       ; Tx Modulation
BM_TXMOD    EQU (1<<TXMOD)       

SFM       EQU 1       ; Select FSK Modulation
BM_SFM    EQU (1<<SFM)       

TXMS0       EQU 2       ; Tx Modulation Source bit 0
BM_TXMS0    EQU (1<<TXMS0)       

TXMS1       EQU 3       ; Tx Modulation Source bit 1
BM_TXMS1    EQU (1<<TXMS1)       

PAOER       EQU 4       ; Power Amplifier Output Enable Register
BM_PAOER    EQU (1<<PAOER)       

PAON       EQU 7       ; Power Amplifier Output is On
BM_PAON    EQU (1<<PAON)       

; FSEN - Frequency Synthesizer Enable register
SDPU       EQU 0       ; Sigma-delta Modulator Power Up
BM_SDPU    EQU (1<<SDPU)       

SDEN       EQU 1       ; Sigma-delta Modulator Enable
BM_SDEN    EQU (1<<SDEN)       

GAEN       EQU 2       ; Gauss Filtering Enable
BM_GAEN    EQU (1<<GAEN)       

PEEN       EQU 3       ; Pre-emphasis Filtering Enable
BM_PEEN    EQU (1<<PEEN)       

ASEN       EQU 4       ; ASK Shaping Enable
BM_ASEN    EQU (1<<ASEN)       

ANTT       EQU 5       ; Antenna Tuning Active
BM_ANTT    EQU (1<<ANTT)       

; FSFCR - Frequency Synthesizer Filter Control Register
BTSEL0       EQU 0       ; Gauss Filter BT Selection bit 0
BM_BTSEL0    EQU (1<<BTSEL0)       

BTSEL1       EQU 1       ; Gauss Filter BT Selection bit 1
BM_BTSEL1    EQU (1<<BTSEL1)       

ASDIV0       EQU 4       ; ASK Shaping Divider bit 0
BM_ASDIV0    EQU (1<<ASDIV0)       

ASDIV1       EQU 5       ; ASK Shaping Divider bit 1
BM_ASDIV1    EQU (1<<ASDIV1)       

ASDIV2       EQU 6       ; ASK Shaping Divider bit 2
BM_ASDIV2    EQU (1<<ASDIV2)       

ASDIV3       EQU 7       ; ASK Shaping Divider bit 3
BM_ASDIV3    EQU (1<<ASDIV3)       

; GACDIVH - Gauss Clock Divider, High Byte
GACDIV8       EQU 0       ; Gauss Clock Divider bit 8
BM_GACDIV8    EQU (1<<GACDIV8)       

GACDIV9       EQU 1       ; Gauss Clock Divider bit 9
BM_GACDIV9    EQU (1<<GACDIV9)       

GACDIV10       EQU 2       ; Gauss Clock Divider bit 10
BM_GACDIV10    EQU (1<<GACDIV10)       

GACDIV11       EQU 3       ; Gauss Clock Divider bit 11
BM_GACDIV11    EQU (1<<GACDIV11)       

GACDIV12       EQU 4       ; Gauss Clock Divider bit 12
BM_GACDIV12    EQU (1<<GACDIV12)       

; GACDIVL - Gauss Clock Divider, Low Byte
GACDIV0       EQU 0       ; Gauss Clock Divider bit 0
BM_GACDIV0    EQU (1<<GACDIV0)       

GACDIV1       EQU 1       ; Gauss Clock Divider bit 1
BM_GACDIV1    EQU (1<<GACDIV1)       

GACDIV2       EQU 2       ; Gauss Clock Divider bit 2
BM_GACDIV2    EQU (1<<GACDIV2)       

GACDIV3       EQU 3       ; Gauss Clock Divider bit 3
BM_GACDIV3    EQU (1<<GACDIV3)       

GACDIV4       EQU 4       ; Gauss Clock Divider bit 4
BM_GACDIV4    EQU (1<<GACDIV4)       

GACDIV5       EQU 5       ; Gauss Clock Divider bit 5
BM_GACDIV5    EQU (1<<GACDIV5)       

GACDIV6       EQU 6       ; Gauss Clock Divider bit 6
BM_GACDIV6    EQU (1<<GACDIV6)       

GACDIV7       EQU 7       ; Gauss Clock Divider bit 7
BM_GACDIV7    EQU (1<<GACDIV7)       

; ***** TXM **************************
; TMCIH - Tx Modulator CRC Init Value, High Byte
TMCI8       EQU 0       ; Tx Modulator CRC Init Value bit 8
BM_TMCI8    EQU (1<<TMCI8)       

TMCI9       EQU 1       ; Tx Modulator CRC Init Value bit 9
BM_TMCI9    EQU (1<<TMCI9)       

TMCI10       EQU 2       ; Tx Modulator CRC Init Value bit 10
BM_TMCI10    EQU (1<<TMCI10)       

TMCI11       EQU 3       ; Tx Modulator CRC Init Value bit 11
BM_TMCI11    EQU (1<<TMCI11)       

TMCI12       EQU 4       ; Tx Modulator CRC Init Value bit 12
BM_TMCI12    EQU (1<<TMCI12)       

TMCI13       EQU 5       ; Tx Modulator CRC Init Value bit 13
BM_TMCI13    EQU (1<<TMCI13)       

TMCI14       EQU 6       ; Tx Modulator CRC Init Value bit 14
BM_TMCI14    EQU (1<<TMCI14)       

TMCI15       EQU 7       ; Tx Modulator CRC Init Value bit 15
BM_TMCI15    EQU (1<<TMCI15)       

; TMCIL - Tx Modulator CRC Init Value Low Byte
TMCI0       EQU 0       ; Tx Modulator CRC Init Value bit 0
BM_TMCI0    EQU (1<<TMCI0)       

TMCI1       EQU 1       ; Tx Modulator CRC Init Value bit 1
BM_TMCI1    EQU (1<<TMCI1)       

TMCI2       EQU 2       ; Tx Modulator CRC Init Value bit 2
BM_TMCI2    EQU (1<<TMCI2)       

TMCI3       EQU 3       ; Tx Modulator CRC Init Value bit 3
BM_TMCI3    EQU (1<<TMCI3)       

TMCI4       EQU 4       ; Tx Modulator CRC Init Value bit 4
BM_TMCI4    EQU (1<<TMCI4)       

TMCI5       EQU 5       ; Tx Modulator CRC Init Value bit 5
BM_TMCI5    EQU (1<<TMCI5)       

TMCI6       EQU 6       ; Tx Modulator CRC Init Value bit 6
BM_TMCI6    EQU (1<<TMCI6)       

TMCI7       EQU 7       ; Tx Modulator CRC Init Value bit 7
BM_TMCI7    EQU (1<<TMCI7)       

; TMCPH - Tx Modulator CRC Polynomial, High Byte
TMCP8       EQU 0       ; Tx Modulator CRC Polynomial bit 8
BM_TMCP8    EQU (1<<TMCP8)       

TMCP9       EQU 1       ; Tx Modulator CRC Polynomial bit 9
BM_TMCP9    EQU (1<<TMCP9)       

TMCP10       EQU 2       ; Tx Modulator CRC Polynomial bit 10
BM_TMCP10    EQU (1<<TMCP10)       

TMCP11       EQU 3       ; Tx Modulator CRC Polynomial bit 11
BM_TMCP11    EQU (1<<TMCP11)       

TMCP12       EQU 4       ; Tx Modulator CRC Polynomial bit 12
BM_TMCP12    EQU (1<<TMCP12)       

TMCP13       EQU 5       ; Tx Modulator CRC Polynomial bit 13
BM_TMCP13    EQU (1<<TMCP13)       

TMCP14       EQU 6       ; Tx Modulator CRC Polynomial bit 14
BM_TMCP14    EQU (1<<TMCP14)       

TMCP15       EQU 7       ; Tx Modulator CRC Polynomial bit 15
BM_TMCP15    EQU (1<<TMCP15)       

; TMCPL - Tx Modulator CRC Polynomial, Low Byte
TMCP0       EQU 0       ; Tx Modulator CRC Polynomial bit 0
BM_TMCP0    EQU (1<<TMCP0)       

TMCP1       EQU 1       ; Tx Modulator CRC Polynomial bit 1
BM_TMCP1    EQU (1<<TMCP1)       

TMCP2       EQU 2       ; Tx Modulator CRC Polynomial bit 2
BM_TMCP2    EQU (1<<TMCP2)       

TMCP3       EQU 3       ; Tx Modulator CRC Polynomial bit 3
BM_TMCP3    EQU (1<<TMCP3)       

TMCP4       EQU 4       ; Tx Modulator CRC Polynomial bit 4
BM_TMCP4    EQU (1<<TMCP4)       

TMCP5       EQU 5       ; Tx Modulator CRC Polynomial bit 5
BM_TMCP5    EQU (1<<TMCP5)       

TMCP6       EQU 6       ; Tx Modulator CRC Polynomial bit 6
BM_TMCP6    EQU (1<<TMCP6)       

TMCP7       EQU 7       ; Tx Modulator CRC Polynomial bit 7
BM_TMCP7    EQU (1<<TMCP7)       

; TMCR1 - Tx Modulator Control Register 1
TMPIS0       EQU 0       ; Tx Modulator Port Interface Select bit 0
BM_TMPIS0    EQU (1<<TMPIS0)       

TMPIS1       EQU 1       ; Tx Modulator Port Interface Select bit 1
BM_TMPIS1    EQU (1<<TMPIS1)       

TMPIS2       EQU 2       ; Tx Modulator Port Interface Select bit 2
BM_TMPIS2    EQU (1<<TMPIS2)       

TMSCS       EQU 3       ; Tx Modulator Serial Output Clock Select
BM_TMSCS    EQU (1<<TMSCS)       

TMCIM       EQU 4       ; Tx Modulator Transmission Complete Interrupt Mask
BM_TMCIM    EQU (1<<TMCIM)       

; TMCR2 - Tx Modulator Control Register 2
TMCRCE       EQU 0       ; Tx Modulator CRC Enable
BM_TMCRCE    EQU (1<<TMCRCE)       

TMCRCSE0       EQU 1       ; Tx Modulator CRC Select bit 0
BM_TMCRCSE0    EQU (1<<TMCRCSE0)       

TMCRCSE1       EQU 2       ; Tx Modulator CRC Select bit 1
BM_TMCRCSE1    EQU (1<<TMCRCSE1)       

TMNRZE       EQU 3       ; Tx Modulator NRZ Mode Enable
BM_TMNRZE    EQU (1<<TMNRZE)       

TMPOL       EQU 4       ; Tx Modulator Polarity
BM_TMPOL    EQU (1<<TMPOL)       

TMSSE       EQU 5       ; Tx Modulator Stop Sequence Enable
BM_TMSSE    EQU (1<<TMSSE)       

TMLSB       EQU 6       ; Tx Modulator Least Significant Bit First
BM_TMLSB    EQU (1<<TMLSB)       

; TMCRCH - Tx Modulator CRC Result, High Byte
TMCRC8       EQU 0       ; Tx Modulator CRC Result bit 8
BM_TMCRC8    EQU (1<<TMCRC8)       

TMCRC9       EQU 1       ; Tx Modulator CRC Result bit 9
BM_TMCRC9    EQU (1<<TMCRC9)       

TMCRC10       EQU 2       ; Tx Modulator CRC Result bit 10
BM_TMCRC10    EQU (1<<TMCRC10)       

TMCRC11       EQU 3       ; Tx Modulator CRC Result bit 11
BM_TMCRC11    EQU (1<<TMCRC11)       

TMCRC12       EQU 4       ; Tx Modulator CRC Result bit 12
BM_TMCRC12    EQU (1<<TMCRC12)       

TMCRC13       EQU 5       ; Tx Modulator CRC Result bit 13
BM_TMCRC13    EQU (1<<TMCRC13)       

TMCRC14       EQU 6       ; Tx Modulator CRC Result bit 14
BM_TMCRC14    EQU (1<<TMCRC14)       

TMCRC15       EQU 7       ; Tx Modulator CRC Result bit 15
BM_TMCRC15    EQU (1<<TMCRC15)       

; TMCRCL - Tx Modulator CRC Result, Low Byte
TMCRC0       EQU 0       ; Tx Modulator CRC Result bit 0
BM_TMCRC0    EQU (1<<TMCRC0)       

TMCRC1       EQU 1       ; Tx Modulator CRC Result bit 1
BM_TMCRC1    EQU (1<<TMCRC1)       

TMCRC2       EQU 2       ; Tx Modulator CRC Result bit 2
BM_TMCRC2    EQU (1<<TMCRC2)       

TMCRC3       EQU 3       ; Tx Modulator CRC Result bit 3
BM_TMCRC3    EQU (1<<TMCRC3)       

TMCRC4       EQU 4       ; Tx Modulator CRC Result bit 4
BM_TMCRC4    EQU (1<<TMCRC4)       

TMCRC5       EQU 5       ; Tx Modulator CRC Result bit 5
BM_TMCRC5    EQU (1<<TMCRC5)       

TMCRC6       EQU 6       ; Tx Modulator CRC Result bit 6
BM_TMCRC6    EQU (1<<TMCRC6)       

TMCRC7       EQU 7       ; Tx Modulator CRC Result bit 7
BM_TMCRC7    EQU (1<<TMCRC7)       

; TMCSB - Tx Modulator CRC Skip Bit Number
TMCSB0       EQU 0       ; Tx Modulator CRC Skip Bit Number bit 0
BM_TMCSB0    EQU (1<<TMCSB0)       

TMCSB1       EQU 1       ; Tx Modulator CRC Skip Bit Number bit 1
BM_TMCSB1    EQU (1<<TMCSB1)       

TMCSB2       EQU 2       ; Tx Modulator CRC Skip Bit Number bit 2
BM_TMCSB2    EQU (1<<TMCSB2)       

TMCSB3       EQU 3       ; Tx Modulator CRC Skip Bit Number bit 3
BM_TMCSB3    EQU (1<<TMCSB3)       

TMCSB4       EQU 4       ; Tx Modulator CRC Skip Bit Number bit 4
BM_TMCSB4    EQU (1<<TMCSB4)       

TMCSB5       EQU 5       ; Tx Modulator CRC Skip Bit Number bit 5
BM_TMCSB5    EQU (1<<TMCSB5)       

TMCSB6       EQU 6       ; Tx Modulator CRC Skip Bit Number bit 6
BM_TMCSB6    EQU (1<<TMCSB6)       

TMCSB7       EQU 7       ; Tx Modulator CRC Skip Bit Number bit 7
BM_TMCSB7    EQU (1<<TMCSB7)       

; TMFSM - Tx Modulator Finite State Machine
TMSSM0       EQU 0       ; Tx Modulator Sub State Machine bit 0
BM_TMSSM0    EQU (1<<TMSSM0)       

TMSSM1       EQU 1       ; Tx Modulator Sub State Machine bit 1
BM_TMSSM1    EQU (1<<TMSSM1)       

TMSSM2       EQU 2       ; Tx Modulator Sub State Machine bit 2
BM_TMSSM2    EQU (1<<TMSSM2)       

TMSSM3       EQU 3       ; Tx Modulator Sub State Machine bit 3
BM_TMSSM3    EQU (1<<TMSSM3)       

TMMSM0       EQU 4       ; Tx Modulator Master State Machine bit 0
BM_TMMSM0    EQU (1<<TMMSM0)       

TMMSM1       EQU 5       ; Tx Modulator Master State Machine bit 1
BM_TMMSM1    EQU (1<<TMMSM1)       

TMMSM2       EQU 6       ; Tx Modulator Master State Machine bit 2
BM_TMMSM2    EQU (1<<TMMSM2)       

; TMSHR - Tx Modulator Shift Register
TMSHR0       EQU 0       ; Tx Modulator Shift Register bit 0
BM_TMSHR0    EQU (1<<TMSHR0)       

TMSHR1       EQU 1       ; Tx Modulator Shift Register bit 1
BM_TMSHR1    EQU (1<<TMSHR1)       

TMSHR2       EQU 2       ; Tx Modulator Shift Register bit 2
BM_TMSHR2    EQU (1<<TMSHR2)       

TMSHR3       EQU 3       ; Tx Modulator Shift Register bit 3
BM_TMSHR3    EQU (1<<TMSHR3)       

TMSHR4       EQU 4       ; Tx Modulator Shift Register bit 4
BM_TMSHR4    EQU (1<<TMSHR4)       

TMSHR5       EQU 5       ; Tx Modulator Shift Register bit 5
BM_TMSHR5    EQU (1<<TMSHR5)       

TMSHR6       EQU 6       ; Tx Modulator Shift Register bit 6
BM_TMSHR6    EQU (1<<TMSHR6)       

TMSHR7       EQU 7       ; Tx Modulator Shift Register bit 7
BM_TMSHR7    EQU (1<<TMSHR7)       

; TMSR - Tx Modulator Status Register
TMTCF       EQU 0       ; Tx Modulator Transmission Complete Status Flag
BM_TMTCF    EQU (1<<TMTCF)       

; TMSSC - Tx Modulator Stop Sequence Configuration
TMSSP0       EQU 0       ; Tx Modulator Stop Sequence Pattern bit 0
BM_TMSSP0    EQU (1<<TMSSP0)       

TMSSP1       EQU 1       ; Tx Modulator Stop Sequence Pattern bit 1
BM_TMSSP1    EQU (1<<TMSSP1)       

TMSSP2       EQU 2       ; Tx Modulator Stop Sequence Pattern bit 2
BM_TMSSP2    EQU (1<<TMSSP2)       

TMSSP3       EQU 3       ; Tx Modulator Stop Sequence Pattern bit 3
BM_TMSSP3    EQU (1<<TMSSP3)       

TMSSL0       EQU 4       ; Tx Modulator Stop Sequence Length bit 0
BM_TMSSL0    EQU (1<<TMSSL0)       

TMSSL1       EQU 5       ; Tx Modulator Stop Sequence Length bit 1
BM_TMSSL1    EQU (1<<TMSSL1)       

TMSSL2       EQU 6       ; Tx Modulator Stop Sequence Length bit 2
BM_TMSSL2    EQU (1<<TMSSL2)       

TMSSH       EQU 7       ; Tx Modulator Stop Sequence Hold Mode
BM_TMSSH    EQU (1<<TMSSH)       

; TMTLH - Tx Modulator Telegram Length Register, High Byte
TMTL8       EQU 0       ; Tx Modulator Telegram Length bit 8
BM_TMTL8    EQU (1<<TMTL8)       

TMTL9       EQU 1       ; Tx Modulator Telegram Length bit 9
BM_TMTL9    EQU (1<<TMTL9)       

TMTL10       EQU 2       ; Tx Modulator Telegram Length bit 10
BM_TMTL10    EQU (1<<TMTL10)       

TMTL11       EQU 3       ; Tx Modulator Telegram Length bit 11
BM_TMTL11    EQU (1<<TMTL11)       

; TMTLL - Tx Modulator Telegram Length Register, Low Byte
TMTL0       EQU 0       ; Tx Modulator Telegram Length bit 0
BM_TMTL0    EQU (1<<TMTL0)       

TMTL1       EQU 1       ; Tx Modulator Telegram Length bit 1
BM_TMTL1    EQU (1<<TMTL1)       

TMTL2       EQU 2       ; Tx Modulator Telegram Length bit 2
BM_TMTL2    EQU (1<<TMTL2)       

TMTL3       EQU 3       ; Tx Modulator Telegram Length bit 3
BM_TMTL3    EQU (1<<TMTL3)       

TMTL4       EQU 4       ; Tx Modulator Telegram Length bit 4
BM_TMTL4    EQU (1<<TMTL4)       

TMTL5       EQU 5       ; Tx Modulator Telegram Length bit 5
BM_TMTL5    EQU (1<<TMTL5)       

TMTL6       EQU 6       ; Tx Modulator Telegram Length bit 6
BM_TMTL6    EQU (1<<TMTL6)       

TMTL7       EQU 7       ; Tx Modulator Telegram Length bit 7
BM_TMTL7    EQU (1<<TMTL7)       

; ***** VX_MODE **********************
; VXMCTRL - VX Mode Control Register
VX_SEL0       EQU 0       ; Select VX voltage level
BM_VX_SEL0    EQU (1<<VX_SEL0)       

VX_SEL1       EQU 1       ; Select VX voltage level
BM_VX_SEL1    EQU (1<<VX_SEL1)       

EN_VX       EQU 2       ; Enable the VX_LDO
BM_EN_VX    EQU (1<<EN_VX)       

EN_VX_OUT       EQU 3       ; Enable PB4 as an VX/VBAT Supply Output
BM_EN_VX_OUT    EQU (1<<EN_VX_OUT)       

EN_VX_IN       EQU 4       ; Enable VX Inputs
BM_EN_VX_IN    EQU (1<<EN_VX_IN)       

; ***** LOCKSBITS ********************************************************
LB1       EQU 0       ; Lock bit
LB2       EQU 1       ; Lock bit
BLB01       EQU 2       ; Boot Lock bit
BLB02       EQU 3       ; Boot Lock bit
BLB11       EQU 4       ; Boot lock bit
BLB12       EQU 5       ; Boot lock bit
; ***** FUSES ************************************************************
; LOW fuse bits
; ***** CPU REGISTER DEFINITIONS *****************************************
;.def       XH       EQU r27
;.def       XL       EQU r26
;.def       YH       EQU r29
;.def       YL       EQU r28
;.def       ZH       EQU r31
;.def       ZL       EQU r30
; ***** DATA MEMORY DECLARATIONS *****************************************
FLASHEND       EQU 0x7fff       ; Note: Word address
IOEND       EQU 0x01ff
SRAM_START       EQU 0x0200
SRAM_SIZE       EQU 1024
RAMEND       EQU 0x05ff
XRAMEND       EQU 0x0000
E2END       EQU 0x097f
EEPROMEND       EQU 0x097f
EEADRBITS       EQU 11
; ***** BOOTLOADER DECLARATIONS ******************************************
NRWW_START_ADDR       EQU 0xc00
NRWW_STOP_ADDR       EQU 0xfff
RWW_START_ADDR       EQU 0x0
RWW_STOP_ADDR       EQU 0xbff
PAGESIZE       EQU 32
FIRSTBOOTSTART       EQU 0x7000
SECONDBOOTSTART       EQU 0x6000
THIRDBOOTSTART       EQU 0x4000
FOURTHBOOTSTART       EQU 0x3600
SMALLBOOTSTART       EQU FIRSTBOOTSTART
LARGEBOOTSTART       EQU FOURTHBOOTSTART
; ***** INTERRUPT VECTORS ************************************************
INT0addr       EQU 0x0002       ; External Interrupt Request 0
INT1addr       EQU 0x0004       ; External Interrupt Request 1
PCI0addr       EQU 0x0006       ; Pin Change Interrupt Request 0
PCI1addr       EQU 0x0008       ; Pin Change Interrupt Request 1
VMONaddr       EQU 0x000a       ; Voltage Monitoring Interrupt
AVCCRaddr       EQU 0x000c       ; AVCC Reset Interrupt
AVCCLaddr       EQU 0x000e       ; AVCC Low Interrupt
T0INTaddr       EQU 0x0010       ; Timer0 Interval Interrupt
T1COMPaddr       EQU 0x0012       ; Timer/Counter1 Compare Match Interrupt
T1OVFaddr       EQU 0x0014       ; Timer/Counter1 Overflow Interrupt
T2COMPaddr       EQU 0x0016       ; Timer/Counter2 Compare Match Interrupt
T2OVFaddr       EQU 0x0018       ; Timer/Counter2 Overflow Interrupt
T3CAPaddr       EQU 0x001a       ; Timer/Counter3 Capture Event Interrupt
T3COMPaddr       EQU 0x001c       ; Timer/Counter3 Compare Match Interrupt
T3OVFaddr       EQU 0x001e       ; Timer/Counter3 Overflow Interrupt
T4CAPaddr       EQU 0x0020       ; Timer/Counter4 Capture Event Interrupt
T4COMPaddr       EQU 0x0022       ; Timer/Counter4 Compare Match Interrupt
T4OVFaddr       EQU 0x0024       ; Timer/Counter4 Overflow Interrupt
T5COMPaddr       EQU 0x0026       ; Timer/Counter5 Compare Match Interrupt
T5OVFaddr       EQU 0x0028       ; Timer/Counter5 Overflow Interrupt
SPIaddr       EQU 0x002a       ; SPI Serial Transfer Complete Interrupt
SRX_FIFOaddr       EQU 0x002c       ; SPI Rx Buffer Interrupt
STX_FIFOaddr       EQU 0x002e       ; SPI Tx Buffer Interrupt
SSMaddr       EQU 0x0030       ; Sequencer State Machine Interrupt
DFFLRaddr       EQU 0x0032       ; Data FIFO fill level reached Interrupt
DFOUEaddr       EQU 0x0034       ; Data FIFO overflow or underflow error Interrupt
SFFLRaddr       EQU 0x0036       ; RSSI/Preamble FIFO fill level reached Interrupt
SFOUEaddr       EQU 0x0038       ; RSSI/Preamble FIFO overflow or underflow error Interrupt
TMTCFaddr       EQU 0x003a       ; Tx Modulator Telegram Finish Interrupt
AESaddr       EQU 0x003c       ; AES Krypto Unit Interrupt
TPINTaddr       EQU 0x003e       ; Transponder Mode Interrupt
TPTOERRaddr       EQU 0x0040       ; Transponder Timeout Error Interrupt
LFID0INTaddr       EQU 0x0042       ; LF receiver Identifier 0 Interrupt
LFID1INTaddr       EQU 0x0044       ; LF receiver Identifier 1 Interrupt
LFFEINTaddr       EQU 0x0046       ; LF receiver Frame End Interrupt
LFBCRaddr       EQU 0x0048       ; LF receiver Bit Count Reached Interrupt
LFPBDaddr       EQU 0x004a       ; LF receiver PreBurst Detected Interrupt
LFDEaddr       EQU 0x004c       ; LF receiver Decoder Error Interrupt
LFEOTaddr       EQU 0x004e       ; LF receiver End of Telegram Interrupt
LFTCORaddr       EQU 0x0050       ; LF receiver Timer Compare Match Interrupt
LFRSCOaddr       EQU 0x0052       ; LF receiver RSSI measurement Interrupt
LDFFLRaddr       EQU 0x0054       ; LF Data FIFO Fill Level Reached Interrupt
LDFOUEaddr       EQU 0x0056       ; LF Data FIFO Overflow or Underflow Error Interrupt
EXCMaddr       EQU 0x0058       ; External input Clock monitoring Interrupt
E2CINTaddr       EQU 0x005a       ; EEPROM Error Correction Interrupt
ERDYaddr       EQU 0x005c       ; EEPROM Ready Interrupt
SPMRaddr       EQU 0x005e       ; Store Program Memory Ready
TWI1addr       EQU 0x0060       ; TWI1 Interrupt
SPI2addr       EQU 0x0062       ; SPI2 Interrupt
TWI2addr       EQU 0x0064       ; TWI2 Interrupt
INT_VECTORS_SIZE       EQU 102       ; size in words
#endif /* _REGDEF_INC_ */
; ***** END OF FILE ******************************************************
