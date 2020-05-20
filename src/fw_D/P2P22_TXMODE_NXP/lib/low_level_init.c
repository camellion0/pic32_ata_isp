/**************************************************************
 **             - __low_level_init.c -
 **
 **     Special initializations that are performed before
 **     segment initialization. It is also possible to
 **     completely block the normal segment initialization.
 **
 **     Used with iccAVR.
 **
 **     $Revision: 328482 $
 **
 **************************************************************/
#include <intrinsics.h>

extern void *__RSTACK_in_external_ram;

/**************************************************************
 **
 ** How to implement a low-level initialization function in C
 ** =========================================================
 **
 ** 1) Only use local auto variables.
 ** 2) Don't use global or static variables.
 ** 3) Don't use global or static objects (EC++ only).
 ** 4) Don't use agregate initializers, e.g. struct a b = {1};
 ** 5) Don't call any library functions (function calls that
 **    the compiler generates, e.g. to do integer math, are OK).
 ** 6) Setup the RSTACK as is appropriate! 
 **    See code below or use the command line --enable_external_bus
 **
 **************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
char __low_level_init()
{
  /* Uncomment the statement below if the RSTACK */
  /* segment has been placed in external SRAM!   */

  /* __require(__RSTACK_in_external_ram); */
  /* Or */
  /* __require(__RSTACK_in_external_ram_new_way); */
  /* Plus, if you uncomment this you will have to */
  /* define the sfr __?XMCRA in assembler         */

  /* If the low-level initialization routine is  */
  /* written in assembler, the line above should */
  /* be written as:                              */
  /*     EXTERN  __RSTACK_in_external_ram        */
  /*     REQUIRE __RSTACK_in_external_ram        */

  /* Add your custom setup here. */
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
    /* R16 used as return value */
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

  /* Return 1 to indicate that normal segment */
  /* initialization should be performed. If   */
  /* normal segment initialization should not */
  /* be performed, return 0.                  */
  return 1;
}
#ifdef __cplusplus
}
#endif

