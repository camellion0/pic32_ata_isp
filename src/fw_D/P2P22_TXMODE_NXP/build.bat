@ECHO OFF
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~ $LastChangedRevision: 383329 $
REM ~~ $LastChangedDate: 2016-04-14 14:41:33 -0600 (Thu, 14 Apr 2016) $
REM ~~ $LastChangedBy: grueter $ 
REM ~~ $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/build.bat $
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~ Created by: gwillbol  on: 27.Sep.2012
REM ~~ (c) Atmel Automotive GmbH
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
IF [%1] == [all] GOTO ALL
IF [%1] == [clib] GOTO CLIB
IF [%1] == [appFlash] GOTO APPFLASH
IF [%1] == [appFlash_simTest] GOTO APPFLASHSIMTEST
IF [%1] == [appFlash_customerExample] GOTO APPFLASHCUSTOMEREXAMPLE
IF [%1] == [appImmobilizer] GOTO APPIMMOBILIZER
IF [%1] == [libFwRom] GOTO LIBFWROM
IF [%1] == [doxygen] GOTO DOXYGEN
IF [%1] == [lint] GOTO LINT
IF [%1] == [dev] GOTO LIBFWROM

ECHO possible targets are
ECHO     - all
ECHO     - clib
ECHO     - appFlash
ECHO     - appFlash_simTest
ECHO     - appFlash_customerExample
ECHO     - appImmobilizer
ECHO     - libFwRom
ECHO     - doxygen
ECHO     - lint

GOTO END

:ALL
IF [%1] == [all] GOTO CLIB
GOTO END

:CLIB
call p2p_make.bat clib
IF [%1] == [all] GOTO LIBFWROM
GOTO END

:LIBFWROM
call p2p_make.bat libFwRom
rem copy .\appl\libFwROM\Iar\Debug\Exe\libFwROM.hct T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\libFwROM\IAR\Debug\Exe\libFwROM.hct
rem copy .\appl\libFwROM\Iar\Debug\Exe\libFwROM.hex T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\libFwROM\IAR\Debug\Exe\libFwROM.hex
rem copy .\appl\libFwROM\Iar\Debug\Exe\libFwROM_defines.v T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\libFwROM\IAR\Debug\Exe\libFwROM_defines.v
rem copy .\appl\libFwROM\Iar\Debug\List\libFwROM.map T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\libFwROM\IAR\Debug\List\libFwROM.map

IF [%1] == [all] GOTO APPFLASH
rem IF [%1] == [dev] GOTO APPFLASH
IF [%1] == [dev] GOTO APPFLASHSIMTEST
GOTO END

:APPFLASH
call p2p_make.bat appFlash
rem copy .\appl\appFlash\iar\Debug\Exe\appFLASH.hct T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appFlash\IAR\Debug\Exe\appFLASH.hct
rem copy .\appl\appFlash\iar\Debug\Exe\fwROMFLASH.hct T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appFlash\IAR\Debug\Exe\fwROMFLASH.hct
rem copy .\appl\appFlash\iar\Debug\Exe\appFLASH.hex T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appFlash\IAR\Debug\Exe\appFLASH.hex
rem copy .\appl\appFlash\iar\Debug\List\appFLASH.map T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appFlash\IAR\Debug\List\appFLASH.map
IF [%1] == [all] GOTO APPFLASHSIMTEST
GOTO END

:APPFLASHSIMTEST
call p2p_make.bat appFlash_simTest
rem copy .\appl\appFlash_simTest\iar\Debug\Exe\appFLASH.hct   T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appFlash_simTest\IAR\Debug\Exe\appFLASH.hct
rem copy .\appl\appFlash_simTest\iar\Debug\Exe\fwROMFLASH.hct T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appFlash_simTest\IAR\Debug\Exe\fwROMFLASH.hct
rem copy .\appl\appFlash_simTest\iar\Debug\Exe\appFLASH.hex   T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appFlash_simTest\IAR\Debug\Exe\appFLASH.hex
rem copy .\appl\appFlash_simTest\iar\Debug\List\appFLASH.map  T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appFlash_simTest\IAR\Debug\List\appFLASH.map
IF [%1] == [all] GOTO APPFLASHCUSTOMEREXAMPLE
GOTO END

:APPFLASHCUSTOMEREXAMPLE
call p2p_make.bat appFlash_customerExample
rem copy .\appl\appFlash_simTest\iar\Debug\Exe\appFLASH.hct   T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appFlash_simTest\IAR\Debug\Exe\appFLASH.hct
rem copy .\appl\appFlash_simTest\iar\Debug\Exe\fwROMFLASH.hct T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appFlash_simTest\IAR\Debug\Exe\fwROMFLASH.hct
rem copy .\appl\appFlash_simTest\iar\Debug\Exe\appFLASH.hex   T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appFlash_simTest\IAR\Debug\Exe\appFLASH.hex
rem copy .\appl\appFlash_simTest\iar\Debug\List\appFLASH.map  T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appFlash_simTest\IAR\Debug\List\appFLASH.map
IF [%1] == [all] GOTO APPIMMOBILIZER
GOTO END

:APPIMMOBILIZER
call p2p_make.bat appImmobilizer
rem copy .\appl\appImmobilizer\iar\Debug\Exe\appImmobilizer.hct T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appImmobilizer\IAR\Debug\Exe\appImmobilizer.hct
rem copy .\appl\appImmobilizer\iar\Debug\Exe\appImmobilizer.hex T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appImmobilizer\IAR\Debug\Exe\appImmobilizer.hex
rem copy .\appl\appImmobilizer\iar\Debug\List\appImmobilizer.map T:\all_projects\primus2p\vdev\home\gwillbol\ROM10\design\software\appl\appImmobilizer\IAR\Debug\List\appImmobilizer.map
IF [%1] == [all] GOTO DOXYGEN
GOTO END

:DOXYGEN
if [%2] == [internal] GOTO DOXGEN_INTERNAL
call p2p_make.bat doxygen 
GOTO END
:DOXGEN_INTERNAL
call p2p_make.bat doxygen internal
GOTO END

:LINT
call p2p_make.bat lint
GOTO END

:END
