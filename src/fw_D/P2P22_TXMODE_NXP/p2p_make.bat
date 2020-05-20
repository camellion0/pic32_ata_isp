@ECHO OFF
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~ $LastChangedRevision: 383329 $
REM ~~ $LastChangedDate: 2016-04-14 14:41:33 -0600 (Thu, 14 Apr 2016) $
REM ~~ $LastChangedBy: grueter $ 
REM ~~ $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/p2p_make.bat $
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~ Created by: gwillbol  on: 27.Sep.2012
REM ~~ (c) Atmel Automotive GmbH
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM setup environment
call %SANDBOX%\Environment\PC_Tools\Scripts\setupenv.cmd --silent

call iar_environment.cmd

IF [%1] == [build] GOTO BUILD
IF [%1] == [genRegisterFiles] GOTO GENREGISTERFILES
IF [%1] == [appFlash] GOTO APPFLASH
IF [%1] == [appFlash_simTest] GOTO APPFLASHSIMTEST
REM IF [%1] == [appFlash_customerExample] GOTO APPFLASHCUSTOMEREXAMPLE
IF [%1] == [appImmobilizer] GOTO APPIMMOBILIZER
IF [%1] == [libFwRom] GOTO LIBFWROM
IF [%1] == [clib] GOTO CLIB
IF [%1] == [lint] GOTO LINT
IF [%1] == [automaticBuild] GOTO AUTOMATIC_BUILD
IF [%1] == [doxygen] GOTO DOXYGEN

ECHO possible targets are
ECHO    - build
ECHO    - genRegisterFiles
ECHO    - appFlash
ECHO    - appFlash_simTest
REM ECHO    - appFlash_customerExample
ECHO    - appImmobilizer
ECHO    - libFwRom
ECHO    - clib
ECHO    - lint
ECHO    - automaticBuild
ECHO    - doxygen
GOTO END

:BUILD
    call .\lib\build.cmd build
    call .\appl\libFwROM\IAR\build.cmd build
    call .\appl\appFlash\IAR\build.cmd build
    call .\appl\appFlash_simTest\IAR\build.cmd build
    REM call .\appl\appFlash_customerExample\IAR\build.cmd build
    call .\appl\appImmobilizer\IAR\build.cmd build
    GOTO END

:GENREGISTERFILES
    call .\appl\libFwROM\IAR\build.cmd genRegisterFiles
    GOTO END

:APPFLASH
    call .\appl\appFlash\IAR\build.cmd build
    GOTO END

:APPFLASHSIMTEST
    call .\appl\appFlash_simTest\IAR\build.cmd build
    GOTO END

REM :APPFLASHCUSTOMEREXAMPLE
REM     call .\appl\appFlash_customerExample\IAR\build.cmd build
REM     GOTO END
    
:APPIMMOBILIZER
    call .\appl\appImmobilizer\IAR\build.cmd build
    GOTO END

:LIBFWROM
    call .\appl\libFwROM\IAR\build.cmd build
    GOTO END

:CLIB
    call .\lib\build.cmd build
    GOTO END
    
:LINT
    call .\appl\libFwROM\IAR\build.cmd lint
    call .\appl\appFlash\IAR\build.cmd lint
    call .\appl\appFlash_simTest\IAR\build.cmd lint
    REM call .\appl\appFlash_customerExample\IAR\build.cmd lint
    call .\appl\appImmobilizer\IAR\build.cmd lint
    GOTO END
    
:AUTOMATIC_BUILD
    ECHO. automatic build of Primus2p SW

    del *.hex /s /q
    del *.hct /s /q
    SET WORKINGPATH=%cd%

    call sb up 02_AutoRF IP_SW Trunk
    call sb up 02_AutoRF Primus2pSW Trunk

    cd %WORKINGPATH%
    
    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    REM CLIB build
    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    call .\lib\build.cmd build
    ECHO. CLIB build finished
    REM <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    REM libFwROM
    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    call .\appl\libFwROM\iar\build.cmd genRegisterFiles
    call .\appl\libFwROM\iar\build.cmd build
    call .\appl\libFwROM\iar\build.cmd lint
    ECHO. libFwROM build finished
    REM <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    REM appFlash
    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    call .\appl\appFlash\iar\build.cmd build
    call .\appl\appFlash\iar\build.cmd lint
    ECHO. appFlash build finished
    REM <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    REM appFlashSimTest
    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    call .\appl\appFlash_simTest\iar\build.cmd build
    call .\appl\appFlash_simTest\iar\build.cmd lint
    ECHO. appFlashSimTest build finished
    REM <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	
    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    REM appFlashCustomerExample
    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    REM call .\appl\appFlash_customerExample\iar\build.cmd build
    REM ECHO. appFlashCustomerExample build finished
    REM <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    REM appImmobilizer
    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    call .\appl\appImmobilizer\iar\build.cmd build
    call .\appl\appImmobilizer\iar\build.cmd lint
    ECHO. appImmobilizer build finished
    REM <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    REM commit build results
    REM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    call svn ci -m"check in of results of automatic build"
    REM <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
   
    ECHO. automatic build finished @ %DATE% %TIME%
    GOTO END

    
:DOXYGEN
    IF [%2] == [internal] GOTO DOXYGEN_INTERNAL
    call .\appl\appImmobilizer\iar\build.cmd doxygen
    call .\appl\libFwROM\IAR\build.cmd doxygen
    GOTO END

:DOXYGEN_INTERNAL
    call .\appl\appImmobilizer\iar\build.cmd doxygen_INTERNAL
    call .\appl\libFwROM\IAR\build.cmd doxygen_INTERNAL
    GOTO END

:END    
