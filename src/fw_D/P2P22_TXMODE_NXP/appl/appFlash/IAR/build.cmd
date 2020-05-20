@ECHO OFF
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~ $LastChangedRevision: 328482 $
REM ~~ $LastChangedDate: 2015-07-22 13:17:23 -0600 (Wed, 22 Jul 2015) $
REM ~~ $LastChangedBy: grueter $ 
REM ~~ $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/IAR/build.cmd $
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~ Created by: gwillbol  on: 30.Aug.2011
REM ~~ (c) Atmel Automotive GmbH
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

IF [%1] == [] GOTO BUILD
IF [%1] == [build] GOTO BUILD
IF [%1] == [lint] GOTO LINT
IF [%1] == [clean] GOTO CLEAN

:BUILD
    ECHO. building primus2p flash app sw
    
    cd .\appl\appFLASH\iar

    del Debug\Exe\*.d90     /s /q   > NUL
    del Debug\Exe\*.hct     /s /q   > NUL
    del Debug\Exe\*.hex     /s /q   > NUL
    del Debug\List\*.map    /s /q   > NUL
    
    call iarbuild appFLASH.ewp -build Debug
    
    call perl %SANDBOX%\02_AutoRF\IP_SW\tools\pl\hex2hct_iar.pl Debug\exe\appFLASH.hex Debug\exe\appFLASH.hct > nul
    
    rem combine libfwrom and appFlash
    call perl combineRomFlashHct.pl ..\..\libFwROM\IAR\Debug\Exe\libFwROM.hct Debug\Exe\appFLASH.hct Debug\Exe\fwROMFLASH.hct >nul
    
    call perl %SANDBOX%\02_AutoRF\IP_SW\tools\pl\extractSimInfo_iar.pl .\Debug\List\appFlash.map .\..\..\..\firmware "STIM" > .\Debug\Exe\appFlash_defines.v "appFlash_"   
    cd .\..\..\..

    ECHO. building primus2p flash app sw finished

    GOTO END
:LINT
    ECHO. lint checking of primus2p flash application appFlash
    cd .\tools\lint
    SET LINT_DIR=..\..\appl\appFlash\IAR\Lint
    call lint %LINT_DIR%\lint_files.lnt %LINT_DIR%\appFlash_lint.log
    cd .\..\..
    ECHO. lint checking of primus2p flash application appFlash finished
    GOTO END

:CLEAN
    ECHO. cleanup of primus2p flash app sw
    cd .\appl\appFLASH\iar

    call iarbuild appflash.ewp -clean Debug

    cd .\..\..\..

    ECHO. cleanup of primus2p flash app sw finished
    
    GOTO END

:END
