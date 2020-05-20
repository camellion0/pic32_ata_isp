@ECHO OFF
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~ $LastChangedRevision: 328482 $
REM ~~ $LastChangedDate: 2015-07-22 13:17:23 -0600 (Wed, 22 Jul 2015) $
REM ~~ $LastChangedBy: grueter $ 
REM ~~ $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/libFwROM/IAR/build.cmd $
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~ Created by: gwillbol  on: 30.Aug.2011
REM ~~ (c) Atmel Automotive GmbH
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SET CLOC_INPUT=%SANDBOX%\02_AutoRF\Primus2pSW\Trunk\firmware
SET CLOC_OUTPUT=.\doc\SrcDoc\clocLog.xml
SET CLOC_OPTIONS=--by-file-by-lang --force-lang="Assembly",s90 --force-lang="Assembly",inc --xml --xsl=%SANDBOX%\02_AutoRF\Primus2pSW\Trunk\doc\SrcDoc\clocstyle.xsl


IF [%1] == [] GOTO BUILD
IF [%1] == [build] GOTO BUILD
IF [%1] == [genRegisterFiles] GOTO GENREGISTERFILES
IF [%1] == [clean] GOTO CLEAN
IF [%1] == [lint] GOTO LINT
IF [%1] == [check] GOTO CHECK
IF [%1] == [doxygen] GOTO DOXYGEN
IF [%1] == [doxygen_INTERNAL] GOTO DOXYGEN_INTERNAL

:BUILD
    ECHO. building primus2p rom sw library
    
    cd .\appl\libFwROM\iar
    
    del Debug\Exe\*.d90     /s /q   > NUL
    del Debug\Exe\*.hct     /s /q   > NUL
    del Debug\Exe\*.hex     /s /q   > NUL
    del Debug\Exe\*.v       /s /q   > NUL
    del Debug\List\*.map    /s /q   > NUL
    
    call iarbuild libFwROM.ewp -build Debug

    call perl %SANDBOX%\02_AutoRF\IP_SW\tools\pl\hex2hct_iar.pl Debug\exe\libFwROM.hex Debug\exe\libFwROM.hct > NUL
    call perl %SANDBOX%\02_AutoRF\IP_SW\tools\pl\extractLinkerInfo_iar.pl Debug\List\libFwROM.map > NUL

    call perl updateFlashLinkerFile.pl ..\..\appFlash\IAR\ATA5700_flash.master output_symbols.txt ..\..\appFLASH\IAR\ATA5700_flash.xcl > NUL
    call perl updateFlashLinkerFile.pl ..\..\appFlash_simTest\IAR\ATA5700_flash.master output_symbols.txt ..\..\appFlash_simTest\IAR\ATA5700_flash.xcl > NUL
    
    call perl %SANDBOX%\02_AutoRF\IP_SW\tools\pl\extractSimInfo_iar.pl .\Debug\List\libFwROM.map .\..\..\..\firmware "STIM" > .\..\..\libFwROM\iar\Debug\Exe\libFwROM_defines.v ""
    REM call perl %SANDBOX%\02_AutoRF\IP_SW\tools\pl\extractSimInfo_iar.pl %SANDBOX%\02_AutoRF\Primus2pSW\Trunk\appl\libFwROM\iar\Debug\List\libFwROM.map %SANDBOX%\02_AutoRF\Primus2pSW\Trunk\firmware "PYTHON" > %SANDBOX%\02_AutoRF\Primus2pSW\tools\TestFramework\Implementation\TestExecutionManager\Utilities\Defines\SigmaXFwROMDefines_10.py
    REM call perl %SANDBOX%\02_AutoRF\IP_SW\tools\pl\extractSimInfo_iar.pl %SANDBOX%\02_AutoRF\Primus2pSW\Trunk\appl\libFwROM\iar\Debug\List\libFwROM.map %SANDBOX%\02_AutoRF\Primus2pSW\Trunk\firmware "LUA" > %SANDBOX%\02_AutoRF\Primus2pSW\Trunk\appl\fwROM\iar\Debug\Exe\libFwROM_defines.lua
    
    cd .\..\..\..

    ECHO. building primus2p rom sw library finished

    GOTO END

:GENREGISTERFILES
    
    rem generate ioATA5700.h
    call "%AVRSTUDIO_PATH%/xmlconvert" -c -fiar -b -l -1 -o.\ %SANDBOX%\02_AutoRF\IP_SW\tools\avrstudio\ATA5700.xml
    call perl %SANDBOX%/02_AutoRF/IP_SW/tools/pl/convert2IarH_ATA5700.pl ioATA5700.h .\firmware\stdc\src\ioATA5700.h
    
    rem generate regs.inc
    call "%AVRSTUDIO_PATH%/xmlconvert" -c -favrasm -b -l -1 -o .\ %SANDBOX%\02_AutoRF\IP_SW\tools\avrstudio\ATA5700.xml
    call perl %SANDBOX%/02_AutoRF/IP_SW/tools/pl/convert2IarInc.pl ATA5700def.inc .\firmware\stdc\src\regs.inc
    GOTO END

:CLEAN
    ECHO. cleanup of primus2p rom sw library
    cd .\appl\libFwROM\iar

    call iarbuild libFwROM.ewp -clean Debug

    cd .\..\..\..

    ECHO. cleanup of primus2p rom sw library finished
    
    GOTO END
    
:LINT
    ECHO. lint checking of primus2p rom sw
    cd .\tools\lint
    SET LINT_DIR=..\..\appl\libFwROM\IAR\Lint
    call lint %LINT_DIR%\lint_files.lnt %LINT_DIR%\libFwRom_lint.log
    cd .\..\..
    ECHO. lint checking of primus2p rom sw library finished
    
    GOTO END
    
:CHECK
    cd .\firmware
    call python ..\appl\libFwROM\IAR\check_regIO.py > ..\appl\libFwROM\IAR\check_regIO.log 
    call python ..\appl\libFwROM\IAR\check_pushpop.py > ..\appl\libFwROM\IAR\check_pushpop.log 
    call python ..\appl\libFwROM\IAR\asm_c_check.py > ..\appl\libFwROM\IAR\asm_c_check.log
    
    cd .\..

    GOTO END

:DOXYGEN
    cd .\doc\SrcDoc
    call perl %SANDBOX%\02_AutoRF\IP_SW\tools\pl\genDoxyFiles.pl %SANDBOX%\02_AutoRF\Primus2pSW\Trunk\firmware    
    call doxygen Doxyfile
    cd .\..\..
    
    GOTO END
    
:DOXYGEN_INTERNAL
    SET SVN_PATH=http://svnservulm.corp.atmel.com/svn/CDB/02_AutoRF/Primus2pSW/Trunk

    call svn log %SVN_PATH% -v -l50 --xml | xsltproc .\doc\SrcDoc\svnstyle.xsl - > .\doc\SrcDoc\log.html

    call cloc-1.56.exe %CLOC_INPUT% %CLOC_OPTIONS% --out=%CLOC_OUTPUT%
    call xsltproc doc\SrcDoc\clocstyle.xsl %CLOC_OUTPUT% > doc\SrcDoc\cloc.html


    cd .\appl\libFwROM\IAR\Debug\List
    call perl %SANDBOX%\02_AutoRF\IP_SW\tools\pl\extractMemUsage.pl
    cd .\..\..\..\..\..
      
    cd .\doc\SrcDoc
    call perl %SANDBOX%\02_AutoRF\IP_SW\tools\pl\genDoxyFiles.pl %SANDBOX%\02_AutoRF\Primus2pSW\Trunk\firmware
    call doxygen Doxyfile_INTERNAL
    cd .\..\..
    
    GOTO END
    
:END
