@ECHO OFF
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~ $LastChangedRevision: 328482 $
REM ~~ $LastChangedDate: 2015-07-22 13:17:23 -0600 (Wed, 22 Jul 2015) $
REM ~~ $LastChangedBy: grueter $ 
REM ~~ $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/lib/build.cmd $
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~ Created by: gwillbol  on: 30.Aug.2011
REM ~~ (c) Atmel Automotive GmbH
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

IF [%1] == [] GOTO BUILD
IF [%1] == [build] GOTO BUILD
IF [%1] == [clean] GOTO CLEAN

:BUILD
    ECHO. building clib
    cd .\lib
    
    call iarbuild projects\clib\cl3s-ec_mul-sf.ewp -build Release
    
    cd .\..

    ECHO. clib build finished

    GOTO END

:CLEAN
    ECHO. cleanup of clib
    cd .\lib
    
    call iarbuild projects\clib\cl3s-ec_mul-sf.ewp -clean Release

    cd .\..

    ECHO. cleanup of clib finished
    
    GOTO END

:END
