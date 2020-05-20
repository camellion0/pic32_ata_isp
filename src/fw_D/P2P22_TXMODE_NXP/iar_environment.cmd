@ECHO OFF
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~ $LastChangedRevision: 383329 $
REM ~~ $LastChangedDate: 2016-04-14 14:41:33 -0600 (Thu, 14 Apr 2016) $
REM ~~ $LastChangedBy: grueter $ 
REM ~~ $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/iar_environment.cmd $
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~ Created by: gwillbol  on: 23.May.2012
REM ~~ (c) Atmel Automotive GmbH
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

IF NOT [%1] == [-h] GOTO BEGIN
ECHO.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ECHO.   Purpose: 
ECHO.   IAR Build
ECHO.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
GOTO TERMINATION

:BEGIN
ECHO. ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ECHO. IAR Build Batch V1.0 [for Windows platforms]
ECHO. by $LastChangedBy: grueter $  / ATMEL Automotive GmbH   $LastChangedDate: 2016-04-14 14:41:33 -0600 (Thu, 14 Apr 2016) $
ECHO. $Rev: 383329 $  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/iar_environment.cmd $
ECHO. ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM add IAR path to PATH variable
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SET IAR_PATH=%ProgramFiles(x86)%\IAR Systems\Embedded Workbench 6.0\common
IF EXIST "%IAR_PATH%" GOTO SET_IAR_PATH
SET IAR_PATH=%ProgramFiles%\IAR Systems\Embedded Workbench 6.0\common
:SET_IAR_PATH
SET PATH=%IAR_PATH%\bin;%PATH%

REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM add AVR Studio path to PATH variable
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SET AVRSTUDIO_PATH=%ProgramFiles(x86)%\Atmel\AVR Tools\AvrStudio4
IF EXIST "%AVRSTUDIO_PATH%" GOTO SET_AVRSTUDIO_PATH
SET AVRSTUDIO_PATH=%ProgramFiles%\Atmel\AVR Tools\AvrStudio4
:SET_AVRSTUDIO_PATH
SET PATH=%AVRSTUDIO_PATH%;%PATH%

SET CLOC_PATH=%SANDBOX%\02_AutoRF\IP_SW\tools\cloc
SET PATH=%CLOC_PATH%;%PATH%

SET XSLTPROC_PATH=%SANDBOX%\02_AutoRF\IP_SW\tools\libxml\bin
SET PATH=%XSLTPROC_PATH%;%PATH%

REM ~~~~ END ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:END

:TERMINATION
