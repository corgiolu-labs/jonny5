@echo off
REM deploy.bat — Lancia deploy.sh sul Pi tramite Git Bash / WSL
REM Utilizzo:  deploy.bat [PI_HOST]
REM Esempio:   deploy.bat jonny5@192.168.1.50

SET PI_HOST=%1
IF "%PI_HOST%"=="" SET PI_HOST=jonny5@10.42.0.1

REM Prova Git Bash
WHERE bash >nul 2>&1
IF %ERRORLEVEL%==0 (
    bash "%~dp0deploy.sh" %PI_HOST%
    GOTO END
)

REM Prova WSL
WHERE wsl >nul 2>&1
IF %ERRORLEVEL%==0 (
    wsl bash "$(wslpath '%~dp0deploy.sh')" %PI_HOST%
    GOTO END
)

echo ERRORE: bash non trovato. Installa Git for Windows oppure WSL.
exit /b 1

:END
