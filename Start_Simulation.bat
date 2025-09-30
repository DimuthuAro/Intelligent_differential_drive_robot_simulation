@echo off
setlocal enabledelayedexpansion
rem Self-Running Batch Script for Robot Simulation
rem Checks for Python and required libraries before starting simulation

title Robot Simulation Environment Checker
echo ========================================
echo Robot Simulation Environment Checker
echo ========================================
echo.

echo Step 1: Checking Python installation...

rem Check if Python is installed
python --version >nul 2>&1
if %errorlevel% equ 0 (
    for /f "tokens=*" %%i in ('python --version 2^>^&1') do set pythonVersion=%%i
    echo * Python OK - !pythonVersion!
    set pythonOK=1
) else (
    echo * Python is not installed
    echo Please install Python from https://python.org
    set pythonOK=0
)

if %pythonOK% equ 0 (
    echo.
    echo Please install Python before running the simulation.
    pause
    exit /b 1
)

echo.
echo Step 2: Checking required libraries...

set missingLibs=
set installedLibs=
set allLibsOK=1

rem Check pygame
python -c "import pygame; print('pygame installed')" >nul 2>&1
if %errorlevel% equ 0 (
    echo   * pygame - OK
    if defined installedLibs (
        set installedLibs=!installedLibs!, pygame
    ) else (
        set installedLibs=pygame
    )
) else (
    echo   * pygame - Not installed
    if defined missingLibs (
        set missingLibs=!missingLibs! pygame
    ) else (
        set missingLibs=pygame
    )
    set allLibsOK=0
)

rem Check matplotlib
python -c "import matplotlib; print('matplotlib installed')" >nul 2>&1
if %errorlevel% equ 0 (
    echo   * matplotlib - OK
    if defined installedLibs (
        set installedLibs=!installedLibs!, matplotlib
    ) else (
        set installedLibs=matplotlib
    )
) else (
    echo   * matplotlib - Not installed
    if defined missingLibs (
        set missingLibs=!missingLibs! matplotlib
    ) else (
        set missingLibs=matplotlib
    )
    set allLibsOK=0
)

rem Check numpy
python -c "import numpy; print('numpy installed')" >nul 2>&1
if %errorlevel% equ 0 (
    echo   * numpy - OK
    if defined installedLibs (
        set installedLibs=!installedLibs!, numpy
    ) else (
        set installedLibs=numpy
    )
) else (
    echo   * numpy - Not installed
    if defined missingLibs (
        set missingLibs=!missingLibs! numpy
    ) else (
        set missingLibs=numpy
    )
    set allLibsOK=0
)

rem Display results
if %allLibsOK% equ 1 (
    echo * All libraries OK: !installedLibs!
) else (
    echo * Missing libraries: !missingLibs!
    echo Install missing libraries with: pip install !missingLibs!
    echo.
    echo Please install missing libraries before running the simulation.
    pause
    exit /b 1
)

echo.
echo All requirements satisfied!
echo.
echo Starting Robot Simulation...
echo ========================================

rem Start the simulation - check for main files
if exist "robot_simulation.py" (
    python robot_simulation.py
) else if exist "main_state.py" (
    python main_state.py
) else (
    echo Error: No main simulation file found!
    echo Looking for: robot_simulation.py or main_state.py
    pause
    exit /b 1
)

rem Keep window open if there's an error
if %errorlevel% neq 0 (
    echo.
    echo Simulation ended with errors.
    pause
)