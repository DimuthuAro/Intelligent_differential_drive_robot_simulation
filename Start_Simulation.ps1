# Self-Running PowerShell Script for Robot Simulation
# Checks for Python and required libraries before starting simulation

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Robot Simulation Environment Checker" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Function to check if Python is installed
function Check-Python {
    try {
        $pythonVersion = python --version 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-Host "* Python OK - $pythonVersion" -ForegroundColor Green
            return $true
        }
        else {
            Write-Host "* Python is not installed" -ForegroundColor Red
            Write-Host "Please install Python from https://python.org" -ForegroundColor Yellow
            return $false
        }
    }
    catch {
        Write-Host "* Python is not installed" -ForegroundColor Red
        Write-Host "Please install Python from https://python.org" -ForegroundColor Yellow
        return $false
    }
}

# Function to check required Python libraries
function Check-Libraries {
    $requiredLibraries = @("pygame", "matplotlib", "numpy")
    $missingLibraries = @()
    $installedLibraries = @()
    
    Write-Host "Checking required Python libraries..." -ForegroundColor Yellow
    
    foreach ($library in $requiredLibraries) {
        try {
            $output = python -c "import $library; print('$library installed')" 2>&1
            if ($LASTEXITCODE -eq 0) {
                $installedLibraries += $library
                Write-Host "  * $library - OK" -ForegroundColor Green
            }
            else {
                $missingLibraries += $library
                Write-Host "  * $library - Not installed" -ForegroundColor Red
            }
        }
        catch {
            $missingLibraries += $library
            Write-Host "  * $library - Not installed" -ForegroundColor Red
        }
    }
    
    if ($missingLibraries.Count -eq 0) {
        Write-Host "* All libraries OK: $($installedLibraries -join ', ')" -ForegroundColor Green
        return $true
    }
    else {
        Write-Host "* Missing libraries: $($missingLibraries -join ', ')" -ForegroundColor Red
        Write-Host "Install missing libraries with: pip install $($missingLibraries -join ' ')" -ForegroundColor Yellow
        return $false
    }
}

# Function to start the simulation
function Start-Simulation {
    Write-Host ""
    Write-Host "Starting Robot Simulation..." -ForegroundColor Cyan
    Write-Host "========================================" -ForegroundColor Cyan
    
    # Check if main simulation file exists
    if (Test-Path "robot_simulation.py") {
        python robot_simulation.py
    }
    elseif (Test-Path "main_state.py") {
        python main_state.py
    }
    else {
        Write-Host "Error: No main simulation file found!" -ForegroundColor Red
        Write-Host "Looking for: robot_simulation.py or main_state.py" -ForegroundColor Yellow
    }
}

# Main execution
Write-Host "Step 1: Checking Python installation..." -ForegroundColor Yellow
$pythonOK = Check-Python

if ($pythonOK) {
    Write-Host ""
    Write-Host "Step 2: Checking required libraries..." -ForegroundColor Yellow
    $librariesOK = Check-Libraries
    
    if ($librariesOK) {
        Write-Host ""
        Write-Host "All requirements satisfied!" -ForegroundColor Green
        Start-Simulation
    }
    else {
        Write-Host ""
        Write-Host "Please install missing libraries before running the simulation." -ForegroundColor Red
        Write-Host "Press any key to exit..."
        $null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
    }
}
else {
    Write-Host ""
    Write-Host "Please install Python before running the simulation." -ForegroundColor Red
    Write-Host "Press any key to exit..."
    $null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
}