@echo off
echo ========================================
echo    CREATING DESKTOP SHORTCUTS
echo ========================================
echo.
echo This will create desktop shortcuts for easy access
echo.

REM Get the current directory
set "CURRENT_DIR=%~dp0"

REM Get the desktop path
set "DESKTOP=%USERPROFILE%\Desktop"

echo Creating shortcuts on desktop...

REM Create shortcut for main launcher
echo Creating main launcher shortcut...
powershell -Command "$WshShell = New-Object -comObject WScript.Shell; $Shortcut = $WshShell.CreateShortcut('%DESKTOP%\Drone Navigation System.lnk'); $Shortcut.TargetPath = '%CURRENT_DIR%START_DRONE_NAVIGATION.bat'; $Shortcut.WorkingDirectory = '%CURRENT_DIR%'; $Shortcut.Description = 'Autonomous Drone Navigation System'; $Shortcut.Save()"

REM Create shortcut for 3D environment
echo Creating 3D environment shortcut...
powershell -Command "$WshShell = New-Object -comObject WScript.Shell; $Shortcut = $WshShell.CreateShortcut('%DESKTOP%\Drone 3D Environment.lnk'); $Shortcut.TargetPath = '%CURRENT_DIR%start_drone_3d.bat'; $Shortcut.WorkingDirectory = '%CURRENT_DIR%'; $Shortcut.Description = '3D Drone Navigation Environment'; $Shortcut.Save()"

REM Create shortcut for simple drone
echo Creating simple drone shortcut...
powershell -Command "$WshShell = New-Object -comObject WScript.Shell; $Shortcut = $WshShell.CreateShortcut('%DESKTOP%\Simple Drone.lnk'); $Shortcut.TargetPath = '%CURRENT_DIR%start_drone_simple.bat'; $Shortcut.WorkingDirectory = '%CURRENT_DIR%'; $Shortcut.Description = 'Simple Drone Navigation'; $Shortcut.Save()"

echo.
echo Desktop shortcuts created successfully!
echo You can now double-click the shortcuts on your desktop to run the simulations.
echo.
pause
