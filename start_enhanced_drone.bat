@echo off
echo ========================================
echo   ENHANCED 3D DRONE NAVIGATION SYSTEM
echo ========================================
echo.
echo Starting Enhanced 3D Drone Navigation...
echo.
echo Features:
echo - Real-time obstacle editing during simulation
echo - Pause/Resume simulation (Press P)
echo - Smart collision detection prevents drone overlap
echo - Automatic path replanning around new obstacles
echo - Interactive obstacle placement and dragging
echo - 2D top-down view with real-time tracking
echo.
echo Controls:
echo - SPACE: Start simulation
echo - P: Pause/Resume simulation
echo - 1/2/3: Add cylinder/cube/sphere obstacles
echo - E: Toggle edit mode (drag obstacles)
echo - Click: Add obstacles or select for editing
echo - +/-: Increase/Decrease speed
echo - 0-9: Preset speeds (0=slow, 9=normal, 8=fast, 7=very fast)
echo - H: Show full help
echo.
echo Press Ctrl+C to stop the simulation
echo.

REM Change to the script directory
cd /d "%~dp0"

REM Create logs directory if it doesn't exist
if not exist "logs" mkdir logs

REM Log the start
echo [%date% %time%] - ENHANCED_3D_DRONE - STARTED - Real-time obstacle editing >> logs\run_log.txt

REM Check if virtual environment exists and activate if available
if exist "venv\Scripts\activate.bat" (
    echo Activating virtual environment...
    call venv\Scripts\activate.bat
    echo Installing required packages...
    venv\Scripts\python.exe -m pip install numpy matplotlib opencv-python PyQt5 --quiet
) else (
    echo Using system Python installation...
)

REM Run the stable 3D drone simulation
echo Launching enhanced 3D drone simulation...
python src\stable_3d_drone.py

REM Log the end
echo [%date% %time%] - ENHANCED_3D_DRONE - COMPLETED - Simulation finished normally >> logs\run_log.txt

echo.
echo Enhanced 3D simulation ended.
echo Check logs\run_log.txt for details.
pause
