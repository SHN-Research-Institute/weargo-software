@echo off
echo WearGo Software Setup
echo =====================
echo.
echo Activating virtual environment...
call ..\..venv\Scripts\activate
echo.
echo Virtual environment activated!
echo Current Python: %VIRTUAL_ENV%
echo.
echo To run the IMU visualizer:
echo   python imu_visualizer.py
echo.
echo To test with specific port:
echo   python imu_visualizer.py --port COM6
echo.
echo To run dependency test:
echo   python test_setup.py
echo.
cmd
