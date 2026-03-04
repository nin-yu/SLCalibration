@echo off
call "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvarsall.bat" x64 >nul 2>nul
set PATH=D:/qt/6.7.3/msvc2022_64/bin;D:/opencv/newbuild/install/x64/vc16/bin;%PATH%
cd /d D:/qt/code/aBPS/SLCalibration/SLCalibration
echo === QMAKE === > build_output.log 2>&1
qmake SLCalibration.pro -spec win32-msvc "CONFIG+=debug" >> build_output.log 2>&1
echo QMAKE_EXIT=%ERRORLEVEL% >> build_output.log
echo === NMAKE === >> build_output.log 2>&1
nmake >> build_output.log 2>&1
echo NMAKE_EXIT=%ERRORLEVEL% >> build_output.log
