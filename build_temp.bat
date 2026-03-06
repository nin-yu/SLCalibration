@echo off
call "E:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>nul
set PATH=E:\Qt\6.7.3\msvc2022_64\bin;F:\GuanZiwei\congliu\chenghao_bps\install\x64\vc16\bin;F:\GuanZiwei\PCL115~1.1\bin;F:\GuanZiwei\PCL115~1.1\3rdParty\VTK\bin;F:\GuanZiwei\congliu\chenghao_bps\PCL1.12.1\3rdParty\OpenNI2\Redist;C:\PROGRA~2\COMMON~1\MVS\Runtime\Win64_x64;%PATH%
cd /d F:\GuanZiwei\newProject
echo === QMAKE === > build_output.log 2>&1
qmake SLCalibration.pro -spec win32-msvc "CONFIG+=debug" >> build_output.log 2>&1
echo QMAKE_EXIT=%ERRORLEVEL% >> build_output.log
echo === NMAKE === >> build_output.log 2>&1
nmake >> build_output.log 2>&1
echo NMAKE_EXIT=%ERRORLEVEL% >> build_output.log
