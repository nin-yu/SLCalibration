@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64
set PATH=D:\qt\6.7.3\msvc2022_64\bin;D:\opencv\newbuild\install\x64\vc16\bin;%PATH%
cd /d D:\qt\code\aBPS\SLCalibration\SLCalibration
qmake SLCalibration.pro -spec win32-msvc "CONFIG+=debug"
nmake