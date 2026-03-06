@echo off
call "E:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64
set PATH=E:\Qt\6.7.3\msvc2022_64\bin;F:\GuanZiwei\congliu\chenghao_bps\install\x64\vc16\bin;F:\GuanZiwei\PCL115~1.1\bin;F:\GuanZiwei\PCL115~1.1\3rdParty\VTK\bin;F:\GuanZiwei\congliu\chenghao_bps\PCL1.12.1\3rdParty\OpenNI2\Redist;C:\PROGRA~2\COMMON~1\MVS\Runtime\Win64_x64;%PATH%
cd /d F:\GuanZiwei\newProject
qmake SLCalibration.pro -spec win32-msvc "CONFIG+=debug"
nmake
