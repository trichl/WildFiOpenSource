@echo off

echo ============================
echo Flashing..
echo ============================

call ..\..\..\esp-idf-v4.1\export.bat

echo ============================

mode

echo ============================
echo Exit console with STRG and +
echo ============================

set /p COMNUM="Enter COM Port: "
idf.py monitor -p COM%COMNUM%

pause