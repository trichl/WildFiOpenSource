@echo off

echo ============================
echo Flashing..
echo ============================

call ..\..\..\esp-idf-v4.1\export.bat

echo ============================

mode

set /p COMNUM="Enter COM Port: "
espefuse.py -p COM%COMNUM% set_flash_voltage 3.3V

pause