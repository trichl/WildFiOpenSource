@echo off

echo ============================
echo Compiling..
echo ============================

call ..\..\..\esp-idf-v4.1\export.bat
call idf.py build

pause