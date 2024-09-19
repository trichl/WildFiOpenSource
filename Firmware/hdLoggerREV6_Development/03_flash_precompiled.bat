@echo off

echo ============================
echo Flashing..
echo ============================

call ..\..\..\esp-idf-v4.1\export.bat

echo ============================

mode

echo ============================

set /p COMNUM="Enter COM Port: "
..\..\..\esp-idf-v4.1\components\esptool_py\esptool\esptool.py -p COM%COMNUM% -b 460800 --before default_reset --after hard_reset --chip esp32 write_flash --flash_mode dio --flash_size detect --flash_freq 80m 0x1000 _COMPILED\bootloader\bootloader.bin 0x8000 _COMPILED\partition_table\partition-table.bin 0x10000 _COMPILED\wildfi-firmware.bin

pause