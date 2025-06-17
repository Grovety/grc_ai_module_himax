@echo off

set COM_PORT=com9

python flasher/flasher.py %COM_PORT% --file_path=build\output.img

rem the alternative method
rem python flasher\xmodem\xmodem_send.py --port=%COM_PORT% --baudrate=921600 --protocol=xmodem --file=build\output.img

pause
