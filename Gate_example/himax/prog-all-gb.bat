@echo off

set serial_port=com5
set models_dir=./bin
set firmware_bin=./bin
set flasher_dir=./flasher

echo Open Serial Port %serial_port%
echo Device init successfully
echo Please press reset button

start cmd.exe /k python %flasher_dir%/flasher.py %serial_port% --file_path=%firmware_bin%/dummy.img
echo Press any key after closing the child console..
pause > NUL

start cmd.exe /k python %flasher_dir%/xmodem/xmodem_send.py --port=%serial_port% --baudrate=921600 --protocol=xmodem --model=%models_dir%"/lpr/lpr_256_no_ctc_10k_vela.tflite 0x200000 0x00000"
echo Press any key after closing the child console..
pause > NUL

start cmd.exe /k python %flasher_dir%/xmodem/xmodem_send.py --port=%serial_port% --baudrate=921600 --protocol=xmodem --model=%models_dir%"/lpr/lpr_250k_512_india_vela.tflite 0x400000 0x00000"
echo Press any key after closing the child console..
pause > NUL

start cmd.exe /k python %flasher_dir%/xmodem/xmodem_send.py --port=%serial_port% --baudrate=921600 --protocol=xmodem --model=%models_dir%"/lpd/lpd_ssdlite_ocr_no_postprocess_2_vela.tflite 0x680000 0x00000"
echo Press any key after closing the child console..
pause > NUL

start cmd.exe /k python %flasher_dir%/xmodem/xmodem_send.py --port=%serial_port% --baudrate=921600 --protocol=xmodem --model=%models_dir%"/lpd/TFLite_Detection_PostProcess_float.tflite 0xB00000 0x00000"
echo Press any key after closing the child console..
pause > NUL

echo Open Serial Port %serial_port%
echo Device init successfully
echo Please press reset button

start cmd.exe /k python %flasher_dir%/flasher.py %serial_port% --file_path=%firmware_bin%/output.img
echo Press any key after closing the child console..
pause > NUL
