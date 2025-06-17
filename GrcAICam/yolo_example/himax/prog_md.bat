@echo off

set COM_PORT=com12

set MODEL_FILE=model\yolov8n_full_integer_quant_vela.tflite

set MEM_OFFSET=0xB10000

python flasher\xmodem\xmodem_send.py --port=%COM_PORT% --baudrate=921600 --protocol=xmodem --model="%MODEL_FILE% %MEM_OFFSET% 0x00000"

pause
