#!/bin/sh

TTY_DEV=/dev/ttyUSB0

MODEL_FILE=./model/yolov8n_full_integer_quant_vela.tflite

MEM_OFFSET=0xB10000

python3 flasher/xmodem/xmodem_send.py --port=$TTY_DEV --baudrate=921600 --protocol=xmodem --model="$MODEL_FILE $MEM_OFFSET 0x00000"
