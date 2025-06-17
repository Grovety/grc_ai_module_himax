#!/bin/sh

TTY_DEV=/dev/ttyUSB0

python3 ./flasher/flasher.py $TTY_DEV --file_path=./build/output.img
