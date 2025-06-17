#!/bin/bash

TTY_DEV=/dev/ttyACM0

mcumgr conn add testDK type="serial" connstring="$TTY_DEV,baud=115200,mtu=1024"

sleep 1

mcumgr -c testDK image upload ./build/nrf/zephyr/zephyr.signed.bin

sleep 1

mcumgr -c testDK reset
