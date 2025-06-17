@echo off

set COM_PORT=COM11

mcumgr conn add testDK type="serial" connstring="%COM_PORT%,baud=115200,mtu=1024"

mcumgr -c testDK image upload ./build/nrf/zephyr/zephyr.signed.bin

mcumgr -c testDK reset

pause
