@echo off

set COM_PORT=COM4

mcumgr conn add testDK type="serial" connstring="%COM_PORT%,baud=115200,mtu=1024"

mcumgr -c testDK image upload ./bin/zephyr.signed.bin

mcumgr -c testDK reset

pause
