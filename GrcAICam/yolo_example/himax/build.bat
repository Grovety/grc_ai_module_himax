@echo off

set GNU_MAKE_PATH=c:\xpack-windows-build-tools-4.4.1-3\bin
set SGV_INST_PATH=c:\Users\Vladimir.Shalyt\work\Seeed_Grove_Vision_AI_Module_V2

set EPII_ROOT=%SGV_INST_PATH%\EPII_CM55M_APP_S
set IMAGE_GEN=%SGV_INST_PATH%\we2_image_gen_local

rem %GNU_MAKE_PATH%\make clean EPII_ROOT=%EPII_ROOT%
rem if ERRORLEVEL 1 goto script_exit

%GNU_MAKE_PATH%\make EPII_ROOT=%EPII_ROOT%
if ERRORLEVEL 1 goto script_exit

rem pause

copy build\EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf %IMAGE_GEN%\input_case1_secboot
copy build\EPII_CM55M_gnu_epii_evb_WLCSP65_s.map %IMAGE_GEN%\input_case1_secboot

pushd %IMAGE_GEN%

we2_local_image_gen.exe project_case1_blp_wlcsp.json

popd

copy %IMAGE_GEN%\output_case1_sec_wlcsp\output.img build

:script_exit

pause
