#!/bin/sh

# Seeed_Grove_Vision_AI_Module_V2 repository folder
SGV_INST_PATH=/tmp/Seeed_Grove_Vision_AI_Module_V2

EPII_ROOT=$SGV_INST_PATH/EPII_CM55M_APP_S
IMAGE_GEN=$SGV_INST_PATH/we2_image_gen_local
CUR_DIR=$(pwd)

# make clean EPII_ROOT=$EPII_ROOT
# if [ $? -ne 0 ]; then
#     exit
# fi

make EPII_ROOT=$EPII_ROOT
if [ $? -ne 0 ]; then
    exit
fi

cp build/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf $IMAGE_GEN/input_case1_secboot
cp build/EPII_CM55M_gnu_epii_evb_WLCSP65_s.map $IMAGE_GEN/input_case1_secboot

cd $IMAGE_GEN

./we2_local_image_gen project_case1_blp_wlcsp.json

cd $CUR_DIR

cp $IMAGE_GEN/output_case1_sec_wlcsp/output.img build
