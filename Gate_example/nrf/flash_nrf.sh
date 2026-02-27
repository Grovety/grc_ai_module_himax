#!/bin/bash

# Configuration
TTY_DEV=/dev/ttyACM0

while getopts "p:" opt; do
  case $opt in
    p)
      TTY_DEV="$OPTARG"
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

BAUD_RATE=115200
MTU=1024
CONN_NAME="testDK"
IMAGE_FILE="./bin/zephyr.signed.bin"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "Using Port: ${GREEN}$TTY_DEV${NC}"

if [ ! -f "$IMAGE_FILE" ]; then
    echo -e "${RED}Error: Firmware file not found: $IMAGE_FILE${NC}"
    exit 1
fi

mcumgr conn delete $CONN_NAME 2>/dev/null

echo "Configuring connection..."
mcumgr conn add $CONN_NAME type="serial" connstring="dev=$TTY_DEV,baud=$BAUD_RATE,mtu=$MTU"
if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Failed to configure mcumgr connection.${NC}"
    exit 1
fi

sleep 1

echo "Uploading image..."
mcumgr -c $CONN_NAME image upload "$IMAGE_FILE"
if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Image upload failed.${NC}"
    exit 1
fi

sleep 1

echo "Resetting device..."
mcumgr -c $CONN_NAME reset
if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Reset failed.${NC}"
    exit 1
fi

echo -e "${GREEN}nRF Flashing Complete.${NC}"