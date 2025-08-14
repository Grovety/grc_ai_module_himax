import asyncio
import threading
import time
from datetime import datetime

import sys
sys.path.append("../tests")

from bleak import BleakClient, BleakScanner, BleakError
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

GRC_DEV_NAME = "GrcAllCam"
FIND_DEV_BY_NAME = True # 0-find by uuid,1-find by name
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

async def find_device(by_name):

    def match_nus_uuid(device: BLEDevice, adv: AdvertisementData):
        if UART_SERVICE_UUID.lower() in adv.service_uuids:
            return True
        return False

    def match_grc_name(device: BLEDevice, adv: AdvertisementData):
        if GRC_DEV_NAME == adv.local_name:
            return True
        return False

    filter = match_grc_name if by_name else match_nus_uuid

    print("Connecting...", end=" ")
    device = await BleakScanner.find_device_by_filter(match_grc_name)
    if device is None:
        print("no matching device found.")
    else:
        now = str(datetime.fromtimestamp(time.time())).split(".")
        print(f"[{now[0]}] device {device.name} has found: {device.address}.")
    return device

def read_loop(device, main_proc, main_argv):

    loop = None
    client = None
    rx_char = None
    rx_data = bytearray()
    rx_mutex = threading.Lock()
    is_connected = False

    def handle_disconnect(_: BleakClient):
        nonlocal is_connected
        for task in asyncio.all_tasks():
            task.cancel()
        is_connected = False
        print(" device has disconnected")

    def handle_rx(_: BleakGATTCharacteristic, data: bytearray):
        nonlocal rx_mutex, rx_data
        rx_mutex.acquire()
        rx_data += data
        rx_mutex.release()

    def write(tx_data):
        nonlocal loop, client, rx_char, rx_data, is_connected
        for iter in range(3):
            if is_connected == False:
                try:
                    loop.run_until_complete(client.connect())
                    nu_serv = client.services.get_service(UART_SERVICE_UUID)
                    rx_char = nu_serv.get_characteristic(UART_RX_CHAR_UUID)
                    loop.run_until_complete(client.start_notify(UART_TX_CHAR_UUID, handle_rx))
                    is_connected = True
                except:
                    print("!!!connect() retry")
                    continue
            #if is_connected == True: # it's unnecessary apparently..
            try:
                rx_data = b""
                loop.run_until_complete(client.write_gatt_char(rx_char, tx_data, response=True))
                ## print("tx:", tx_data)
                return True
            except:
                print("!!!write() failed")
                loop.run_until_complete(asyncio.sleep(1))
                break
        return False

    def read(ret_numb):
        nonlocal loop, rx_mutex, rx_data
        for tm_cnt in range(200):
            rx_mutex.acquire()
            if len(rx_data) >= ret_numb:
                rx_mutex.release()
                break
            rx_mutex.release()
            loop.run_until_complete(asyncio.sleep(0.01))
        rx_mutex.acquire()
        ret_data = rx_data[0:ret_numb]
        rx_data = rx_data[ret_numb:]
        rx_mutex.release()
        ## print("rx:", tm_cnt, ret_data)
        return ret_data

    def close():
        nonlocal loop, client
        print("Disconnecting...", end = "")
        try:
            loop.run_until_complete(client.stop_notify(UART_TX_CHAR_UUID))
            loop.run_until_complete(client.disconnect())
        except asyncio.exceptions.CancelledError:
            pass

    def handle_exception(loop, context):
        # print("asyncio exception handler")
        print("Trying to reconnect..") #?

    def init():
        nonlocal loop
        loop = asyncio.new_event_loop()
        loop.set_exception_handler(handle_exception)
        loop.run_until_complete(asyncio.sleep(1))

    client = BleakClient(device, disconnected_callback = handle_disconnect)
    main_proc(init, close, write, read, main_argv)

def bluetooth_wrapper(main_proc, main_argv = None):
    device = asyncio.run(find_device(True))
    if device is not None:
        read_loop(device, main_proc, main_argv)
