import time
import random
import serial
from threading import Thread
from pynput import keyboard
from colorama import init
init()
from colorama import Fore, Back, Style

port = "COM10"
baudrate = 115200
timeout = 0.01
inter_byte_timeout = 0.01

min_int_period = 300
max_int_period = 30000

user_break = False

def ubreak():
    global user_break
    while True:
        time.sleep(0.01)
        with keyboard.Events() as events:
            for event in events:
                if event.key == keyboard.Key.space:
                    user_break = True
                    return

def write():
    global user_break
    while user_break == False:
        delay = random.randint(min_int_period, max_int_period-1)
        time.sleep(delay/1000)
        ser.write(b'\0')
        print(Fore.RED + "tof interrupt")

def read():
    global user_break
    while user_break == False:
        rd = ser.read(1)
        if rd:
            print(Fore.GREEN + rd.decode("utf-8"), end = "")

ser = serial.Serial(port, baudrate = baudrate, timeout = timeout, inter_byte_timeout = inter_byte_timeout)

user_break = False
thread_wr = Thread(target = write)
thread_rd = Thread(target = read)
thread_ub = Thread(target = ubreak)
thread_wr.start()
thread_rd.start()
thread_ub.start()
thread_rd.join()

ser.close()
