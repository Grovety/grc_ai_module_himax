import serial
import os
import sys
sys.path.append("../tests")

if os.name == "nt":
    port = "COM4"
elif os.name == "posix":
    port = "/dev/ttyACM0"
else:
    print("bad os")
    exit()

baudrate = 115200
timeout = 10
inter_byte_timeout = 10

ser = None
log = False

def init():
    global ser
    ser = serial.Serial(port, baudrate = baudrate, timeout = timeout, inter_byte_timeout = inter_byte_timeout)

def write(data):
    global ser
    written = ser.write(data)
    return written == len(data)

def read(numb):
    global ser
    ret = ser.read(numb)
    return ret

def close():
    global ser
    ser.close()

def serial_wrapper(proc, main_argv = None):
    proc(init, close, write, read, main_argv)

if __name__ == "__main__":
    if (len(sys.argv)) > 1:
        port = sys.argv[1]
    init()
    cmd = b"AT\n"
    print(">", cmd)
    write(cmd)
    ret = read(3)
    print("<", ret)
    close()
