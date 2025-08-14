import sys
sys.path.append("./interfaces")
sys.path.append("./tests")

from serial_wrapper import serial_wrapper
from tof_control import tof_control

serial_wrapper(tof_control, sys.argv[1:])
