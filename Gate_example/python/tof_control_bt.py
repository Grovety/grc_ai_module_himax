import sys
sys.path.append("./interfaces")
sys.path.append("./tests")

from bluetooth_wrapper import bluetooth_wrapper
from tof_control import tof_control

bluetooth_wrapper(tof_control, sys.argv[1:])
