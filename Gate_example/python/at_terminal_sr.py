import sys
sys.path.append("./interfaces")
sys.path.append("./tests")

from serial_wrapper import serial_wrapper
from at_terminal import at_terminal

serial_wrapper(at_terminal, sys.argv[1:])