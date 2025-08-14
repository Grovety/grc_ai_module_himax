import sys
sys.path.append("./interfaces")
sys.path.append("./tests")

from serial_wrapper import serial_wrapper
from image_reader import image_reader

serial_wrapper(image_reader, sys.argv[1:])
