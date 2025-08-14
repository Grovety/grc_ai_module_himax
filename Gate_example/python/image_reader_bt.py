import os
import ctypes
import sys
sys.path.append("./interfaces")
sys.path.append("./tests")

from bluetooth_wrapper import bluetooth_wrapper
from image_reader import image_reader

#if os.name == 'nt' and len(sys.argv) == 1:
#    ctypes.windll.user32.MessageBoxW(0, "You have to be sure,\nthat device is unpared\nin device manager!", "Check it!",  0)

bluetooth_wrapper(image_reader, sys.argv[1:])
