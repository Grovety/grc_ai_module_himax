import sys
sys.path.append("./interfaces")
sys.path.append("./tests")

from bluetooth_wrapper import bluetooth_wrapper
from at_terminal import at_terminal

bluetooth_wrapper(at_terminal, sys.argv[1:])