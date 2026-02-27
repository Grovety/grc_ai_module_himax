import logging
from abc import ABC, abstractmethod

import serial


logger = logging.getLogger(__name__)

class Transport(ABC):
    @abstractmethod
    def open(self) -> bool:
        pass

    @abstractmethod
    def close(self):
        pass

    @abstractmethod
    def write(self, data: bytes) -> bool:
        pass

    @abstractmethod
    def read(self, size: int) -> bytes:
        pass

    @abstractmethod
    def read_until(self, terminator: bytes = b'\n') -> bytes:
        pass

    @property
    @abstractmethod
    def is_open(self) -> bool:
        pass

    @abstractmethod
    def flush(self):
        pass

class SerialTransport(Transport):
    def __init__(self, port, baudrate=115200, timeout=10.0, inter_byte_timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.inter_byte_timeout = inter_byte_timeout
        self._ser = None

    def open(self) -> bool:
        try:
            self._ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
                inter_byte_timeout=self.inter_byte_timeout
            )
            logger.info(f"Serial port {self.port} opened at {self.baudrate} baud")

            return self._ser.is_open
        except Exception as e:
            logger.error(f"Serial Open Error on {self.port}: {e}")

            return False

    def close(self):
        if self._ser:
            self._ser.close()
            logger.debug(f"Serial port {self.port} closed")

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def flush(self):
        if self._ser and self._ser.is_open:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()

    def write(self, data: bytes) -> bool:
        if not self._ser:
            logger.warning("Write attempted on closed serial port")

            return False
        try:
            written = self._ser.write(data)
            if written != len(data):
                logger.warning(f"Partial write: {written}/{len(data)} bytes")
            
            return written == len(data)
        except Exception as e:
            logger.error(f"Serial write error: {e}")

            return False

    def read(self, size: int) -> bytes:
        if not self._ser:
            return b''
        
        return self._ser.read(size)

    def read_until(self, terminator: bytes = b'\n') -> bytes:
        if not self._ser: return b''
        
        return self._ser.read_until(terminator)
