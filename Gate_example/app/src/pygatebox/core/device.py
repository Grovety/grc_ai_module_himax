import logging
import threading
import time
from datetime import datetime

from ..constants import TIMING_DEFAULTS
from .definitions import AtCmd, ResponseStatus
from .transport import Transport


logger = logging.getLogger(__name__)

class GateboxDevice:
    """High-level interface for controlling the Gatebox device via AT commands."""
    def __init__(self, transport: Transport):
        self.transport = transport
        self._lock = threading.RLock()

    def connect(self) -> bool:
        with self._lock:
            if self.transport.open():
                time.sleep(TIMING_DEFAULTS.CONNECT_DELAY)
                self.transport.flush()
                logger.info("Device connected successfully")
                return True
            logger.warning("Device connection failed")
            return False

    def disconnect(self):
        with self._lock:
            self.transport.close()
            logger.info("Device disconnected")

    def _send_cmd(self, cmd: str, log: bool = True) -> bool:
        full_cmd = f"{cmd}\n".encode()
        if log:
            logger.debug(f"> {full_cmd.strip().decode('utf-8', errors='ignore')}")
        self.transport.flush()

        return self.transport.write(full_cmd)

    def _read_line(self, log: bool = True) -> str | None:
        raw = self.transport.read_until(b'\n')
        if not raw:
            return None

        decoded = raw.decode('utf-8', errors='ignore').strip()
        if log and decoded:
            logger.debug(f"< {decoded}")
        
        return decoded

    def _drain_buffer(self):
        """Reads all pending data from buffer."""
        while True:
            chunk = self.transport.read(1024)
            if not chunk or len(chunk) < 1024:
                break

    def send_command_expect_ok(self, cmd: str) -> bool:
        with self._lock:
            if not self._send_cmd(cmd): return False
            for _ in range(5):
                line = self._read_line()
                if line is None: continue
                if "OK" in line: return True
                if "ERROR" in line: return False
            
            return False

    def init_sequence(self):
        logger.info("Device: Init sequence started...")
        self.transport.flush()
        self._send_cmd(AtCmd.CHECK.value)
        self._read_line()

        if self.send_command_expect_ok(AtCmd.MODE_WORK.value):
            logger.info("Device: Mode set to WORK")
        
        if self.send_command_expect_ok(AtCmd.HIST_CLEAR.value):
            logger.info("Device: History cleared")

    def capture_image(self) -> bool:
        with self._lock:
            if not self._send_cmd(AtCmd.IMG_CAPTURE.value):
                return False
            for _ in range(15):
                line = self._read_line()

                if line is None:
                    continue

                if "OK" in line:
                    return True
                
                if "ERROR" in line:
                    logger.warning(f"Image capture error: {line}")
                    return False
            
            logger.warning("Image capture timeout")
            return False

    def get_complete_image(self, check_cancelled=None) -> bytes | None:
        """
        Download complete image from device.
        
        Args:
            check_cancelled: Optional callable that returns True if operation should be cancelled.
                            Called periodically during download.
        """
        chunk_data = None

        with self._lock:
            # 1. First chunk
            for i in range(TIMING_DEFAULTS.IMAGE_FIRST_RETRIES):
                # Check for cancellation
                if check_cancelled and check_cancelled():
                    logger.info("Image download cancelled")
                    return None
                    
                if i > 0 and i % 5 == 0:
                    logger.debug(f"Waiting for image... attempt {i}")

                self._send_cmd(AtCmd.IMG_READ_FIRST.value, log=False)
                chunk_data, is_last = self._read_chunk_optimized(
                    timeout=TIMING_DEFAULTS.IMAGE_CAPTURE_TIMEOUT
                )

                if chunk_data == b'NRDY':
                    time.sleep(0.05)
                    continue

                if chunk_data is None:
                    self._drain_buffer()
                    continue
                break

            if chunk_data == b'NRDY' or chunk_data is None:
                logger.error("Device: Image fetch timeout")
                return None

            image_buffer = bytearray(chunk_data)
            if is_last:
                logger.info(f"Device: Frame downloaded ({len(image_buffer)} bytes)")
                return bytes(image_buffer)

            # 2. Next chunks
            cnt = 0
            while cnt < TIMING_DEFAULTS.IMAGE_MAX_CHUNKS:
                # Check for cancellation
                if check_cancelled and check_cancelled():
                    logger.info("Image download cancelled (mid-stream)")
                    return None
                    
                self._send_cmd(AtCmd.IMG_READ_NEXT.value, log=False)
                chunk_data, is_last = self._read_chunk_optimized(
                    timeout=TIMING_DEFAULTS.IMAGE_CHUNK_TIMEOUT
                )

                if chunk_data is None or chunk_data == b'NRDY':
                    logger.error("Device: Stream broken (next chunk timeout)")
                    self._drain_buffer()
                    return None

                image_buffer.extend(chunk_data)
                if is_last:
                    break
                cnt += 1

            logger.info(f"Device: Frame downloaded ({len(image_buffer)} bytes)")
            return bytes(image_buffer)

    def _read_chunk_optimized(self, timeout: float = 0.5) -> tuple[bytes | None, bool]:
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            try:
                byte = self.transport.read(1)
            except Exception as e:
                logger.error(f"Serial read error in _read_chunk_optimized: {e}")
                return None, False
            
            if not byte:
                continue

            char = byte[0]
            if char in [ord('\r'), ord('\n'), 0]: continue

            if char == ord('A'): # Echo 'A'T...
                self.transport.read_until(b'\n')
                continue

            if char == ord('N'): # NRDY
                rest = self.transport.read(3)
                if rest == b'RDY':
                    self.transport.read_until(b'\n')
                    return b'NRDY', False
                continue

            if char in [ord('#'), ord('!')]:
                marker = char
                size_bytes = self.transport.read(3)
                if len(size_bytes) < 3: return None, False

                try:
                    size = int(size_bytes)
                except ValueError:
                    return None, False

                data = self.transport.read(size)
                if len(data) != size: return None, False

                return data, (marker == ord('!'))

        return None, False

    # --- Distance ---
    def get_distance(self) -> tuple[int, int]:
        with self._lock:
            if self._send_cmd(AtCmd.TOF_DIST.value):
                resp = self._read_response_ignore_echo()
                try:
                    if resp and ',' in resp:
                        parts = resp.split(',')
                        return int(parts[0]), int(parts[1])
                except ValueError:
                    pass
        
        return -1, 0

    # --- History ---
    def get_history_list(self) -> list[str]:
        history_items = []
        MAX_PAGES = 10
        page = 0
        with self._lock:
            if not self._send_cmd("AT+LRREAD=7"):
                return []

            while page < MAX_PAGES:
                resp = self._read_response_ignore_echo()

                if resp is None: break
                if "ERROR" in resp: break
                if resp == "": break
                if len(resp) > 300 and ';' not in resp: break

                parts = resp.split(';')
                for p in parts:
                    if p.strip():
                        history_items.append(p.strip())

                if not self._send_cmd("AT+LRRDNEXT=7"):
                    break

                page += 1
        
        return history_items

    # --- Permitted ---
    def get_permitted_list(self) -> list[str]:
        plates = []
        max_pages = 10
        page = 0
        stop_responses = {
            "",
            "ERROR",
            "NRDY",
            "BAD_CRC",
            "BAD_FORMAT",
            "BAD_PARAM",
            "NOT_FOUND",
            "OTHER_ERROR",
            "BUFFER_TOO_SMALL",
        }

        with self._lock:
            if not self._send_cmd(AtCmd.LIST_READ_FIRST.value):
                return []
            
            while page < max_pages:
                resp = self._read_response_ignore_echo()

                if resp is None:
                    break

                normalized = resp.strip().upper()
                if normalized in stop_responses or normalized.startswith("BAD_"):
                    break

                items = resp.split(',')
                for item in items:
                    val = item.strip()
                    if not val:
                        continue
                    # Firmware may transiently return command status tokens in list reads.
                    if val.upper() == ResponseStatus.OK.value:
                        continue
                    plates.append(val)

                if not self._send_cmd(AtCmd.LIST_READ_NEXT.value):
                    break

                page += 1
        
        return plates

    # --- Validation Masks ---
    def get_validation_masks(self) -> list[str]:
        masks = []
        with self._lock:
            if self._send_cmd("AT%VALMASK!"):
                resp = self._read_response_ignore_echo()
                if resp and "ERROR" not in resp:
                    if "EMPTY" in resp: return []
                    parts = resp.split(';')
                    for p in parts:
                        if p.strip(): masks.append(p.strip())
        
        return masks

    def add_validation_mask(self, mask: str) -> bool:
        with self._lock:
            if self._send_cmd(f"AT%VALMASK=ADD,{mask}"):
                resp = self._read_response_ignore_echo()
                return resp and "OK_ADDED" in resp
        
        return False

    def clear_validation_masks(self) -> bool:
        with self._lock:
            if self._send_cmd("AT%VALMASK=CLEAR"):
                resp = self._read_response_ignore_echo()
                return resp and "OK_CLEARED" in resp
        
        return False

    def clear_history(self) -> bool:
        with self._lock:
            if self._send_cmd("AT+LRCLEAR"): return self._expect_ok()
        
        return False

    def sync_time(self) -> bool:
        now = datetime.now()
        time_str = now.strftime("%Y-%m-%d %H:%M:%S")
        with self._lock:
            if self._send_cmd(f"AT%TIME={time_str}"): return self._expect_ok()
        
        return False

    def get_gate_interval(self) -> int:
        with self._lock:
            if self._send_cmd(AtCmd.GATE_INTERVAL_GET.value):
                resp = self._read_response_ignore_echo()
                if resp and resp.isdigit(): return int(resp)
        
        return -1

    def set_gate_interval(self, seconds: int) -> bool:
        with self._lock:
            if self._send_cmd(f"{AtCmd.GATE_INTERVAL_SET.value}={seconds}"): return self._expect_ok()
        
        return False

    def add_permitted_plate(self, plate: str) -> bool:
        with self._lock:
            if self._send_cmd(f"{AtCmd.LIST_ADD.value}={plate}"): return self._expect_ok()
        
        return False

    def delete_permitted_plate(self, plate: str) -> bool:
        with self._lock:
            if self._send_cmd(f"{AtCmd.LIST_DEL.value}={plate}"): return self._expect_ok()

        return False

    def clear_permitted_list(self) -> bool:
        with self._lock:
            if self._send_cmd(AtCmd.LIST_CLEAR.value): return self._expect_ok()
        
        return False

    def _read_response_ignore_echo(self) -> str | None:
        for _ in range(6):
            line = self._read_line()
            if line is None:
                return None
            if line == "":
                continue
            if line.startswith("AT"):
                continue
            return line
        
        return None

    def _expect_ok(self) -> bool:
        resp = self._read_response_ignore_echo()
        
        return resp == ResponseStatus.OK.value
