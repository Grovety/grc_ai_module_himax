import logging
import queue
import time

from PyQt6.QtCore import QThread, pyqtSignal

from pygatebox.constants import TIMING_DEFAULTS
from pygatebox.core.device import GateboxDevice


logger = logging.getLogger(__name__)


class ImageWorker(QThread):
    image_received = pyqtSignal(bytes)
    distance_ready = pyqtSignal(int, int)
    history_ready = pyqtSignal(list)
    permitted_list_ready = pyqtSignal(list)
    masks_list_ready = pyqtSignal(list)
    connection_result = pyqtSignal(bool, str)
    status_msg = pyqtSignal(str)
    connection_lost = pyqtSignal()  # Signal for connection loss

    def __init__(self, device: GateboxDevice):
        super().__init__()
        self.device = device
        self.command_queue = queue.Queue()
        self.running = True

        # State
        self.STATE_IMAGE = 0
        self.STATE_DISTANCE = 1
        self.STATE_HISTORY = 2
        self.current_data_state = self.STATE_IMAGE

        # Feature flags
        self.enable_image = True
        self.enable_distance = True
        self.enable_history = True

        # Timer
        self.last_history_time = 0
        self.history_interval = TIMING_DEFAULTS.HISTORY_INTERVAL

    def queue_command(self, cmd_data):
        self.command_queue.put(cmd_data)
        logger.debug(f"Command queued: {cmd_data.get('action')}")

    def run(self):
        while self.running:
            try:
                while not self.command_queue.empty():
                    cmd = self.command_queue.get_nowait()
                    self.process_command(cmd)
            except queue.Empty:
                pass

            try:
                if self.device.transport.is_open:
                    self.process_data_loop()
                    time.sleep(TIMING_DEFAULTS.WORKER_POLL_INTERVAL)
                else:
                    time.sleep(TIMING_DEFAULTS.WORKER_IDLE_INTERVAL)
            except Exception as e:
                logger.error(f"Worker run error: {e}")
                
                if "Serial" in str(type(e).__name__) or "serial" in str(e).lower():
                    logger.warning("Serial port error detected, closing connection")
                    self.device.disconnect()
                    self.connection_lost.emit()
                    self.status_msg.emit("Connection lost")
                time.sleep(TIMING_DEFAULTS.WORKER_IDLE_INTERVAL)

    def process_command(self, cmd):
        action = cmd.get("action")
        logger.debug(f"Processing command: {action}")

        if action == "toggle_feature":
            feature = cmd.get("feature")
            state = cmd.get("state")

            if feature == "image":
                self.enable_image = state
            
            elif feature == "distance":
                self.enable_distance = state
            
            elif feature == "history":
                self.enable_history = state
            
            logger.info(f"Feature '{feature}' toggled: {state}")
            return

        if action == "connect":
            port = cmd.get("port")
            self.status_msg.emit(f"Connecting to {port}...")
            self.device.transport.port = port

            if self.device.connect():
                try:
                    self.device.init_sequence()
                    self.device.sync_time()
                    self.connection_result.emit(True, "Connected")
                    logger.info(f"Device connected and initialized on {port}")
                except Exception as e:
                    self.device.disconnect()
                    logger.error(f"Init error on {port}: {e}")
                    self.connection_result.emit(False, f"Init Error: {e}")
            else:
                logger.error(f"Failed to open port {port}")
                self.connection_result.emit(False, "Failed to open port")
        
        elif action == "disconnect":
            self.device.disconnect()
            self.connection_result.emit(False, "Disconnected")
            logger.info("Device disconnected")

        if not self.device.transport.is_open:
            return

        if action == "clear_history":
            if self.device.clear_history():
                self.status_msg.emit("History cleared")
                self.history_ready.emit([])
                logger.info("History cleared")
        
        elif action == "read_permitted":
            lst = self.device.get_permitted_list()
            self.permitted_list_ready.emit(lst)
            logger.debug(f"Read {len(lst)} permitted plates")
        
        elif action == "add_plate":
            plate = cmd.get("plate")
            added = self.device.add_permitted_plate(plate)
            if added:
                self.status_msg.emit(f"Added: {plate}")
                logger.info(f"Permitted plate added: {plate}")
            else:
                self.status_msg.emit(f"Add failed: {plate}")
                logger.warning(f"Failed to add permitted plate: {plate}")

            lst = []
            # Device may need a short delay before list reads reflect the new item.
            for _ in range(3):
                lst = self.device.get_permitted_list()
                if plate in lst:
                    break
                time.sleep(0.05)
            self.permitted_list_ready.emit(lst)
        
        elif action == "clear_permitted":
            cleared = self.device.clear_permitted_list()
            if cleared:
                self.status_msg.emit("Permitted list cleared")
                logger.info("Permitted list cleared")
            else:
                self.status_msg.emit("Clear permitted list failed")
                logger.warning("Failed to clear permitted list")

            lst = []
            for i in range(3):
                lst = self.device.get_permitted_list()
                if (len(lst) == 0) and (i >= 1):
                    break
                time.sleep(0.05)
            self.permitted_list_ready.emit(lst)
        
        elif action == "delete_plate":
            plate = cmd.get("plate")
            removed = plate and self.device.delete_permitted_plate(plate)
            if removed:
                self.status_msg.emit(f"Removed: {plate}")
                logger.info(f"Permitted plate removed: {plate}")
            elif plate:
                self.status_msg.emit(f"Remove failed: {plate}")
                logger.warning(f"Failed to remove permitted plate: {plate}")

            if plate:
                lst = []
                for i in range(3):
                    lst = self.device.get_permitted_list()
                    # Require at least two reads to avoid transient post-command responses.
                    if (plate not in lst) and (i >= 1):
                        break
                    time.sleep(0.05)
                self.permitted_list_ready.emit(lst)
        
        elif action == "set_gate":
            val = cmd.get("value")
            if self.device.set_gate_interval(val):
                self.status_msg.emit(f"Gate interval set: {val}")
                logger.info(f"Gate interval set to {val}s")
        
        elif action == "read_masks":
            lst = self.device.get_validation_masks()
            self.masks_list_ready.emit(lst)
            logger.debug(f"Read {len(lst)} validation masks")
        
        elif action == "add_mask":
            mask = cmd.get("mask")
            if self.device.add_validation_mask(mask):
                self.status_msg.emit(f"Mask added: {mask}")
                lst = self.device.get_validation_masks()
                self.masks_list_ready.emit(lst)
            else:
                self.status_msg.emit(f"Mask add failed: {mask}")
                logger.warning(f"Failed to add validation mask: {mask}")
        
        elif action == "clear_masks":
            if self.device.clear_validation_masks():
                self.status_msg.emit("Masks cleared")
                self.masks_list_ready.emit([])
            else:
                self.status_msg.emit("Masks clear failed")
                logger.warning("Failed to clear validation masks")

        elif action == "delete_mask":
            mask = cmd.get("mask")
            if not mask:
                return

            masks = self.device.get_validation_masks()
            if mask not in masks:
                self.status_msg.emit(f"Mask not found: {mask}")
                self.masks_list_ready.emit(masks)
                return

            removed_once = False
            new_masks = []
            for item in masks:
                if (not removed_once) and item == mask:
                    removed_once = True
                    continue
                new_masks.append(item)

            ok = self.device.clear_validation_masks()
            if ok:
                for item in new_masks:
                    if not self.device.add_validation_mask(item):
                        ok = False
                        break

            if ok:
                self.status_msg.emit(f"Mask removed: {mask}")
                logger.info(f"Validation mask removed: {mask}")
            else:
                self.status_msg.emit(f"Mask remove failed: {mask}")
                logger.warning(f"Failed to remove validation mask: {mask}")

            self.masks_list_ready.emit(self.device.get_validation_masks())

    def process_data_loop(self):
        """State machine with feature-flag checks."""

        # Exit early if all features are disabled or the port is closed
        if not (self.enable_image or self.enable_distance or self.enable_history):
            return
        
        if not self.device.transport.is_open:
            return

        try:
            if self.current_data_state == self.STATE_IMAGE:
                # Image (if enabled)
                if self.enable_image:
                    if self.device.capture_image():
                        img_data = self.device.get_complete_image(
                            check_cancelled=lambda: not self.running
                        )
                        if img_data:
                            self.image_received.emit(img_data)
                            logger.debug(f"Image received: {len(img_data)} bytes")

                self.current_data_state = self.STATE_DISTANCE
                # Process pending commands between states
                self._process_pending_commands()

            elif self.current_data_state == self.STATE_DISTANCE:
                # Distance (if enabled)
                if self.enable_distance:
                    status, dist = self.device.get_distance()
                    if status != -1:
                        self.distance_ready.emit(status, dist)

                self.current_data_state = self.STATE_HISTORY
                # Process pending commands between states
                self._process_pending_commands()

            elif self.current_data_state == self.STATE_HISTORY:
                # History (if enabled and interval elapsed)
                now = time.time()
                if self.enable_history:
                    if now - self.last_history_time > self.history_interval:
                        try:
                            hist_list = self.device.get_history_list()
                            if hist_list:
                                self.history_ready.emit(hist_list)
                                logger.debug(f"History updated: {len(hist_list)} items")
                            self.last_history_time = now
                        except Exception as e:
                            logger.error(f"History read error: {e}")

                self.current_data_state = self.STATE_IMAGE
                # Process pending commands between states
                self._process_pending_commands()
        except Exception as e:
            logger.error(f"process_data_loop error: {e}")
            # Check if it's a serial port error
            if "Serial" in str(type(e).__name__) or "serial" in str(e).lower():
                logger.warning("Serial port error in process_data_loop, closing connection")
                self.device.disconnect()
                self.connection_lost.emit()
                self.status_msg.emit("Connection lost")
            # Reset state to prevent stuck
            self.current_data_state = self.STATE_IMAGE

    def _process_pending_commands(self):
        """Process any pending commands without blocking."""
        try:
            while not self.command_queue.empty():
                cmd = self.command_queue.get_nowait()
                self.process_command(cmd)
        except queue.Empty:
            pass

    def stop(self):
        self.running = False
        logger.info("Worker stopping...")
        self.wait()
