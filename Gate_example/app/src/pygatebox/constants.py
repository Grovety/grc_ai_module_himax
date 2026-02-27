"""
UI Constants and Configuration for Gatebox Control Center.

This module contains all UI-related constants to avoid magic numbers
and provide centralized configuration.
"""

from typing import NamedTuple


class WindowGeometry(NamedTuple):
    """Default window dimensions."""

    WIDTH: int = 1100
    HEIGHT: int = 750
    MIN_WIDTH: int = 800
    MIN_HEIGHT: int = 600


class VideoDisplay(NamedTuple):
    """Video display settings."""

    MIN_WIDTH: int = 480
    MIN_HEIGHT: int = 360
    BACKGROUND_COLOR: str = "black"
    TEXT_COLOR: str = "white"
    FONT_SIZE: int = 20


class DistanceLabel(NamedTuple):
    """Distance label styling."""

    FONT_SIZE: int = 18
    FONT_WEIGHT: str = "bold"
    COLOR_DEFAULT: str = "gray"
    COLOR_SUCCESS: str = "green"
    COLOR_ERROR: str = "red"
    BORDER_WIDTH_DEFAULT: int = 1
    BORDER_WIDTH_SUCCESS: int = 2
    BORDER_WIDTH_ERROR: int = 1


class LayoutStretch(NamedTuple):
    """
    Layout stretch factors for QBoxLayout.

    These values determine how space is distributed among widgets.
    Higher values = more space.
    """

    LEFT_PANEL: int = 55
    RIGHT_PANEL: int = 45

    class RightPanel(NamedTuple):
        HISTORY: int = 35
        PERMITTED: int = 30
        MASKS: int = 25
        SETTINGS: int = 10

    right: RightPanel = RightPanel()


class ButtonStyles(NamedTuple):
    """Button CSS styles."""

    CONNECT_ACTIVE: str = "background-color: #ffcccc"
    CONNECT_INACTIVE: str = ""


class ConfigPaths(NamedTuple):
    """Configuration file paths."""

    FILENAME: str = "config.json"


# Default timing constants (seconds)
class TimingDefaults(NamedTuple):
    """Default timing values for device communication."""

    SERIAL_TIMEOUT: float = 10.0
    SERIAL_INTER_BYTE_TIMEOUT: float = 0.1
    CONNECT_DELAY: float = 0.5
    WORKER_POLL_INTERVAL: float = 0.005
    WORKER_IDLE_INTERVAL: float = 0.1
    HISTORY_INTERVAL: float = 1.0
    IMAGE_CAPTURE_TIMEOUT: float = 0.3
    IMAGE_CHUNK_TIMEOUT: float = 0.2
    IMAGE_FIRST_RETRIES: int = 20
    IMAGE_MAX_CHUNKS: int = 200
    COMMAND_RESPONSE_RETRIES: int = 5


# Feature defaults
class FeatureDefaults(NamedTuple):
    """Default feature states."""

    IMAGE_ENABLED: bool = True
    HISTORY_ENABLED: bool = True
    DISTANCE_ENABLED: bool = True


# Serial defaults
class SerialDefaults(NamedTuple):
    """Default serial port settings."""

    BAUDRATE: int = 115200


class InputValidation(NamedTuple):
    """Input validation limits for user-entered data."""

    PLATE_MAX_LEN: int = 67
    MASK_MAX_LEN: int = 32
    FORBIDDEN_COMMON: str = "\r\n;"
    FORBIDDEN_PLATE_EXTRA: str = ","


# Instantiate default configurations
WINDOW_GEOMETRY = WindowGeometry()
VIDEO_DISPLAY = VideoDisplay()
DISTANCE_LABEL = DistanceLabel()
LAYOUT_STRETCH = LayoutStretch()
BUTTON_STYLES = ButtonStyles()
CONFIG_PATHS = ConfigPaths()
TIMING_DEFAULTS = TimingDefaults()
FEATURE_DEFAULTS = FeatureDefaults()
SERIAL_DEFAULTS = SerialDefaults()
INPUT_VALIDATION = InputValidation()
