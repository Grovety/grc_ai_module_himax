"""
Configuration management for Gatebox application.

Handles loading and saving user preferences, window state, and feature flags.
"""

import json
import logging
from pathlib import Path
from typing import Any

from .constants import CONFIG_PATHS, FEATURE_DEFAULTS, SERIAL_DEFAULTS, TIMING_DEFAULTS, WINDOW_GEOMETRY


logger = logging.getLogger(__name__)


def get_config_path() -> Path:
    """
    Get the configuration file path.

    Returns:
        Path to config.json in the application directory.
    """
    return Path(__file__).parent / CONFIG_PATHS.FILENAME


def get_default_config() -> dict[str, Any]:
    """
    Get default configuration values.

    Returns:
        Dictionary with default configuration.
    """
    return {
        "serial": {
            "last_port": "",
            "baudrate": SERIAL_DEFAULTS.BAUDRATE,
            "timeout": TIMING_DEFAULTS.SERIAL_TIMEOUT,
            "inter_byte_timeout": TIMING_DEFAULTS.SERIAL_INTER_BYTE_TIMEOUT,
        },
        "ui": {
            "window_width": WINDOW_GEOMETRY.WIDTH,
            "window_height": WINDOW_GEOMETRY.HEIGHT,
        },
        "features": {
            "image": FEATURE_DEFAULTS.IMAGE_ENABLED,
            "history": FEATURE_DEFAULTS.HISTORY_ENABLED,
            "distance": FEATURE_DEFAULTS.DISTANCE_ENABLED,
        },
        "logging": {
            "level": "INFO",
        },
    }


def load_config() -> dict[str, Any]:
    """
    Load configuration from file.

    If config file doesn't exist or is invalid, returns default configuration.

    Returns:
        Dictionary with configuration values.
    """
    config_path = get_config_path()

    if not config_path.exists():
        logger.info(f"Config file not found, using defaults: {config_path}")
        return get_default_config()

    try:
        with open(config_path, encoding="utf-8") as f:
            config = json.load(f)
        logger.info(f"Config loaded from: {config_path}")
        return _merge_with_defaults(config)
    except json.JSONDecodeError as e:
        logger.error(f"Invalid JSON in config file: {e}")
        return get_default_config()
    except Exception as e:
        logger.error(f"Error loading config: {e}")
        return get_default_config()


def _merge_with_defaults(config: dict[str, Any]) -> dict[str, Any]:
    """
    Merge loaded config with defaults to ensure all keys exist.

    Args:
        config: Loaded configuration dictionary.

    Returns:
        Merged configuration dictionary.
    """
    defaults = get_default_config()

    # Merge serial settings
    if "serial" not in config:
        config["serial"] = defaults["serial"]
    else:
        for key, value in defaults["serial"].items():
            if key not in config["serial"]:
                config["serial"][key] = value

    # Merge UI settings
    if "ui" not in config:
        config["ui"] = defaults["ui"]
    else:
        for key, value in defaults["ui"].items():
            if key not in config["ui"]:
                config["ui"][key] = value

    # Merge features
    if "features" not in config:
        config["features"] = defaults["features"]
    else:
        for key, value in defaults["features"].items():
            if key not in config["features"]:
                config["features"][key] = value

    # Merge logging
    if "logging" not in config:
        config["logging"] = defaults["logging"]
    else:
        for key, value in defaults["logging"].items():
            if key not in config["logging"]:
                config["logging"][key] = value

    return config


def save_config(config: dict[str, Any]) -> bool:
    """
    Save configuration to file.

    Args:
        config: Configuration dictionary to save.

    Returns:
        True if save was successful, False otherwise.
    """
    config_path = get_config_path()

    try:
        with open(config_path, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
        logger.info(f"Config saved to: {config_path}")
        return True
    except Exception as e:
        logger.error(f"Error saving config: {e}")
        return False
