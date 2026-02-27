import argparse
import logging
import sys

from PyQt6.QtWidgets import QApplication

from pygatebox.config import load_config

from .window import MainWindow


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument(
        "-l",
        "--log-level",
        type=str.upper,
        choices=["ERROR", "WARNING", "INFO", "DEBUG"],
        help="Override logging level for this run",
    )
    args, qt_argv = parser.parse_known_args(argv)
    return args, qt_argv


logger = logging.getLogger(__name__)


def main():
    args, qt_argv = parse_args(sys.argv[1:])
    config = load_config()
    config_level_name = str(config.get("logging", {}).get("level", "INFO")).upper()
    effective_level_name = args.log_level or config_level_name
    log_level = getattr(logging, effective_level_name, logging.INFO)

    logging.basicConfig(
        level=log_level,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )

    logger.info("Starting Gatebox Control Center...")
    app = QApplication([sys.argv[0], *qt_argv])
    window = MainWindow(log_level_override=args.log_level)
    window.show()
    logger.info("Application window shown")
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
