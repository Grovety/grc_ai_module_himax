# Gatebox Control Center (Python App)

Desktop GUI application for controlling a Gatebox device over serial (PyQt6).

## Features

- Connect/disconnect to a selected serial port
- Live image updates
- Distance reading
- Recognition history view
- Manage permitted plate list:
  - add plate
  - delete selected plate
  - clear list
  - refresh list
- Manage validation masks:
  - add mask
  - delete selected mask
  - clear masks
  - refresh masks
- Gate interval setting
- Runtime feature toggles (image/history/distance updates)
- Config persistence (`config.json`)
- Configurable logging level in UI and via CLI

## Requirements

- Python 3.12+
- A connected Gatebox device (serial)

Dependencies are defined in `pyproject.toml`.

## Install

Using `uv` (recommended):

```bash
uv sync
```

Or with `pip`:

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -e .
```

## Run

From `gatebox_out/app`:

```bash
uv run app
```

You can override log level for a run:

```bash
uv run app --log-level DEBUG
```

Supported levels: `ERROR`, `WARNING`, `INFO`, `DEBUG`.

## Validation Mask Syntax

- `#` digit
- `L` letter
- `X` letter or digit
- `C` any single character
- `?` any single character
- `P` token in form `<...>`
- spaces in mask are ignored

Example mask:

```text
L # # <Region>
```

## Notes

- On disconnect, runtime lists in UI (history, permitted plates, masks) are cleared.
- Input validation is applied in the app before sending data to the device.
- Effective limits in the app:
  - plate max length: `67`
  - mask max length: `32`
