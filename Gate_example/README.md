# Gatebox Firmware Installation Guide (nRF + Himax)

This document describes a practical firmware update flow for the board in this package.

## Package Layout

```text
Gate_example/
├── README.md
├── app/
│   ├── README.md
│   ├── pyproject.toml
│   └── src/
├── nrf/
│   ├── flash_nrf.ps1
│   ├── flash_nrf.sh
│   └── bin/
│       └── zephyr.signed.bin
└── himax/
    ├── HimaxFlasher.exe
    ├── config.yaml
    └── images/
        ├── firmware.img
        ├── lpd/
        └── lpr/
```

Key files:

- `nrf/flash_nrf.ps1` - nRF flashing script for Windows (mcumgr-based).
- `nrf/bin/zephyr.signed.bin` - nRF firmware image.
- `himax/HimaxFlasher.exe` - Himax flashing utility.
- `himax/config.yaml` - Himax flashing plan (firmware/models and addresses).
- `himax/images/` - Himax firmware and model artifacts.
- `app/README.md` - desktop app usage and command reference.

## Before You Start

- Disconnect/close all software that may use the COM port (terminal, app, etc.).
- Use a stable USB-C cable.
- Confirm the board appears in Windows Device Manager under `Ports (COM & LPT)`.
- Install `mcumgr` (required for nRF flashing script).

## 1. Flash nRF Firmware

### Enter DFU mode

1. Unplug the board from USB.
2. Press and hold `SW2`.
3. Plug USB-C cable in while still holding `SW2`.
4. Release `SW2` after a few seconds.

### Find COM port

Open `Device Manager -> Ports (COM & LPT)` and note the board COM port (for example `COM5`).

### Run flashing script

Run from `nrf`:

```powershell
.\flash_nrf.ps1 <COMx>
```

Expected success line:

```text
nRF Flashing Complete.
```

If flashing/upload does not start, unplug and reconnect the cable with the connector flipped to the opposite side/orientation, then run the script again.

## 2. Flash Himax Firmware and Models

### Step 1 - Connect board for Himax

1. Unplug the board from USB.
2. Press and hold `SW2`.
3. Insert the USB-C cable with the opposite side/orientation (flip the connector) while still holding `SW2`.
4. Release `SW2` after a few seconds.

### Step 2 - Find Himax COM port

Open `Device Manager -> Ports (COM & LPT)` and note the Himax COM port (for example `COM6`).  
Do not reuse nRF port value blindly; verify it each time.

### Step 3 - Run Himax flasher

Run from `himax`:

```powershell
.\HimaxFlasher.exe --port <COMx>
```

В начале скрипт запросит для какой страны загружать модели.
После запросит нажать кнопку SW1, это нужно проделать один раз.

Expected success line:

```text
== Success! ==
```

### Step 4 - If Himax flashing does not start

1. Unplug the board from USB.
2. Press and hold `SW2`.
3. Reconnect cable with the connector flipped to the opposite side/orientation while still holding `SW2`.
4. Release `SW2` after a few seconds.

## 3. Optional: Run Desktop App

After flashing is complete, you can connect to the device with the desktop app.

Run from `app`:

```powershell
uv run app
```

With explicit log level:

```powershell
uv run app --log-level INFO
```

## Quick Command Summary (Windows)

```powershell
# nRF
cd nrf
.\flash_nrf.ps1 <COMx>

# Himax
cd himax
.\HimaxFlasher.exe --port <COMx>

# App (optional)
cd app
uv run app --log-level INFO
```