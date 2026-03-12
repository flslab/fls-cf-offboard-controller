import os
import threading
import time
import json
import serial

import logging
from logger import setup_logging

# ---------- user-configurable ----------
SERIAL_PORT = "/dev/cu.usbmodem1201"  # e.g. "COM5" on Windows, "/dev/ttyACM0"
BAUD = 9600
READ_TIMEOUT_S = 0.05

def parse_loadcell_line(line: bytes):
    try:
        s = line.decode(errors="ignore").strip()
        if not s:
            return None
        if s.startswith("OK:"):
            return None
        tokens = [t for t in s.replace(",", " ").split()
                  if any(c.isdigit() or c in ".-+" for c in t)]
        if not tokens:
            return None
        return float(tokens[-1])
    except Exception:
        return None


def send_tare(ser: serial.Serial, logger=None):
    try:
        # Clear any buffered noise before issuing tare
        ser.reset_input_buffer()
        ser.write(b"tare\n")
        ser.flush()
        # Read a couple lines quickly for acknowledgement
        t0 = time.time()
        while time.time() - t0 < 0.4:  # ~400 ms window for "OK: tare done"
            line = ser.readline()
            if line:
                txt = line.decode(errors="ignore").strip()
                if txt.startswith("OK:") and logger is not None:
                    logger.info(f"[HX711] {txt}")
                    break
        # After tare, give the scale a brief moment to settle
        time.sleep(0.2)
    except Exception as e:
        if logger is not None:
            logger.warning(f"[HX711] tare send failed: {e}")


def safe_load_json_list(filename):
    if not os.path.exists(filename) or os.path.getsize(filename) == 0:
        return []

    # Open in read+write mode to fix it in place if needed
    with open(filename, 'rb+') as f:
        # 1. Read the end of the file to check for the closing bracket
        f.seek(0, os.SEEK_END)
        size = f.tell()

        # Backtrack to find the last non-whitespace character
        last_char = b""
        cursor = size - 1
        while cursor >= 0:
            f.seek(cursor)
            char = f.read(1)
            if not char.isspace():
                last_char = char
                break
            cursor -= 1

        # 2. If it doesn't end with ']', we need to fix it
        if last_char != b']':

            # If the last character is a comma, we must remove it
            if last_char == b',':
                f.seek(cursor)
                f.truncate()  # Remove the comma

            # Now safely append the closing bracket
            f.write(b']')
            f.flush()

        # 3. Now load the (hopefully) fixed file
        f.seek(0)
        try:
            return json.load(f)
        except json.JSONDecodeError as e:
            return []


def loadcell_worker(stop_event, logger, filename, ser):
    samples = []

    logger.info("Load cell thread started.")
    send_tare(ser, logger)
    time.sleep(0.1)
    while not stop_event.is_set():
        now = time.time()

        line = ser.readline()
        if line:
            val = parse_loadcell_line(line)
            if val is not None:
                force = val
                logger.info(f"Load cell reading: {force:.3f}.")
                sample = {'type': 'measurement', 'name': 'loadcell', 'data': {'force': force, 'time': now}}
                samples.append(sample)

        # time.sleep(0.001)

    # Process results even if terminated early
    if not samples:
        logger.info("Load cell reading is empty.")
        return

    if filename:
        existing_data = safe_load_json_list(filename)
        existing_data.append(samples)
        logger.info(f"Saving session to {filename}...")
        with open(filename, 'w') as f:
            json.dump(existing_data, f, indent=2)

    logger.info("Load cell thread exiting.")

def loadcell_correct(filename):

    existing_data = safe_load_json_list(filename)
    for i, item in enumerate(existing_data):
        existing_data[i]['data']['force'] = existing_data[i]['data']['force'] * 1.2

        # existing_data[i]['data']['time'] = existing_data[i]['data']['time'] -0.5

    with open(filename, 'w') as f:
        json.dump(existing_data, f, indent=2)


if __name__ == '__main__':
    setup_logging()
    stop_signal = threading.Event()

    t = threading.Thread(
        target=loadcell_worker,
        args=(stop_signal, logging.getLogger(__name__), None, serial.Serial(SERIAL_PORT, BAUD, timeout=READ_TIMEOUT_S))
    )
    t.start()
    time.sleep(50)
    t.join()
