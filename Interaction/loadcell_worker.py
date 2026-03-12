import json
import time
import serial
from Interaction.load_cell import send_tare, parse_loadcell_line, SERIAL_PORT, BAUD, READ_TIMEOUT_S


def loadcell_worker(logger, running, tag, serial_port=SERIAL_PORT, baud=BAUD, time_out=READ_TIMEOUT_S):
    ser = serial.Serial(serial_port, baud, timeout=time_out)
    samples = []

    logger.info("Load cell thread started.")
    time.sleep(3)
    send_tare(ser, logger)
    while running:
        now = time.time()

        line = ser.readline()
        if line:
            val = parse_loadcell_line(line)
            if val is not None:
                force = val
                # logger.info(f"Load cell reading: {force:.3f}.")
                sample = {'type': 'measurement', 'name': 'loadcell', 'data': {'force': force, 'time': now}}
                samples.append(sample)

    filename = f"./logs/loadcell_{tag}.json"

    with open(filename, 'w') as f:
        json.dump(samples, f, indent=2)

    logger.info("Load cell thread exiting.")
