import csv
import os
import serial  # pip install pyserial

from Interaction.flight_behaviors import *

# ---------- user-configurable ----------
SERIAL_PORT = "/dev/cu.usbmodem1201"  # e.g. "COM5" on Windows, "/dev/ttyACM0"
BAUD = 9600
READ_TIMEOUT_S = 0.05

STABILIZE_WAIT_S = 3.0  # wait after starting each thrust before collecting
COLLECT_WINDOW_S = 3.0  # seconds of data to collect per step
DURATION_PER_STEP_S = STABILIZE_WAIT_S + COLLECT_WINDOW_S + 1.0  # padding

PCT_START = 10
PCT_END = 100
PCT_STEP = 10

LOG_FILE = "HQ_thrust.csv"
RAW_DIR = None  # set to None to disable raw per-step dumps


# --------------------------------------

def thrust_from_percent(pct: float) -> int:
    """Your mapping: 1% -> ~10001 + 0.01*pct*(60000-10001), 100% -> 60000."""
    return int(10001 + 0.01 * pct * (60000 - 10001))


def parse_loadcell_line(line: bytes):
    """Extract the last numeric token from a serial line, return float or None."""
    try:
        s = line.decode(errors="ignore").strip()
        if not s:
            return None
        # Accept plain numbers or lines like "OK: tare done" (ignored)
        # If it's "OK: ..." return None
        if s.startswith("OK:"):
            return None
        # Extract numeric token
        tokens = [t for t in s.replace(",", " ").split()
                  if any(c.isdigit() or c in ".-+" for c in t)]
        if not tokens:
            return None
        return float(tokens[-1])
    except Exception:
        return None


def send_tare(ser: serial.Serial, logger=None):
    """Send 'tare' command and try to read ack without blocking too long."""
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


def sweep_thrust_with_loadcell(cf, state_que, start, end, step, logger):
    """For each thrust %, tare the scale, stream setpoints, and collect load-cell + CF telemetry."""
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=READ_TIMEOUT_S)
    if RAW_DIR:
        os.makedirs(RAW_DIR, exist_ok=True)

    new_file = not os.path.exists(LOG_FILE)
    with open(LOG_FILE, "a", newline="") as f_csv:
        writer = csv.writer(f_csv)
        if new_file:
            writer.writerow([
                "timestamp_iso",
                "thrust_percent", "thrust_cmd",
                "n_loadcell", "loadcell_mean", "loadcell_median", "loadcell_std", "loadcell_min", "loadcell_max",
                "n_cf_samples", "avg_motor_pwm", "avg_battery_V", "avg_pitch_deg"
            ])

        try:
            # make sure we're starting from zero thrust pulses
            stop_pwm_override(cf)
            time.sleep(0.1)

            for pct in range(start, end + 1, step):

                if pct == 0:
                    pct = 1
                thrust_cmd = thrust_from_percent(pct)
                logger.info(f"=== Thrust step {pct}% (cmd={thrust_cmd}) ===")

                # ----- TARE BEFORE THIS STEP -----
                send_tare(ser, logger)

                step_start = time.time()
                step_end = step_start + DURATION_PER_STEP_S

                # accumulators
                loadcell_samples = []
                raw_lines = deque(maxlen=10000)

                cf_samples = 0
                motor_pwm_avg_sum = 0.0   # sum of per-sample (m1+m2+m3+m4)/4
                battery_sum = 0.0
                pitch_sum = 0.0

                set_pwm_all(cf, thrust_cmd)
                while time.time() < step_end:
                    now = time.time()
                    # Are we inside the collection window?
                    elapsed = now - step_start
                    collecting = (elapsed >= STABILIZE_WAIT_S) and (elapsed <= STABILIZE_WAIT_S + COLLECT_WINDOW_S)

                    # Read serial (bounded by timeout)
                    line = ser.readline()
                    if line:
                        raw_lines.append(line)
                        if collecting:
                            val = parse_loadcell_line(line)
                            if val is not None:
                                loadcell_samples.append(val)

                            logger.info(val)
                    # Also sample CF telemetry during the collection window
                    if collecting:
                        # Non-blocking peek at newest state (if any)
                        try:
                            latest = state_que.get_nowait()
                        except Exception:
                            latest = None

                        if latest is not None:
                            m_tuple = latest.get("m", None)
                            vbat = latest.get("vbat", None)
                            pdeg = latest.get("pitch", None)

                            if m_tuple and len(m_tuple) == 4:
                                motor_pwm_avg_sum += (m_tuple[0] + m_tuple[1] + m_tuple[2] + m_tuple[3]) / 4.0
                                cf_samples += 1
                            if isinstance(vbat, (int, float)) and not (vbat != vbat):  # not NaN
                                battery_sum += vbat
                            if isinstance(pdeg, (int, float)) and not (pdeg != pdeg):
                                pitch_sum += pdeg

                    time.sleep(0.01)

                set_pwm_all(cf, 0)
                stop_pwm_override(cf)

                time.sleep(1)

                # ---- stats: load cell ----
                if loadcell_samples:
                    import math
                    n = len(loadcell_samples)
                    mean = sum(loadcell_samples) / n
                    srt = sorted(loadcell_samples)
                    med = srt[n // 2] if n % 2 else 0.5 * (srt[n // 2 - 1] + srt[n // 2])
                    var = sum((x - mean) ** 2 for x in loadcell_samples) / (n - 1) if n > 1 else 0.0
                    std = math.sqrt(var)
                    mn = srt[0]
                    mx = srt[-1]
                else:
                    n = 0
                    mean = med = std = mn = mx = float('nan')

                # ---- stats: CF telemetry (averages over collection window) ----
                if cf_samples > 0:
                    avg_motor_pwm = motor_pwm_avg_sum / cf_samples
                    # battery & pitch may have fewer valid adds than cf_samples, but that’s OK for an average
                    avg_battery_V = battery_sum / cf_samples
                    avg_pitch_deg = pitch_sum / cf_samples
                else:
                    avg_motor_pwm = avg_battery_V = avg_pitch_deg = float('nan')

                # write row
                writer.writerow([
                    time.strftime("%Y-%m-%dT%H:%M:%S"),
                    pct, thrust_cmd,
                    n, mean, med, std, mn, mx,
                    cf_samples, avg_motor_pwm, avg_battery_V, avg_pitch_deg
                ])
                f_csv.flush()

                # optional raw dump per step
                if RAW_DIR is not None:
                    with open(os.path.join(RAW_DIR, f"raw_step_{pct:03d}.txt"), "wb") as f_raw:
                        f_raw.writelines(raw_lines)

                logger.info(
                    f"Step {pct}%: loadcell n={n}, mean={mean:.3f} | "
                    f"CF n={cf_samples}, avg_motor={avg_motor_pwm:.1f}, vbat={avg_battery_V:.3f} V, pitch={avg_pitch_deg:.2f}°"
                )

        finally:
            for _ in range(20):
                cf.hl_commander.send_setpoint(0.0, 0.0, 0.0, 0)
                time.sleep(0.02)
            ser.close()
            logger.info(f"Done. Summary saved to {LOG_FILE}. Raw: {RAW_DIR or 'disabled'}.")



def read_loadcell():
    """For each thrust %, tare the scale, stream setpoints, and collect load-cell + CF telemetry."""
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=READ_TIMEOUT_S)

    start_time = time.time()

    try:
        while True:
            send_tare(ser)
            # accumulators
            loadcell_samples = []
            raw_lines = deque(maxlen=10000)

            while time.time() - start_time < 5:
                line = ser.readline()
                # logger.info(line)
                if line:
                    raw_lines.append(line)
                    val = parse_loadcell_line(line)
                    if val is not None:
                        loadcell_samples.append(val)
                        print(val)
                time.sleep(0.001)

            start_time = time.time()

    finally:
        ser.close()


def manual_sweep_steps_with_loadcell(cf, state_que, logger, fixed_pct):
    if fixed_pct <= 0:
        fixed_pct = 1
    thrust_cmd = thrust_from_percent(fixed_pct)

    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=READ_TIMEOUT_S)
    if RAW_DIR:
        os.makedirs(RAW_DIR, exist_ok=True)

    new_file = not os.path.exists(LOG_FILE)
    with open(LOG_FILE, "a", newline="") as f_csv:
        writer = csv.writer(f_csv)
        if new_file:
            writer.writerow([
                "timestamp_iso",
                "thrust_percent", "thrust_cmd",
                "n_loadcell", "loadcell_mean", "loadcell_median", "loadcell_std", "loadcell_min", "loadcell_max",
                "n_cf_samples", "avg_motor_pwm", "avg_battery_V", "avg_pitch_deg"
            ])

        try:
            # make sure we're starting from zero thrust pulses
            for _ in range(10):
                cf.hl_commander.send_setpoint(0.0, 0.0, 0.0, 0)
                time.sleep(0.02)

            logger.info(f"Fixed thrust mode at {fixed_pct}% (cmd={thrust_cmd}).")
            logger.info("Change the drone pitch externally. Press Enter to TARE + SAMPLE. Type 'q' then Enter to quit.")

            step_idx = 0
            while True:
                try:
                    user_in = input("")  # blocks until Enter
                except EOFError:
                    user_in = "q"
                if user_in.strip().lower() == "q":
                    logger.info("Received quit signal.")
                    break

                step_idx += 1
                logger.info(f"=== Step #{step_idx}: Tare + collect at {fixed_pct}% thrust (cmd={thrust_cmd}) ===")

                # ----- TARE BEFORE THIS STEP -----
                send_tare(ser, logger)

                step_start = time.time()
                step_end = step_start + DURATION_PER_STEP_S

                # accumulators
                loadcell_samples = []
                raw_lines = deque(maxlen=10000)

                cf_samples = 0
                motor_pwm_avg_sum = 0.0   # sum of per-sample (m1+m2+m3+m4)/4
                battery_sum = 0.0
                pitch_sum = 0.0

                next_send_ts = 0.0

                while time.time() < step_end:
                    now = time.time()

                    # Keep motors alive: send setpoint at ~50 Hz using latest attitude
                    if now >= next_send_ts:
                        try:
                            data = state_que.get_nowait()
                            roll = float(data.get('roll', 0.0))
                            pitch = float(data.get('pitch', 0.0))
                        except Exception:
                            roll = 0.0
                            pitch = 0.0
                        cf.hl_commander.send_setpoint(roll=roll, pitch=pitch, yawrate=0.0, thrust=thrust_cmd)
                        next_send_ts = now + 0.02

                    # Are we inside the collection window?
                    elapsed = now - step_start
                    collecting = (elapsed >= STABILIZE_WAIT_S) and (elapsed <= STABILIZE_WAIT_S + COLLECT_WINDOW_S)

                    # Read serial (bounded by timeout)
                    line = ser.readline()
                    if line:
                        raw_lines.append(line)
                        val = parse_loadcell_line(line)
                        if val is not None:
                            logger.info(val)
                            if collecting:
                                loadcell_samples.append(val)

                    # Also sample CF telemetry during the collection window
                    if collecting:
                        # Non-blocking peek at newest state (if any)
                        try:
                            latest = state_que.get_nowait()
                        except Exception:
                            latest = None

                        if latest is not None:
                            m_tuple = latest.get("m", None)
                            vbat = latest.get("vbat", None)
                            pdeg = latest.get("pitch", None)

                            if m_tuple and len(m_tuple) == 4:
                                motor_pwm_avg_sum += (m_tuple[0] + m_tuple[1] + m_tuple[2] + m_tuple[3]) / 4.0
                                cf_samples += 1
                            if isinstance(vbat, (int, float)) and not (vbat != vbat):  # not NaN
                                battery_sum += vbat
                            if isinstance(pdeg, (int, float)) and not (pdeg != pdeg):
                                pitch_sum += pdeg

                    time.sleep(0.001)

                # Short cooldown at zero thrust, e.g., STABILIZE_WAIT_S seconds
                for _ in range(int(STABILIZE_WAIT_S * 50)):
                    cf.hl_commander.send_setpoint(0.0, 0.0, 0.0, 0)
                    time.sleep(0.02)

                # ---- stats: load cell ----
                if loadcell_samples:
                    import math
                    n = len(loadcell_samples)
                    mean = sum(loadcell_samples) / n
                    srt = sorted(loadcell_samples)
                    med = srt[n // 2] if n % 2 else 0.5 * (srt[n // 2 - 1] + srt[n // 2])
                    var = sum((x - mean) ** 2 for x in loadcell_samples) / (n - 1) if n > 1 else 0.0
                    std = math.sqrt(var)
                    mn = srt[0]
                    mx = srt[-1]
                else:
                    n = 0
                    mean = med = std = mn = mx = float('nan')

                # ---- stats: CF telemetry (averages over collection window) ----
                if cf_samples > 0:
                    avg_motor_pwm = motor_pwm_avg_sum / cf_samples
                    # battery & pitch may have fewer valid adds than cf_samples, but that’s OK for an average
                    avg_battery_V = battery_sum / cf_samples
                    avg_pitch_deg = pitch_sum / cf_samples
                else:
                    avg_motor_pwm = avg_battery_V = avg_pitch_deg = float('nan')

                # write row (same schema as before)
                writer.writerow([
                    time.strftime("%Y-%m-%dT%H:%M:%S"),
                    fixed_pct, thrust_cmd,
                    n, mean, med, std, mn, mx,
                    cf_samples, avg_motor_pwm, avg_battery_V, avg_pitch_deg
                ])
                f_csv.flush()

                # optional raw dump per step
                if RAW_DIR is not None:
                    raw_name = f"raw_step_{fixed_pct:03d}_{step_idx:03d}.txt"
                    with open(os.path.join(RAW_DIR, raw_name), "wb") as f_raw:
                        f_raw.writelines(raw_lines)

                logger.info(
                    f"Step #{step_idx} at {fixed_pct}%: loadcell n={n}, mean={mean:.3f} | "
                    f"CF n={cf_samples}, avg_motor={avg_motor_pwm:.1f}, vbat={avg_battery_V:.3f} V, pitch={avg_pitch_deg:.2f}°"
                )

        except KeyboardInterrupt:
            logger.info("Interrupted by user.")
        finally:
            for _ in range(20):
                cf.hl_commander.send_setpoint(0.0, 0.0, 0.0, 0)
                time.sleep(0.02)
            ser.close()
            logger.info(f"Done. Summary saved to {LOG_FILE}. Raw: {RAW_DIR or 'disabled'}.")



if __name__ == "__main__":
    read_loadcell()