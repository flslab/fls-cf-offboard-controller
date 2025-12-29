#!/usr/bin/python3
import os
import time


class pi5RC:
    def __init__(self, Pin):
        # Define supported GPIO pins and their mappings
        pins = [12, 13, 14, 15, 18, 19]
        afunc = ['a0', 'a0', 'a0', 'a0', 'a3', 'a3']
        pwmchip_map = [0, 0, -1, -1, 0, 0]
        pwmchan_map = [0, 1, -1, -1, 2, 3]

        if Pin not in pins:
            raise ValueError(f"Unsupported PWM pin: GPIO{Pin}")

        self.pin = Pin
        self.pinIdx = pins.index(Pin)
        self.pwmchip = pwmchip_map[self.pinIdx]
        self.pwmchan = pwmchan_map[self.pinIdx]
        self.enableFlag = False
        self.file_duty = None
        self.pwm_path = f"/sys/class/pwm/pwmchip{self.pwmchip}/pwm{self.pwmchan}"

        # Set pin function
        os.system(f"/usr/bin/pinctrl set {self.pin} {afunc[self.pinIdx]}")
        time.sleep(0.1)

        # Export if not already
        if not os.path.exists(self.pwm_path):
            try:
                with open(f"/sys/class/pwm/pwmchip{self.pwmchip}/export", "w") as f:
                    f.write(str(self.pwmchan))
                time.sleep(0.2)
            except OSError as e:
                if "Device or resource busy" not in str(e):
                    raise e

        # Set 20ms period (50Hz servo signal)
        self._write(f"{self.pwm_path}/period", "20000000")
        self.enable(False)
        self.file_duty = open(f"{self.pwm_path}/duty_cycle", "w")

    def enable(self, flag: bool):
        self.enableFlag = flag
        self._write(f"{self.pwm_path}/enable", "1" if flag else "0")

    def set(self, angle: float, angle_range: float = 180.0, pulse_range: tuple = (500, 2500)):
        pulse_width = int(((angle / angle_range) * (pulse_range[1] - pulse_range[0]) + pulse_range[0]))
        self.set_pwm(pulse_width)

    def set_pwm(self, onTime_us: int):
        """Set pulse width in microseconds (e.g., 1500 for center)"""
        if not self.enableFlag:
            self.enable(True)
        self.onTime_us = onTime_us
        self.file_duty.seek(0)
        self.file_duty.write(str(onTime_us * 1000))  # Convert µs to ns
        self.file_duty.flush()

    def _write(self, path, value):
        try:
            with open(path, "w") as f:
                f.write(value)
        except Exception as e:
            print(f"Failed to write to {path}: {e}")
            raise

    def __del__(self):
        try:
            if self.file_duty and not self.file_duty.closed:
                self.file_duty.close()
            self.enable(False)
            if os.path.exists(f"/sys/class/pwm/pwmchip{self.pwmchip}/unexport"):
                with open(f"/sys/class/pwm/pwmchip{self.pwmchip}/unexport", "w") as f:
                    f.write(str(self.pwmchan))
            os.system(f"/usr/bin/pinctrl set {self.pin} no")
        except Exception as e:
            print(f"Cleanup failed: {e}")
