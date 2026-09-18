# Connection Settings
DEFAULT_URI = 'usb://0'
# DEFAULT_RADIO_URI = 'radio://0/100/2M/E7E7E7E706'
DEFAULT_RADIO_URI = 'radio://0/6/1M/E7E7E7E704'

# URI_DEFAULT = 'radio://0/80/2M/E7E7E7E713'

# Mocap Settings
MOCAP_HOST_NAME = '192.168.1.39'
MOCAP_SYSTEM_TYPE = 'vicon'
POSITION_STD_DEV = 0.001
ORIENTATION_STD_DEV = 0.001

# Flight Settings
DEFAULT_HEIGHT = 1.0
DEFAULT_DURATION = 10.0

# Localization Settings
LOCALIZATION_SHM_NAME = "/pos_shared_mem"

# Battery
MIN_LIHV_VOLT = 3.5
# MIN_LIHV_VOLT = 1.2

PI = 3.141592653589793

T_TO_PWM = {
    "a": 626.4245572193438,
    "b": -3562.5191163719087,
    "c": 19553.322555868166,
    "d": 7549.884731434861
}

# Logging Configuration
Z_LOG_VARS = {
    "ctrltarget.z": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "stateEstimate.z": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "ctrltarget.vz": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.vz": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
}

XY_POSITION_LOG_VARS = {
    "ctrltarget.x": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "stateEstimate.x": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "ctrltarget.y": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "stateEstimate.y": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
}

XY_VELOCITY_LOG_VARS = {
    "ctrltarget.vx": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.vx": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "ctrltarget.vy": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.vy": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
}

XY_ATTITUDE_LOG_VARS = {
    "controller.pitch": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "stateEstimate.pitch": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "controller.roll": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "stateEstimate.roll": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
}

XY_RATE_LOG_VARS = {
    "controller.pitchRate": {
        "type": "float",
        "unit": "rad/s",
        "scale": PI / 180,
        "data": [],
    },
    "stateEstimateZ.ratePitch": {
        "type": "int16_t",
        "unit": "rad/s",
        "scale": 0.001,
        "data": [],
    },
    "controller.rollRate": {
        "type": "float",
        "unit": "rad/s",
        "scale": PI / 180,
        "data": [],
    },
    "stateEstimateZ.rateRoll": {
        "type": "int16_t",
        "unit": "rad/s",
        "scale": 0.001,
        "data": [],
    },
}

MOTOR_LOG_VARS = {
    "motor.m1": {
        "type": "uint16_t_t",
        "unit": "PWM",
        "data": [],
    },
    "motor.m2": {
        "type": "uint16_t_t",
        "unit": "PWM",
        "data": [],
    },
    "motor.m3": {
        "type": "uint16_t_t",
        "unit": "PWM",
        "data": [],
    },
    "motor.m4": {
        "type": "uint16_t_t",
        "unit": "PWM",
        "data": [],
    },
}

KALMAN_POSITION_LOG_VARS = {
    "kalman.stateX": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "kalman.stateY": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "kalman.stateZ": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
}

ATT_LOG_VARS = {
    "stateEstimate.roll": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "stateEstimate.pitch": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "stateEstimate.yaw": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
}

FORCE_RENDER_LOG_VARS = {
    # "stateEstimate.z": {
    #     "type": "float",
    #     "unit": "m",
    #     "data": [],
    # },

    "stateEstimate.vx": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.roll": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "stateEstimate.pitch": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "stateEstimate.yaw": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "acc.x": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
}


TRANSLATION_LOG_VARS = {
    "stateEstimate.vx": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.vy": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.vz": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
}



POS_ORI = {
    "stateEstimate.roll": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "stateEstimate.pitch": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "stateEstimate.yaw": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "stateEstimate.x": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
    "stateEstimate.y": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },

    "stateEstimate.z": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
}


POS_VEL = {
    "stateEstimate.vx": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.vy": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.vz": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.x": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
    "stateEstimate.y": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },

    "stateEstimate.z": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
}


POS_ACC = {
    "stateEstimate.ax": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
    "stateEstimate.ay": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
    "stateEstimate.az": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
    "stateEstimate.x": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
    "stateEstimate.y": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },

    "stateEstimate.z": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
}



VEL_ACC = {
    "stateEstimate.vx": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.vy": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.vz": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.ax": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
    "stateEstimate.ay": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
    "stateEstimate.az": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
}



MOT_BAT = {
    "motor.m1": {
        "type": "uint16_t",
        "unit": "None",
        "data": [],
    },
    "motor.m2": {
        "type": "uint16_t",
        "unit": "None",
        "data": [],
    },
    "motor.m3": {
        "type": "uint16_t",
        "unit": "None",
        "data": [],
    },
    "motor.m4": {
        "type": "uint16_t",
        "unit": "None",
        "data": [],
    },
    "pm.vbat": {
        "type": "float",
        "unit": "V",
        "data": [],
    },
}


VEL_ORI = {
    "stateEstimate.vx": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.vy": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.vz": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "stateEstimate.roll": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "stateEstimate.pitch": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "stateEstimate.yaw": {
        "type": "float",
        "unit": "deg",
        "data": [],
    }
}


# Compressed onboard angular-rate estimate. Values are milliradians/second in
# the Crazyflie firmware and are converted to radians/second by the onboard
# interaction path.
RATE_EST = {
    "stateEstimateZ.rateRoll": {
        "type": "int16_t",
        "unit": "mrad/s",
        "scale": 0.001,
        "data": [],
    },
    "stateEstimateZ.ratePitch": {
        "type": "int16_t",
        "unit": "mrad/s",
        "scale": 0.001,
        "data": [],
    },
    "stateEstimateZ.rateYaw": {
        "type": "int16_t",
        "unit": "mrad/s",
        "scale": 0.001,
        "data": [],
    },
}


# Simulation-only firmware 15-state reference packets.  They are kept in
# separate CRTP blocks so the reference can be sampled without changing the
# active stateEstimate/PID estimator path.
P_REL_STATUS = {
    "log_period_ms": 20,
    "kalmanPRel.active": {"type": "uint8_t", "unit": "bool", "data": []},
    "kalmanPRel.armPend": {"type": "uint8_t", "unit": "bool", "data": []},
    "kalmanPRel.primed": {"type": "uint8_t", "unit": "bool", "data": []},
    "kalmanPRel.valid": {"type": "uint8_t", "unit": "bool", "data": []},
    "kalmanPRel.brkReady": {"type": "uint8_t", "unit": "bool", "data": []},
    "kalmanPRel.reason": {"type": "uint8_t", "unit": "enum", "data": []},
    "kalmanPRel.timeUs": {"type": "uint32_t", "unit": "us", "data": []},
    "kalmanPRel.relN": {"type": "uint32_t", "unit": "count", "data": []},
    "kalmanPRel.strictOk": {"type": "uint8_t", "unit": "bool", "data": []},
    "kalmanPRel.relSense": {"type": "uint8_t", "unit": "bool", "data": []},
    # Do not stream raw 1 kHz IMU samples through CRTP. The estimator consumes
    # them in firmware; these counters sampled at 50 Hz prove its actual
    # propagation rate and worst observed source gap without starving control
    # and motor packets.
    "kalmanPRel.imuN": {"type": "uint32_t", "unit": "count", "data": []},
    "kalmanPRel.maxGapUs": {"type": "uint32_t", "unit": "us", "data": []},
}

P_REL_ATT = {
    "log_period_ms": 10,
    "kalmanPRel.q0": {"type": "float", "unit": "1", "data": []},
    "kalmanPRel.q1": {"type": "float", "unit": "1", "data": []},
    "kalmanPRel.q2": {"type": "float", "unit": "1", "data": []},
    "kalmanPRel.q3": {"type": "float", "unit": "1", "data": []},
    # Keep planar velocity in the same atomic 10 ms block as attitude.  Six
    # floats fit the CRTP log payload exactly and prevent the free-stop
    # planner from combining the isolated estimator's current attitude with
    # the ordinary Kalman filter's differently delayed velocity.
    "kalmanPRel.vx": {"type": "float", "unit": "m/s", "data": []},
    "kalmanPRel.vy": {"type": "float", "unit": "m/s", "data": []},
}

P_REL_ACC = {
    "log_period_ms": 10,
    "kalmanPRel.ax": {"type": "float", "unit": "m/s^2", "data": []},
    "kalmanPRel.ay": {"type": "float", "unit": "m/s^2", "data": []},
    "kalmanPRel.az": {"type": "float", "unit": "m/s^2", "data": []},
}

P_REL_FUSION = {
    "log_period_ms": 20,
    "kalmanPRel.posN": {"type": "uint32_t", "unit": "count", "data": []},
    "kalmanPRel.posRej": {"type": "uint32_t", "unit": "count", "data": []},
    "kalmanPRel.oriN": {"type": "uint32_t", "unit": "count", "data": []},
    "kalmanPRel.oriRej": {"type": "uint32_t", "unit": "count", "data": []},
    "kalmanPRel.oriTimeUs": {"type": "uint32_t", "unit": "us", "data": []},
}

P_REL_HLC = {
    "log_period_ms": 20,
    "kalmanPRel.seedSkew": {"type": "uint32_t", "unit": "us", "data": []},
    "kalmanPRel.transEpoch": {"type": "uint8_t", "unit": "bool", "data": []},
    "kalmanPRel.transAgree": {"type": "uint8_t", "unit": "count", "data": []},
    "kalmanPRel.transErr": {"type": "float", "unit": "m/s", "data": []},
    "hlCommander.pRelUsed": {"type": "uint8_t", "unit": "bool", "data": []},
    "hlCommander.pRelReady": {"type": "uint8_t", "unit": "bool", "data": []},
    "hlCommander.pRelEn": {"type": "uint8_t", "unit": "bool", "data": []},
    "hlCommander.pRelGoN": {"type": "uint8_t", "unit": "count", "data": []},
    "hlCommander.pRelTryN": {"type": "uint8_t", "unit": "count", "data": []},
    "hlCommander.pRelKeepN": {"type": "uint8_t", "unit": "count", "data": []},
    "hlCommander.pRelLandN": {"type": "uint8_t", "unit": "count", "data": []},
    "hlCommander.pRelVx": {"type": "float", "unit": "m/s", "data": []},
    "hlCommander.pRelVy": {"type": "float", "unit": "m/s", "data": []},
}


def log_vars_for_crazysim(mission):
    """Use the control-essential streams plus the isolated 15-state reference.

    The Gazebo UDP bridge serializes CRTP packets and cannot sustain the
    hardware diagnostic set (notably the 1 kHz gyro block) alongside the
    reference estimator without starving motor/state packets. Simulator
    odometry is captured independently by the CrazySim runner.
    """
    return {
        'VEL_ORI': VEL_ORI,
        'POS_ACC': POS_ACC,
        'RATE_EST': RATE_EST,
        'ATT_DES': ATT_DES,
        # These groups are needed for diagnostics and battery protection, but
        # not for the 100 Hz release-state/planner loop.  Slowing them prevents
        # the serialized UDP bridge from batching the control-essential state
        # and 15-state reference packets into 80--100 ms bursts.
        'YAW_CTL': {'log_period_ms': 50, **YAW_CTL},
        'MOT_BAT': {'log_period_ms': 50, **MOT_BAT},
        'P_REL_STATUS': P_REL_STATUS,
        'P_REL_ATT': P_REL_ATT,
        'P_REL_ACC': P_REL_ACC,
        'P_REL_FUSION': P_REL_FUSION,
        'P_REL_HLC': P_REL_HLC,
    }


# Full-resolution, bias-corrected IMU gyro samples.  Keep this in its own
# compact block so the 1 kHz diagnostic stream does not raise the rate of the
# position, estimator, motor, or controller log groups.
GYRO_1KHZ = {
    "log_period_ms": 1,
    "gyro.x": {
        "type": "float",
        "unit": "deg/s",
        "data": [],
    },
    "gyro.y": {
        "type": "float",
        "unit": "deg/s",
        "data": [],
    },
    "gyro.z": {
        "type": "float",
        "unit": "deg/s",
        "data": [],
    },
}


# Opt-in firmware-latched replacement for GYRO_1KHZ. Twelve FP16 values and
# one uint16 epoch fill one 26-byte CRTP log payload, so gyro, accelerometer,
# onboard position, and onboard velocity share one producer sample and consume
# one radio packet per requested millisecond instead of three.
CONTACT_IMU_1KHZ = {
    "log_period_ms": 1,
    "contactImu.gx": {
        "type": "FP16", "unit": "deg/s", "data": [],
    },
    "contactImu.gy": {
        "type": "FP16", "unit": "deg/s", "data": [],
    },
    "contactImu.gz": {
        "type": "FP16", "unit": "deg/s", "data": [],
    },
    "contactImu.ax": {"type": "FP16", "unit": "g", "data": []},
    "contactImu.ay": {"type": "FP16", "unit": "g", "data": []},
    "contactImu.az": {"type": "FP16", "unit": "g", "data": []},
    "contactImu.px": {"type": "FP16", "unit": "m", "data": []},
    "contactImu.py": {"type": "FP16", "unit": "m", "data": []},
    "contactImu.pz": {"type": "FP16", "unit": "m", "data": []},
    "contactImu.vx": {"type": "FP16", "unit": "m/s", "data": []},
    "contactImu.vy": {"type": "FP16", "unit": "m/s", "data": []},
    "contactImu.vz": {"type": "FP16", "unit": "m/s", "data": []},
    "contactImu.epoch": {
        "type": "uint16_t", "unit": "ms low16", "data": [],
    },
}


# PID yaw actuator output and the gyro rate used by the PID controller. Keep
# these in their own small CRTP log block so the existing log groups stay below
# the Crazyflie log-packet payload limit.
YAW_CTL = {
    "controller.cmd_yaw": {
        "type": "float",
        "unit": "controller output",
        "data": [],
    },
    "controller.r_yaw": {
        "type": "float",
        "unit": "rad/s",
        "data": [],
    },
}


# Desired roll/pitch produced by the firmware position/velocity cascade.  The
# ordinary XYZ calibration uses this 100 Hz input together with the separate
# 15-state attitude reference to identify the inner attitude response without
# adding a dedicated multi-trial protocol.
ATT_DES = {
    "controller.roll": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "controller.pitch": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
}


# Controller loop stage 1: Position → Velocity output, Velocity → Attitude output
POS_VEL_CTL = {
    "posCtl.targetVX": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "posCtl.targetVY": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "posCtl.targetVZ": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "posCtl.targetX": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "posCtl.targetY": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "posCtl.targetZ": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
}

# Controller loop stage 2: Attitude → Rate output, Rate → Actuator output
ATT_RATE_CTL = {
    "controller.rollRate": {
        "type": "float",
        "unit": "deg/s",
        "data": [],
    },
    "controller.pitchRate": {
        "type": "float",
        "unit": "deg/s",
        "data": [],
    },
    "controller.yawRate": {
        "type": "float",
        "unit": "deg/s",
        "data": [],
    },
    "controller.roll": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "controller.pitch": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "controller.yaw": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
}

TARGET_POS_VEL = {
    "ctrltarget.x": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "ctrltarget.y": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "ctrltarget.z": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "ctrltarget.vx": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "ctrltarget.vy": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "ctrltarget.vz": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
}


TARGET_ACC_ATT = {
    "ctrltarget.ax": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
    "ctrltarget.ay": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
    "ctrltarget.az": {
        "type": "float",
        "unit": "m/s^2",
        "data": [],
    },
    "ctrltarget.roll": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "ctrltarget.pitch": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
    "ctrltarget.yaw": {
        "type": "float",
        "unit": "deg",
        "data": [],
    },
}

CTL_I_D = {
    "posCtl.Xi": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "posCtl.Yi": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "posCtl.VXi": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "posCtl.VYi": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "posCtl.VXd": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
    "posCtl.VYd": {
        "type": "float",
        "unit": "m/s",
        "data": [],
    },
}

# LOG_VARS = {'POS_ORI': POS_ORI, 'VEL_ACC': VEL_ACC, 'MOT_BAT': MOT_BAT}
# LOG_VARS = {'POS_VEL': POS_VEL}
LOG_VARS = {
    'VEL_ORI': VEL_ORI,
    'POS_ACC': POS_ACC,
    # 'RATE_EST': RATE_EST,
    # Retained above for explicit diagnostic tools; the ordinary interaction
    # path no longer subscribes to a 1 kHz stream that it does not consume.
    'YAW_CTL': YAW_CTL,
    # 'MOT_BAT': MOT_BAT,
    # 'POS_CTL_I_D': CTL_I_D,
    'POS_VEL_CTL': POS_VEL_CTL,
    'ATT_RATE_CTL': ATT_RATE_CTL,
}

FIRMWARE_BRAKE_LOG_VARS = {
    'log_period_ms': 100,
    'hlCommander.pRelReady': {'type': 'uint8_t', 'unit': '', 'data': []},
    'hlCommander.pRelAutoSt': {'type': 'uint8_t', 'unit': '', 'data': []},
    'hlCommander.pRelAutoN': {'type': 'uint32_t', 'unit': '', 'data': []},
    'hlCommander.pRelAutoRej': {'type': 'uint32_t', 'unit': '', 'data': []},
    'hlCommander.pRelAutoTime': {'type': 'uint32_t', 'unit': '', 'data': []},
    'hlCommander.pRelAutoEn': {'type': 'uint8_t', 'unit': '', 'data': []},
    'hlCommander.pRelEvtVer': {'type': 'uint8_t', 'unit': '', 'data': []},
    'hlCommander.pRelMode': {'type': 'uint8_t', 'unit': '', 'data': []},
    'hlCommander.pRelTau': {'type': 'float', 'unit': 's', 'data': []},
}


def log_vars_for_mission(mission):
    """Return interaction logs; the offboard inertial shadow is opt-in only.

    The ordinary path and onboard-mirror comparison do not need IMU samples
    at 1 kHz. Only an explicitly selected offboard inertial shadow requests
    the firmware-latched contactImu packet. The diagnostic definition remains
    available to standalone tools and legacy comparison runs.
    """
    try:
        wrench_config = mission['Interaction']['config'].get(
            'wrench_interaction'
        ) or {}
        enabled = wrench_config.get(
            'contact_attitude_shadow_enabled', False
        )
        mode = wrench_config.get(
            'contact_attitude_shadow_mode', 'inertial_position'
        )
        firmware_brake = (
            wrench_config.get('firmware_auto_brake') or {}
        ).get('enabled', False)
    except (AttributeError, KeyError, TypeError):
        enabled = False
        mode = 'inertial_position'
        firmware_brake = False
    if not isinstance(enabled, bool):
        raise ValueError(
            'contact_attitude_shadow_enabled must be boolean'
        )
    if type(firmware_brake) is not bool:
        raise ValueError('firmware_auto_brake.enabled must be boolean')
    if firmware_brake:
        if enabled:
            raise ValueError('firmware auto brake cannot use offboard shadow')
        return {**LOG_VARS, 'FIRMWARE_BRAKE': FIRMWARE_BRAKE_LOG_VARS}
    # A dormant diagnostic must not make a legacy mission fail because it
    # happens to carry an old or misspelled shadow-mode value.  Mode validation
    # belongs exclusively to the explicitly enabled path.
    if not enabled:
        return LOG_VARS
    if mode not in ('onboard_mirror', 'inertial_position'):
        raise ValueError(
            'contact_attitude_shadow_mode must be onboard_mirror or '
            'inertial_position'
        )
    if mode == 'inertial_position':
        return {
            **LOG_VARS,
            'GYRO_1KHZ': CONTACT_IMU_1KHZ,
        }
    return LOG_VARS
# PID Configurations
PID_VALUES_PROP_2_NO_I = {
    # 'quadSysId.armLength': '0.053',
    'posCtlPid.xKp': '1.9',
    'posCtlPid.xKi': '0.0',
    'posCtlPid.xKd': '0.0',
    'posCtlPid.yKp': '2.1',
    'posCtlPid.yKi': '0.0',
    'posCtlPid.yKd': '0.0',
    'posCtlPid.zKp': '1.9',
    'posCtlPid.zKi': '2.0',
    'posCtlPid.zKd': '0.05',
    'posCtlPid.thrustMin': '12000',
    'posCtlPid.thrustBase': '28000',
    'velCtlPid.vxKp': '30.0',
    'velCtlPid.vxKi': '0.0',
    'velCtlPid.vxKd': '0.0',
    'velCtlPid.vyKp': '30.0',
    'velCtlPid.vyKi': '0.0',
    'velCtlPid.vyKd': '0.0',
    'velCtlPid.vzKp': '30.0',
    'velCtlPid.vzKi': '5.0',
    'velCtlPid.vzKd': '0.0',
    'pid_attitude.roll_kp': '6.0',
    'pid_attitude.roll_ki': '1.0',
    'pid_attitude.roll_kd': '0.005',
    'pid_attitude.pitch_kp': '7.1',
    'pid_attitude.pitch_ki': '1.0',
    'pid_attitude.pitch_kd': '0.005',
    'pid_rate.roll_kp': '90',
    'pid_rate.roll_ki': '270.0',
    'pid_rate.roll_kd': '2.5',
    'pid_rate.pitch_kp': '75',
    'pid_rate.pitch_ki': '270.0',
    'pid_rate.pitch_kd': '2.5',
    'pid_rate.rateFiltEn': '1',
    'pid_rate.omxFiltCut': '160',
    'pid_rate.omyFiltCut': '160',
    'pid_rate.omzFiltCut': '160'
}


PID_VALUES_PROP_2 = {
    'posCtlPid.xKp': '1.9',
    'posCtlPid.xKi': '0.1',
    'posCtlPid.xKd': '0.0',
    'posCtlPid.yKp': '2.1',
    'posCtlPid.yKi': '0.1',
    'posCtlPid.yKd': '0.0',
    'posCtlPid.zKp': '1.9',
    'posCtlPid.zKi': '2.0',
    'posCtlPid.zKd': '0.05',
    'posCtlPid.thrustMin': '12000',
    'posCtlPid.thrustBase': '38000',
    'velCtlPid.vxKp': '30.0',
    'velCtlPid.vxKi': '4.0',
    'velCtlPid.vxKd': '0.005',
    'velCtlPid.vyKp': '30.0',
    'velCtlPid.vyKi': '4.0',
    'velCtlPid.vyKd': '0.005',
    'velCtlPid.vzKp': '30.0',
    'velCtlPid.vzKi': '5.0',
    'velCtlPid.vzKd': '0.05',
    'pid_attitude.roll_kp': '6.0',
    'pid_attitude.roll_ki': '1.0',
    'pid_attitude.roll_kd': '0.005',
    'pid_attitude.pitch_kp': '7.1',
    'pid_attitude.pitch_ki': '1.0',
    'pid_attitude.pitch_kd': '0.005',
    'pid_rate.roll_kp': '90',
    'pid_rate.roll_ki': '270.0',
    'pid_rate.roll_kd': '2.5',
    'pid_rate.pitch_kp': '75',
    'pid_rate.pitch_ki': '270.0',
    'pid_rate.pitch_kd': '2.5',
    'pid_rate.rateFiltEn': '1',
    'pid_rate.omxFiltCut': '160',
    'pid_rate.omyFiltCut': '160',
    'pid_rate.omzFiltCut': '160'
}
PID_VALUES = PID_VALUES_PROP_2
