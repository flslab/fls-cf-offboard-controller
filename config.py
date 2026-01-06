# Connection Settings
DEFAULT_URI = 'usb://0'
# URI_DEFAULT = 'radio://0/80/2M/E7E7E7E713'

# Mocap Settings
MOCAP_HOST_NAME = '192.168.1.39'
MOCAP_SYSTEM_TYPE = 'vicon'
POSITION_STD_DEV = 0.001
ORIENTATION_STD_DEV = 0.06

# Flight Settings
DEFAULT_HEIGHT = 1.0
DEFAULT_DURATION = 10.0

# Localization Settings
LOCALIZATION_SHM_NAME = "/pos_shared_mem"

# Battery
MIN_LIHV_VOLT = 3.5

PI = 3.141592653589793

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
        "type": "uint16_t",
        "unit": "PWM",
        "data": [],
    },
    "motor.m2": {
        "type": "uint16_t",
        "unit": "PWM",
        "data": [],
    },
    "motor.m3": {
        "type": "uint16_t",
        "unit": "PWM",
        "data": [],
    },
    "motor.m4": {
        "type": "uint16_t",
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

LOG_VARS = XY_RATE_LOG_VARS

# PID Configurations
PID_VALUES_PROP_2 = {
    'quadSysId.armLength': '0.053',
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
    'posCtlPid.thrustBase': '28000',
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
