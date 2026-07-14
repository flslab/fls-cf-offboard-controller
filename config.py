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

MOT_ACC = {
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

CTRL_ATT_RATE = {
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
    "controller.pitchRate": {
        "type": "float",
        "unit": "rad/s",
        "scale": PI / 180,
        "data": [],
    },
    "controller.rollRate": {
        "type": "float",
        "unit": "rad/s",
        "scale": PI / 180,
        "data": [],
    },
    "controller.yawRate": {
        "type": "float",
        "unit": "rad/s",
        "scale": PI / 180,
        "data": [],
    },
}

CTRL_VEL_POS = {
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
}

POSCTL_VEL_POS = {
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

ATT_RATE = {
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
    "stateEstimateZ.ratePitch": {
        "type": "int16_t",
        "unit": "rad/s",
        "scale": 0.001,
        "data": [],
    },
    "stateEstimateZ.rateRoll": {
        "type": "int16_t",
        "unit": "rad/s",
        "scale": 0.001,
        "data": [],
    },
    "stateEstimateZ.rateYaw": {
        "type": "int16_t",
        "unit": "rad/s",
        "scale": 0.001,
        "data": [],
    },
}

VEL_POS = {
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
        "unit": "m",
        "data": [],
    },
    "stateEstimate.y": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "stateEstimate.z": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
}


KAL_FLOW = {
    "kalman_pred.predNX": {
        "type": "float",
        "unit": "px/fr",
        "data": [],
    },
    "kalman_pred.predNY": {
        "type": "float",
        "unit": "px/fr",
        "data": [],
    },
    "kalman_pred.measNX": {
        "type": "float",
        "unit": "px/fr",
        "data": [],
    },
    "kalman_pred.measNY": {
        "type": "float",
        "unit": "px/fr",
        "data": [],
    },
}

MOTION = {
    "motion.deltaX": {
        "type": "int16_t",
        "unit": "flow/fr",
        "data": [],
    },
    "motion.deltaY": {
        "type": "int16_t",
        "unit": "flow/fr",
        "data": [],
    },
    "motion.squal": {
        "type": "uint8_t",
        "unit": "#",
        "data": [],
    },
    "motion.std": {
        "type": "float",
        "unit": "flow/fr",
        "data": [],
    },
    "motion.maxRaw": {
        "type": "uint8_t",
        "unit": "px/fr",
        "data": [],
    },
    "motion.minRaw": {
        "type": "uint8_t",
        "unit": "px/fr",
        "data": [],
    },
    "motion.Rawsum": {
        "type": "uint8_t",
        "unit": "px/fr",
        "data": [],
    },
    "motion.outlierCount": {
        "type": "uint8_t",
        "unit": "#",
        "data": [],
    },
}

ACC = {
    "log_period_ms": 1,
    "acc.x": {
        "type": "float",
        "unit": "Gs",
        "data": [],
    },
    "acc.y": {
        "type": "float",
        "unit": "Gs",
        "data": [],
    },
    "acc.z": {
        "type": "float",
        "unit": "Gs",
        "data": [],
    }
}

GYRO = {
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
    "gyro.xRaw": {
        "type": "int16_t",
        "unit": "",
        "data": [],
    },
    "gyro.yRaw": {
        "type": "int16_t",
        "unit": "",
        "data": [],
    },
    "gyro.zRaw": {
        "type": "int16_t",
        "unit": "",
        "data": [],
    }
}

QUAT = {
    "log_period_ms": 1,
    "stateEstimate.qx": {
        "type": "float",
        "unit": "",
        "data": [],
    },
    "stateEstimate.qy": {
        "type": "float",
        "unit": "",
        "data": [],
    },
    "stateEstimate.qz": {
        "type": "float",
        "unit": "",
        "data": [],
    },
    "stateEstimate.qw": {
        "type": "float",
        "unit": "",
        "data": [],
    }
}

TUNING_LOG_VARS = {
    'CTRL_ATT_RATE': CTRL_ATT_RATE,
    'POSCTL_VEL_POS': POSCTL_VEL_POS,
    'ATT_RATE': ATT_RATE,
    'VEL_POS': VEL_POS,
    'MOT_ACC': MOT_ACC,
    'ACC': ACC,
    'GYRO': GYRO,
    'QUAT': QUAT,
}

LOG_VARS = TUNING_LOG_VARS
# LOG_VARS = {
#     'ATT_LOG_VARS': ATT_LOG_VARS,
#     'XY_ATTITUDE_LOG_VARS': XY_ATTITUDE_LOG_VARS,
#     'XY_VELOCITY_LOG_VARS': XY_VELOCITY_LOG_VARS,
#     'XY_POSITION_LOG_VARS': XY_POSITION_LOG_VARS,
# }


# PID Configurations
PID_VALUES = {
    # 'quadSysId.armLength': '0.053',
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

XY_PID_VALUES = {
    # 'quadSysId.armLength': '0.053',
    'posCtlPid.xKp': '1.9',
    'posCtlPid.xKi': '0.1',
    'posCtlPid.xKd': '0.0',
    'posCtlPid.yKp': '2.1',
    'posCtlPid.yKi': '0.1',
    'posCtlPid.yKd': '0.0',
    'posCtlPid.thrustMin': '12000',
    'posCtlPid.thrustBase': '28000',
    'velCtlPid.vxKp': '30.0',
    'velCtlPid.vxKi': '4.0',
    'velCtlPid.vxKd': '0.005',
    'velCtlPid.vyKp': '30.0',
    'velCtlPid.vyKi': '4.0',
    'velCtlPid.vyKd': '0.005',
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

XY_POS_PID_VALUES = {
    'posCtlPid.xKp': '1.9',
    'posCtlPid.xKi': '0.1',
    'posCtlPid.xKd': '0.0',
    'posCtlPid.yKp': '2.1',
    'posCtlPid.yKi': '0.1',
    'posCtlPid.yKd': '0.0',
}

PID_VALUES_FLOWDECK = {
    # 'quadSysId.armLength': '0.053',
    'posCtlPid.xKp': '0.5',
    'posCtlPid.xKi': '0.09',
    'posCtlPid.xKd': '0.0',
    'posCtlPid.yKp': '0.5',
    'posCtlPid.yKi': '0.09',
    'posCtlPid.yKd': '0.0',
    'posCtlPid.zKp': '1.9',
    'posCtlPid.zKi': '2.0',
    'posCtlPid.zKd': '0.05',
    'posCtlPid.thrustMin': '12000',
    'posCtlPid.thrustBase': '28000',
    'velCtlPid.vxKp': '10',
    'velCtlPid.vxKi': '0.012',
    'velCtlPid.vxKd': '0.0025',
    'velCtlPid.vyKp': '10',
    'velCtlPid.vyKi': '0.012',
    'velCtlPid.vyKd': '0.0025',
    'velCtlPid.vzKp': '10.0',
    'velCtlPid.vzKi': '0.0',
    'velCtlPid.vzKd': '0.05',
    'pid_attitude.roll_kp': '6.0',
    'pid_attitude.roll_ki': '1.0',
    'pid_attitude.roll_kd': '0.005',
    'pid_attitude.pitch_kp': '7.1',
    'pid_attitude.pitch_ki': '1.0',
    'pid_attitude.pitch_kd': '0.005',
    'pid_rate.roll_kp': '75',
    'pid_rate.roll_ki': '270.0',
    'pid_rate.roll_kd': '1.5',
    'pid_rate.pitch_kp': '75',
    'pid_rate.pitch_ki': '270.0',
    'pid_rate.pitch_kd': '1.5',
    'pid_rate.rateFiltEn': '1',
    'pid_rate.omxFiltCut': '70',
    'pid_rate.omyFiltCut': '70',
    'pid_rate.omzFiltCut': '70'
}
