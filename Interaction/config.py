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
LOG_VARS = {'VEL_ORI': VEL_ORI, 'POS_ACC': POS_ACC, 'MOT_BAT': MOT_BAT, 'POS_CTL_I_D': CTL_I_D, 'POS_VEL_CTL': POS_VEL_CTL, 'ATT_RATE_CTL': ATT_RATE_CTL}
# PID Configurations
PID_VALUES_PROP_2 = {
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
    'velCtlPid.vxKd': '0.0',
    'velCtlPid.vyKp': '30.0',
    'velCtlPid.vyKi': '4.0',
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


PID_VALUES_Interaction = {
    # 'quadSysId.armLength': '0.053',
    'posCtlPid.xKp': '1.9',
    'posCtlPid.xKi': '0.0',
    'posCtlPid.xKd': '0.0',
    'posCtlPid.yKp': '2.1',
    'posCtlPid.yKi': '0.0',
    'posCtlPid.yKd': '0.0',
    'posCtlPid.zKp': '1.9',
    'posCtlPid.zKi': '0.0',
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

PID_VALUES_PROP_2_GIMBAL = {
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
    'pid_attitude.roll_kp': '6.5',
    'pid_attitude.roll_ki': '0.0',
    'pid_attitude.roll_kd': '0.012',
    'pid_attitude.pitch_kp': '8',
    'pid_attitude.pitch_ki': '0.0',
    'pid_attitude.pitch_kd': '0.02',
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
