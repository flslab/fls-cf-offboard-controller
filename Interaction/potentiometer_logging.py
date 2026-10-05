"""Optional raw potentiometer recording, independent of the contact detector."""

import logging

from Interaction.contact_validation_capture import PotentiometerSampleCapture
from Interaction.potentiometer_force_sensor import (
    COMPRESSION_CALIBRATION_METHOD,
    RAW_COMPRESSION_CALIBRATION,
)


logger = logging.getLogger(__name__)


def validate_potentiometer_recording(mission, args):
    """Reject incomplete recording requests before flight; ignore other modes."""
    if any(getattr(args, name, False) for name in (
            'calibrate', 'braking_test', 'mpc', 'baseline', 'hover',
            'active_septic_brake_calibration')):
        return False
    if not (getattr(args, 'interaction', False) or getattr(args, 'sense', False)):
        return False
    config = ((mission or {}).get('Interaction') or {}).get('config') or {}
    enabled = config.get('record_potentiometer', False)
    if type(enabled) is not bool:
        raise ValueError('Interaction.config.record_potentiometer must be boolean')
    if enabled and (not getattr(args, 'sense', False)
                    or not getattr(args, 'log', False)
                    or getattr(args, 'droneless', False)):
        raise ValueError('record_potentiometer requires --sense --log without --droneless; '
                         'the orchestrator supplies --sense automatically')
    return enabled


def configure_potentiometer_recording(log_manager, mission, args):
    """Return one UART callback, sharing the existing validation capture if set."""
    existing = getattr(log_manager, 'contact_validation_pot_callback', None)
    if not validate_potentiometer_recording(mission, args):
        return existing
    writer = getattr(log_manager, 'live_logger', None)
    if writer is None:
        raise ValueError('record_potentiometer requires the interaction flight logger')
    config = mission['Interaction']['config']
    detector = config.get('detection_method',
                          'potentiometer' if config.get('behavior') == 'level_coast' else 'default')
    writer.write({'type': 'potentiometer_recording', 'data': {
        'schema_version': 1,
        'record_potentiometer': True,
        'primary_detector': detector,
        'sampling': 'every valid UART sample, before control-loop decimation',
        'compression_calibration': {
            'input': 'raw_adc',
            'method': COMPRESSION_CALIBRATION_METHOD,
            'raw_compression_mm_points': RAW_COMPRESSION_CALIBRATION,
            'zero_allowance_adc_counts': 1,
            'out_of_range': 'invalid sample; no extrapolation',
        },
        'force_model': {
            'type': 'linear_spring',
            'spring_constant_n_per_mm': args.sense_spring_constant,
            'quantity': 'compression force estimate along the spring',
        },
        'sensor_settings': {
            'port': args.sense_port,
            'baud': args.sense_baud,
            'max_extension_mm': args.sense_max_extension,
        },
    }})
    logger.info('Potentiometer recording enabled: compression_mm and force_n for every '
                'valid UART sample; detector=%s; calibration=%s',
                detector, COMPRESSION_CALIBRATION_METHOD)
    return existing if existing is not None else PotentiometerSampleCapture(writer)
