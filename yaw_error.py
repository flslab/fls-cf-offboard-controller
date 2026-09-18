"""Dedicated Crazyflie external yaw-error transport and quality gate."""

from dataclasses import dataclass
import math
import struct

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort


GENERIC_LOCALIZATION_CHANNEL = 1
EXT_YAW_ERROR = 13


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def send_yaw_error(cf, yaw_error_rad, stddev_rad):
    """Send estimator-minus-external yaw error to patched Crazyflie firmware."""
    yaw_error_rad = float(yaw_error_rad)
    stddev_rad = float(stddev_rad)
    if not math.isfinite(yaw_error_rad) or abs(yaw_error_rad) > math.pi:
        raise ValueError("yaw error must be finite and within [-pi, pi]")
    if not math.isfinite(stddev_rad) or not 0.0 < stddev_rad <= math.pi:
        raise ValueError("yaw-error standard deviation must be in (0, pi]")

    packet = CRTPPacket()
    packet.port = CRTPPort.LOCALIZATION
    packet.channel = GENERIC_LOCALIZATION_CHANNEL
    packet.data = struct.pack('<Bff', EXT_YAW_ERROR, yaw_error_rad, stddev_rad)
    cf.send_packet(packet)


@dataclass(frozen=True)
class YawCorrectionConfig:
    enabled: bool = False
    stddev_rad: float = 0.01
    maximum_pnp_reprojection_error_px: float = 0.8
    minimum_feature_count: int = 10
    minimum_image_span_px: float = 80.0
    maximum_innovation_rad: float = math.radians(15.0)
    maximum_innovation_rate_rad_s: float = math.radians(120.0)
    minimum_consecutive_frames: int = 3
    send_rate_hz: float = 25.0

    @classmethod
    def from_mapping(cls, values):
        values = values or {}
        config = cls(**{
            field: values[field]
            for field in cls.__dataclass_fields__
            if field in values
        })
        positive = {
            'stddev_rad': config.stddev_rad,
            'maximum_pnp_reprojection_error_px': (
                config.maximum_pnp_reprojection_error_px
            ),
            'minimum_image_span_px': config.minimum_image_span_px,
            'maximum_innovation_rad': config.maximum_innovation_rad,
            'maximum_innovation_rate_rad_s': (
                config.maximum_innovation_rate_rad_s
            ),
            'send_rate_hz': config.send_rate_hz,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'yaw_correction.{name} must be positive')
        if config.stddev_rad > math.pi:
            raise ValueError('yaw_correction.stddev_rad must not exceed pi')
        if config.maximum_innovation_rad > math.pi:
            raise ValueError(
                'yaw_correction.maximum_innovation_rad must not exceed pi'
            )
        if config.minimum_feature_count < 4:
            raise ValueError(
                'yaw_correction.minimum_feature_count must be at least 4'
            )
        if config.minimum_consecutive_frames < 1:
            raise ValueError(
                'yaw_correction.minimum_consecutive_frames must be positive'
            )
        return config


@dataclass(frozen=True)
class YawCorrection:
    frame_id: int
    capture_timestamp: float
    yaw_error_rad: float
    stddev_rad: float
    pnp_reprojection_rms_px: float
    feature_count: int
    image_span_px: float
    valid_streak: int


class YawCorrectionGate:
    """Apply geometric, temporal, and rate gates to yaw innovations."""

    def __init__(self, config):
        self.config = config
        self._last_frame_id = None
        self._previous_candidate = None
        self._valid_streak = 0
        self._last_send_monotonic = -math.inf

    def _reject(self):
        self._previous_candidate = None
        self._valid_streak = 0

    def consider(self, output, acknowledged, tracking, now):
        if output.frame_id == self._last_frame_id:
            return None
        self._last_frame_id = output.frame_id

        error = output.yaw_error
        quality_valid = (
            self.config.enabled
            and acknowledged
            and tracking
            and output.yaw_error_valid
            and math.isfinite(output.timestamp)
            and math.isfinite(error)
            and abs(error) <= self.config.maximum_innovation_rad
            and math.isfinite(output.pnp_reprojection_rms)
            and output.pnp_reprojection_rms
            <= self.config.maximum_pnp_reprojection_error_px
            and output.feature_count >= self.config.minimum_feature_count
            and math.isfinite(output.pnp_image_span_px)
            and output.pnp_image_span_px
            >= self.config.minimum_image_span_px
        )
        if not quality_valid:
            self._reject()
            return None

        candidate = (output.timestamp, error)
        if self._previous_candidate is None:
            self._valid_streak = 1
        else:
            previous_timestamp, previous_error = self._previous_candidate
            elapsed = output.timestamp - previous_timestamp
            rate_valid = (
                0.0 < elapsed <= 0.25
                and abs(wrap_angle(error - previous_error)) / elapsed
                <= self.config.maximum_innovation_rate_rad_s
            )
            self._valid_streak = self._valid_streak + 1 if rate_valid else 1
        self._previous_candidate = candidate

        if self._valid_streak < self.config.minimum_consecutive_frames:
            return None
        if now - self._last_send_monotonic < 1.0 / self.config.send_rate_hz:
            return None
        self._last_send_monotonic = now
        return YawCorrection(
            frame_id=output.frame_id,
            capture_timestamp=output.timestamp,
            yaw_error_rad=error,
            stddev_rad=self.config.stddev_rad,
            pnp_reprojection_rms_px=output.pnp_reprojection_rms,
            feature_count=output.feature_count,
            image_span_px=output.pnp_image_span_px,
            valid_streak=self._valid_streak,
        )
