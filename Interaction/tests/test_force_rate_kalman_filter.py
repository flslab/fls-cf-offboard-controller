"""Numerical and sampling contracts for the paper's KID force-rate KF."""
import numpy as np
import pytest

from Interaction.force_rate_kalman_filter import (
    ForceRateKalmanFilter, measurement_covariance_from_hover,
)
from Interaction.wrench_contact_detector import ContactChannelDetector


def detector(thresholds=(.12, .12, .12)):
    return ContactChannelDetector(
        thresholds, [.015]*3, use_confidence_gate=False, onset_evidence_s=.05,
        release_time_s=0., release_ratio=.55, onset_axes=[0, 1],
        release_projection_axes=[0, 1], max_sample_gap_s=.05)


def test_first_observation_initializes_force_zero_rate_and_covariance():
    r = np.array([[.0004, .0001], [.0001, .0009]])
    kf = ForceRateKalmanFilter(measurement_covariance=r)
    result = kf.update([.2, -.3], 10.)
    np.testing.assert_allclose(kf.state, [.2, -.3, 0., 0.])
    np.testing.assert_allclose(kf.covariance, np.block([[r, np.zeros((2, 2))],
                                                                    [np.zeros((2, 2)), r]]))
    assert not result.ready and not result.rejected
    assert not kf.update([.2, -.3], 10.01).ready
    assert kf.update([.2, -.3], 10.02).ready


def test_variable_interval_update_matches_independent_kalman_equations():
    r = np.array([[.0004, .00012], [.00012, .0009]])
    kf = ForceRateKalmanFilter(measurement_covariance=r)
    kf.update([.1, -.2], 0.)
    x, p = kf.state.copy(), kf.covariance.copy()
    h = .017 / .010
    a = np.eye(4)
    a[0, 2] = a[1, 3] = h
    observation_matrix = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.]])
    q = .1 * np.kron([[h**3 / 3., h**2 / 2.], [h**2 / 2., h]], r)
    x = a @ x
    p = a @ p @ a.T + q
    gain = p @ observation_matrix.T @ np.linalg.inv(observation_matrix @ p @ observation_matrix.T + r)
    x += gain @ (np.array([.4, -.1]) - observation_matrix @ x)
    p = (np.eye(4) - gain @ observation_matrix) @ p
    result = kf.update([.4, -.1], .017)
    np.testing.assert_allclose(kf.state, x, atol=1e-14)
    np.testing.assert_allclose(kf.covariance, p, atol=1e-14)
    np.testing.assert_allclose(result.rate_xy, x[2:] / .01)


def test_filter_tracks_ramp_and_has_positive_covariance_over_irregular_samples():
    kf = ForceRateKalmanFilter()
    t = 0.
    for i in range(200):
        t += [.007, .013, .011][i % 3]
        result = kf.update([.3*t, -.2*t], t)
        assert np.linalg.eigvalsh(kf.covariance).min() > 0
    np.testing.assert_allclose(result.force_xy, [.3*t, -.2*t], atol=1e-8)
    np.testing.assert_allclose(result.rate_xy, [.3, -.2], atol=1e-8)


def test_gap_duplicate_and_invalid_reset_only_filter_history():
    kf = ForceRateKalmanFilter()
    for i in range(10):
        kf.update([.3, .1], i*.01)
    result = kf.update([0., 0.], .2)
    assert result.rejected and not result.ready
    np.testing.assert_array_equal(kf.state, 0.)
    assert not kf.update([0., 0.], .21).ready
    assert kf.update([0., 0.], .22).ready
    assert kf.update([0., 0.], .22).rejected
    assert kf.last_timestamp is None
    with pytest.raises(ValueError):
        kf.update([float('nan'), 0.], .23)
    assert kf.last_timestamp is None


def test_hover_covariance_uses_only_fresh_adjacent_differences():
    observations = np.array([[0., 0.], [.01, .02], [.03, .01],
                             [99., 99.], [98.99, 99.03], [99.01, 99.02]])
    times = [0., .01, .02, .1, .11, .12]
    differences = np.diff(observations, axis=0)[[0, 1, 3, 4]]
    expected = .5*np.cov(differences, rowvar=False)
    expected[0, 1] *= .95
    expected[1, 0] *= .95
    actual = measurement_covariance_from_hover(observations, times)
    np.testing.assert_allclose(actual, expected, atol=1e-14)
    np.testing.assert_allclose(measurement_covariance_from_hover(np.zeros((30, 2)), np.arange(30)*.01),
                               np.eye(2)*1e-8, atol=1e-14)
    with pytest.raises(ValueError):
        measurement_covariance_from_hover([[0., 0.], [1., 1.]], [0., .1])


def test_evidence_uses_force_only_minimum_rate_and_linear_decay():
    d = detector()
    huge_covariance = np.eye(3)*1000.
    d.update([0., 0., 0.], huge_covariance, 0.)
    assert d.update([.12, 0., 0.], huge_covariance, .01).evidence == pytest.approx(.0025)
    assert d.update([.36, 0., 0.], huge_covariance, .02).evidence == pytest.approx(.0225)
    assert d.update([0., 0., 0.], huge_covariance, .03).evidence == pytest.approx(.0125)
    assert d.update([.36, 0., 0.], huge_covariance, .04).evidence == pytest.approx(.0325)
    result = d.update([.36, 0., 0.], huge_covariance, .05)
    assert result.started and result.confidence_sigma == 0.
    assert result.release_direction == (1., 0., 0.)


@pytest.mark.parametrize('release_force', [[.066, 0., 0.], [-.2, 0., 0.], [0., 1., 0.]])
def test_release_first_valid_sample_uses_signed_projection(release_force):
    d = detector()
    d.update([.36, 0., 0.], np.eye(3), 0.)
    for i in range(1, 5):
        result = d.update([.36, 0., 0.], np.eye(3), .01*i)
        if result.started:
            break
    assert d.active
    result = d.update(release_force, np.eye(3), .01*(i+1))
    assert result.ended and not result.active and result.evidence == 0.


def test_gap_and_duplicate_cannot_release_and_new_push_locks_new_direction():
    d = detector((.12, .24, .12))
    d.update([1., 0., 0.], np.eye(3), 0.)
    assert d.update([1., 0., 0.], np.eye(3), .01).started
    assert not d.update([0., 0., 0.], np.eye(3), .01).ended
    assert not d.update([0., 0., 0.], np.eye(3), .10).ended
    assert d.update([0., 0., 0.], np.eye(3), .11).ended
    # The flight state machine explicitly rearms after its configured grace.
    d.reset(.11)
    assert d.update([0., 2., 0.], np.eye(3), .12).started
    result = d.update([0., .132, 0.], np.eye(3), .13)
    assert result.ended and result.release_direction == (0., 1., 0.)


@pytest.mark.parametrize('options', [
    {'tau_s': 0.}, {'process_noise_scale': float('nan')}, {'max_dt_s': True},
    {'minimum_valid_span_s': -1.}, {'measurement_covariance': [[1., 2.], [2., 1.]]},
])
def test_invalid_configuration_rejected(options):
    with pytest.raises(ValueError):
        ForceRateKalmanFilter(**options)
