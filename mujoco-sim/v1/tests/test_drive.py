"""The A6-EC cascade: ramp lag is v/Kp, rest is exact, a fault trips on the following-error window."""

import math

import numpy as np
from cellsim import drive


def _run(bank, inertia, seconds, ramp_rad_s, gravity_nm=0.0, dt=0.00025):
    q = np.zeros(1)
    qd = np.zeros(1)
    cpr = np.array([131072.0 * 100 / (2 * math.pi)])
    bank.enable(np.array([True]), q)
    t = 0.0
    lag = 0.0
    target = 0.0
    while t < seconds:
        if round(t / 0.001) != round((t - dt) / 0.001):
            target = ramp_rad_s * min(t, seconds - 1.0)
            bank.set_target(np.array([target]), 0.001)
        tau = bank.step(q, qd, cpr)
        qdd = (tau[0] - gravity_nm * math.sin(q[0])) / inertia
        qd = qd + qdd * dt
        q = q + qd * dt
        t += dt
        if 0.5 < t < seconds - 1.0:
            lag = max(lag, abs(target - q[0]))
    return q[0], target, lag, bank


def test_default_gains_track_a_ramp_with_the_p_loop_lag():
    bank = drive.DriveBank(np.array([100.0]), [drive.MOTOR_1000W], [drive.DriveGains()], 0.00025)
    inertia = 2.0 + 100.0 ** 2 * drive.MOTOR_1000W.rotor_inertia_kgm2
    q, target, lag, bank = _run(bank, inertia, 3.0, 0.2, gravity_nm=30.0)
    assert lag == 0.005 * 1.0 or abs(lag - 0.2 / 40.0) < 0.001      # v / Kp = 5 mrad
    assert abs(q - target) < 1e-4 and not bank.faulted[0]


def test_speed_feedforward_removes_the_ramp_lag():
    gains = drive.DriveGains(speed_feedforward_pct=100.0)
    bank = drive.DriveBank(np.array([100.0]), [drive.MOTOR_1000W], [gains], 0.00025)
    inertia = 2.0 + 100.0 ** 2 * drive.MOTOR_1000W.rotor_inertia_kgm2
    _, _, lag, _ = _run(bank, inertia, 3.0, 0.2)
    assert lag < 0.001


def test_following_error_window_faults_when_the_load_cannot_follow():
    gains = drive.DriveGains(following_error_window_counts=1000, following_error_timeout_s=0.05, torque_limit_pct=10.0)
    bank = drive.DriveBank(np.array([100.0]), [drive.MOTOR_400W], [gains], 0.00025)
    _, _, _, bank = _run(bank, 50.0, 2.0, 0.2)
    assert bank.faulted[0]


def test_csv_mode_bypasses_the_position_loop():
    bank = drive.DriveBank(np.array([100.0]), [drive.MOTOR_1000W], [drive.DriveGains()], 0.00025)
    inertia = 2.0 + 100.0 ** 2 * drive.MOTOR_1000W.rotor_inertia_kgm2
    q = np.zeros(1)
    qd = np.zeros(1)
    bank.enable(np.array([True]), q)
    cpr = np.array([131072.0 * 100 / (2 * math.pi)])
    for _ in range(8000):
        tau = bank.step(q, qd, cpr, csv_mask=np.array([True]), qd_target=np.array([0.05]))
        qd = qd + tau / inertia * 0.00025
        q = q + qd * 0.00025
    assert abs(qd[0] - 0.05) < 0.0005
