"""One A6-EC servo drive in CSP mode, nine at a time.

This is the cascade the manual draws for CSP (A6-EC manual §4.1.6, figure
4-39): the host writes 607A every cycle; the drive runs a proportional
position loop that produces a speed reference, adds speed feedforward, runs a
PI speed loop with PDFF weighting on the reference, filters the torque
reference, clamps it to the torque limit, and the motor turns. The sim keeps
that structure and runs it in the joint frame with the gearbox reflected,
because that is where MuJoCo applies the torque.

Every gain default below is the manual's factory default (§11.3 group C01,
§7.4-7.8). The cell's drives were never read back into this repo -- the
a6ec.md notes say active C01.00/C01.01/C01.1B were still unproven on
2026-05-31 -- so treat all of them as UNVERIFIED for the real robot. They are
plain dataclass fields so a read-back can be dropped in per axis.

Units, so nothing is converted twice:
  - position loop:  C01.00 in 0.1 /s  -> `position_loop_gain_per_s`
  - speed loop:     C01.01 in 0.1 Hz  -> `speed_loop_gain_hz`; the drive turns
                    a bandwidth into a torque gain through the inertia it was
                    told about: Kv = 2*pi*f * J_rotor * (1 + C00.06/100)
  - integral time:  C01.02 in 0.01 ms -> `speed_loop_integral_s`
  - torque filter:  C01.03 in Hz
  - feedforward:    C01.14 / C01.17 in 0.1 % ; filters C01.15 / C01.18 in Hz
  - PDFF:           C01.1B in 0.1 %, 100 % = plain PI
  - torque limit:   60E0/60E1/6072 in 0.1 % of rated, default 300 %

Motor constants are NOT in the manual's parameter list; `MotorSpec` carries
representative values per motor class and each cell config says which class
sits on which axis. Also unverified.
"""

from __future__ import annotations

import dataclasses
import math
import sys

import numpy as np

TWO_PI = 2.0 * math.pi


@dataclasses.dataclass(frozen=True)
class MotorSpec:
    """Motor-side constants. Representative for the class, not the cell's nameplate."""
    name: str
    rated_torque_nm: float
    rotor_inertia_kgm2: float
    max_speed_rpm: float = 3000.0


# Representative StepperOnline A6-class servo motors (catalogue values of the
# 60/80/130 mm frames, 2026-08-29). Which frame sits on which joint of the
# cell is not recorded anywhere in the repo; the cell config picks.
MOTOR_400W = MotorSpec("400W", rated_torque_nm=1.27, rotor_inertia_kgm2=0.55e-4)
MOTOR_750W = MotorSpec("750W", rated_torque_nm=2.39, rotor_inertia_kgm2=1.6e-4)
MOTOR_1000W = MotorSpec("1kW", rated_torque_nm=4.77, rotor_inertia_kgm2=8.0e-4, max_speed_rpm=2500.0)
MOTOR_1500W = MotorSpec("1.5kW", rated_torque_nm=7.16, rotor_inertia_kgm2=12.0e-4, max_speed_rpm=2500.0)
MOTORS = {m.name: m for m in (MOTOR_400W, MOTOR_750W, MOTOR_1000W, MOTOR_1500W)}


@dataclasses.dataclass(frozen=True)
class DriveGains:
    """Group C01 as the manual ships it. All UNVERIFIED against the cell's drives."""
    position_loop_gain_per_s: float = 160.0       # 4x the C01.00=400 factory default: the soft 40/s let the arm droop ~0.08 rad under gravity at home, right at the playback completion gate, so replay never settled.
    speed_loop_gain_hz: float = 100.0             # 4x the C01.01=250 default, tracking the 4x position loop above: raising position gain alone pushed its ~25 Hz bandwidth up to the speed loop's, leaving the loop underdamped and overshooting each stop. Keeping the 4:1 speed:position ratio restores the damping.
    speed_loop_integral_s: float = 0.03184        # C01.02 = 3184 (0.01 ms); 512 ms = no integral
    torque_filter_hz: float = 200.0               # C01.03 = 200  (Hz)
    speed_feedforward_pct: float = 0.0            # C01.14 = 0    (0.1 %)
    speed_feedforward_filter_hz: float = 318.0    # C01.15 = 318  (Hz)
    torque_feedforward_pct: float = 0.0           # C01.17 = 0    (0.1 %)
    torque_feedforward_filter_hz: float = 100.0   # C01.18: default not tabulated in the text dump
    pdff_pct: float = 100.0                       # C01.1B = 1000 (0.1 %)
    inertia_ratio_pct: float = 100.0              # C00.06 = 100  (%)
    torque_limit_pct: float = 300.0               # 60E0/60E1/6072 = 3000 (0.1 %)
    # 6065/6066 default 0/0: the following-error alarm (Er47.0) is off unless
    # a window is configured. Counts are in the 607A frame (output shaft).
    following_error_window_counts: int = 0
    following_error_timeout_s: float = 0.0


def _alpha(cutoff_hz: float, dt_s: float) -> float:
    """First-order low-pass coefficient; 0 disables the filter (passes through)."""
    if cutoff_hz <= 0.0:
        return 1.0
    rc = 1.0 / (TWO_PI * cutoff_hz)
    return dt_s / (rc + dt_s)


class DriveBank:
    """The nine drives' state, vectorised across axes.

    Frame: everything is joint-side. A gear ratio N reflects motor torque as
    N*tau and motor speed as omega/N, so the motor-side gains become
    Kv_joint = N^2 * Kv_motor and the torque limit becomes N * limit. The
    integrator state is joint-side too.
    """

    def __init__(self, gear_ratio: np.ndarray, motors: list[MotorSpec],
                 gains: list[DriveGains], dt_s: float) -> None:
        n = len(gear_ratio)
        if not (len(motors) == len(gains) == n):
            raise ValueError("gear_ratio, motors and gains must have one entry per axis")
        self.n = n
        self.dt = float(dt_s)
        self.gear = np.asarray(gear_ratio, dtype=np.float64)
        self.gains = list(gains)
        self.motors = list(motors)
        g = gains
        self.kp = np.array([x.position_loop_gain_per_s for x in g])
        rotor = np.array([m.rotor_inertia_kgm2 for m in motors])
        j_assumed = rotor * (1.0 + np.array([x.inertia_ratio_pct for x in g]) / 100.0)
        # bandwidth -> torque gain at the motor, then reflected to the joint
        kv_motor = TWO_PI * np.array([x.speed_loop_gain_hz for x in g]) * j_assumed
        self.kv = kv_motor * self.gear ** 2
        ti = np.array([x.speed_loop_integral_s for x in g])
        self.ki = np.where(ti >= 0.512, 0.0, self.kv / np.maximum(ti, 1e-6))
        self.pdff = np.array([x.pdff_pct for x in g]) / 100.0
        self.ff_speed = np.array([x.speed_feedforward_pct for x in g]) / 100.0
        self.ff_torque = np.array([x.torque_feedforward_pct for x in g]) / 100.0
        self.a_torque = np.array([_alpha(x.torque_filter_hz, dt_s) for x in g])
        self.a_ff_speed = np.array([_alpha(x.speed_feedforward_filter_hz, dt_s) for x in g])
        self.a_ff_torque = np.array([_alpha(x.torque_feedforward_filter_hz, dt_s) for x in g])
        rated = np.array([m.rated_torque_nm for m in motors])
        self.torque_limit = rated * np.array([x.torque_limit_pct for x in g]) / 100.0 * self.gear
        self.max_speed = np.array([m.max_speed_rpm for m in motors]) * TWO_PI / 60.0 / self.gear
        # state
        self.integral = np.zeros(n)
        self.torque_filtered = np.zeros(n)
        self.ff_speed_filtered = np.zeros(n)
        self.ff_torque_filtered = np.zeros(n)
        self.prev_target = np.zeros(n)
        self.prev_target_rate = np.zeros(n)
        self.enabled = np.zeros(n, dtype=bool)
        self.following_error_s = np.zeros(n)
        self.faulted = np.zeros(n, dtype=bool)
        # a fresh target arrives once per CSP cycle; between cycles the
        # reference rate is what the last two targets implied (for feedforward)
        self.target = np.zeros(n)
        self.target_rate = np.zeros(n)

    # -- host side -----------------------------------------------------------

    def set_target(self, q_target: np.ndarray, cycle_s: float) -> None:
        """A new 607A for every axis, once per CSP cycle."""
        q_target = np.asarray(q_target, dtype=np.float64)
        self.target_rate = (q_target - self.target) / cycle_s
        self.target = q_target.copy()

    def enable(self, mask: np.ndarray, q_now: np.ndarray) -> None:
        """Operation Enabled on `mask`: integrators cleared, target aligned to actual."""
        newly = mask & ~self.enabled
        self.integral[newly] = 0.0
        self.torque_filtered[newly] = 0.0
        self.target[newly] = q_now[newly]
        self.target_rate[newly] = 0.0
        self.enabled = self.enabled | mask

    def disable(self, mask: np.ndarray) -> None:
        self.enabled = self.enabled & ~mask
        self.integral[mask] = 0.0
        self.torque_filtered[mask] = 0.0

    # -- one physics step ----------------------------------------------------

    def step(self, q: np.ndarray, qd: np.ndarray, counts_per_rad: np.ndarray,
             csv_mask: np.ndarray | None = None, qd_target: np.ndarray | None = None) -> np.ndarray:
        """Joint torque for this physics step (0 where the drive is not enabled).

        `csv_mask` axes are in Cyclic Synchronous Velocity (mode 9): the host's
        0x60FF goes straight to the speed loop and the position loop is idle,
        which is how RTCore drives a single-axis joint jog.
        """
        err = self.target - q
        # position loop + speed feedforward (C01.14 through C01.15)
        self.ff_speed_filtered += self.a_ff_speed * (self.target_rate - self.ff_speed_filtered)
        v_ref = self.kp * err + self.ff_speed * self.ff_speed_filtered
        if csv_mask is not None and qd_target is not None:
            v_ref = np.where(csv_mask, qd_target, v_ref)
            err = np.where(csv_mask, 0.0, err)
        v_ref = np.clip(v_ref, -self.max_speed, self.max_speed)
        # speed loop, PDFF: proportional on (alpha*ref - fb), integral on the error
        v_err = v_ref - qd
        p = self.kv * (self.pdff * v_ref - qd)
        tau_raw = p + self.ki * self.integral
        # torque feedforward from the reference acceleration (C01.17/18)
        accel_ref = (self.target_rate - self.prev_target_rate) / self.dt
        self.prev_target_rate = self.target_rate.copy()
        self.ff_torque_filtered += self.a_ff_torque * (accel_ref - self.ff_torque_filtered)
        tau_raw = tau_raw + self.ff_torque * self.kv * self.dt * self.ff_torque_filtered
        # torque filter and limit; anti-windup: integrate only when not saturated
        self.torque_filtered += self.a_torque * (tau_raw - self.torque_filtered)
        tau = np.clip(self.torque_filtered, -self.torque_limit, self.torque_limit)
        saturated = (tau != self.torque_filtered) & (np.sign(v_err) == np.sign(tau))
        self.integral += np.where(saturated, 0.0, v_err * self.dt)
        tau = np.where(self.enabled, tau, 0.0)
        # Er47.0 following error, if a window is configured
        window = np.array([g.following_error_window_counts for g in self.gains])
        timeout = np.array([g.following_error_timeout_s for g in self.gains])
        over = (window > 0) & (np.abs(err) * counts_per_rad > window) & self.enabled
        self.following_error_s = np.where(over, self.following_error_s + self.dt, 0.0)
        self.faulted |= over & (self.following_error_s > timeout)
        return tau


def self_test() -> int:
    """A single 100:1 axis follows a 0.2 rad/s ramp with a bounded lag and no overshoot at rest."""
    dt = 0.00025
    bank = DriveBank(np.array([100.0]), [MOTOR_1000W], [DriveGains()], dt)
    inertia = 2.0 + 100.0 ** 2 * MOTOR_1000W.rotor_inertia_kgm2  # link + reflected rotor, kg m^2
    q = np.zeros(1)
    qd = np.zeros(1)
    bank.enable(np.array([True]), q)
    cpr = np.array([131072.0 * 100.0 / TWO_PI])
    t = 0.0
    target = 0.0
    worst_lag = 0.0
    while t < 3.0:
        if round(t / 0.001) != round((t - dt) / 0.001):
            target = 0.2 * min(t, 2.0)      # ramp for 2 s, then hold
            bank.set_target(np.array([target]), 0.001)
        tau = bank.step(q, qd, cpr)
        gravity = 30.0 * math.sin(q[0])  # a hanging load, so the integrator has to work
        qdd = (tau[0] - gravity) / inertia
        qd = qd + qdd * dt
        q = q + qd * dt
        t += dt
        if 0.5 < t < 2.0:
            worst_lag = max(worst_lag, abs(target - q[0]))
    settle = abs(0.4 - q[0])
    # steady-state lag on a ramp = v / Kp = 0.2 / 40 = 5 mrad (position P loop);
    # 20 % margin for the speed loop's own lag
    ok = worst_lag < 0.006 and settle < 1e-4 and not bank.faulted[0]
    print(f"drive self-test: ramp lag {worst_lag * 1e3:.2f} mrad (P-loop predicts 5.0), "
          f"settle error {settle * 1e6:.1f} urad, faulted={bool(bank.faulted[0])} -> {'ok' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(self_test())
