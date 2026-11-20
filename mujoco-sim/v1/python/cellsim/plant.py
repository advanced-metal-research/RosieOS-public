"""The drives and the mechanism: MuJoCo behind RTCore's `Plant` protocol.

Nine A6-EC drives (`drive.DriveBank`) sit between RTCore's per-cycle
targets and MuJoCo's joint torques; the encoders are read back in the
drive's own rotary frame (`0..RM-1` counts per joint revolution, home at
the frame midpoint) so what leaves in `ServoFeedbackV4` is what the real
6064 would say.

Brakes: a disabled drive is a joint held by its holding brake, idealised
here as a frozen joint (velocity zeroed, position pinned each step) rather
than a torque -- the cell's brake torque is not recorded anywhere and the
point is that a disarmed arm does not fall.
"""

from __future__ import annotations

import dataclasses
import math
import pathlib

import mujoco
import numpy as np

from cellsim import drive, model
from cellsim.units import COUNTS_PER_REV, LIVE_AXES

# Which motor class sits on which axis of the cell. Not recorded in the
# repo; the arm's J1-J3 need the most torque through 100:1, the wrist and
# positioner less. UNVERIFIED -- see `drive.MOTORS`.
DEFAULT_MOTOR_CLASSES = ("1.5kW", "1.5kW", "1kW", "750W", "750W", "400W", "1kW", "1kW", "1kW")


@dataclasses.dataclass(frozen=True)
class CellConfig:
    urdf_path: pathlib.Path
    density_kg_m3: float = model.DEFAULT_DENSITY_KG_M3
    motor_classes: tuple[str, ...] = DEFAULT_MOTOR_CLASSES
    gains: tuple[drive.DriveGains, ...] = tuple(drive.DriveGains() for _ in range(LIVE_AXES))
    physics_dt_s: float = model.PHYSICS_DT_S
    cycle_s: float = 0.001
    # a viscous term standing in for gearbox friction; cycle-to-cycle the
    # drives hide it, so it mostly damps the free positioner tables. Unverified.
    joint_damping_nms: float = 2.0
    # C00.06 per axis from the model rather than the manual's 100 %: a
    # commissioned A6-EC has its load inertia ratio auto-tuned, and the speed
    # loop's torque gain scales with it (drive.py). Off = the manual default,
    # which under-drives every joint carrying more than one rotor inertia:
    # with it, J5 sagged 42 mrad at brake release on the e2e run of
    # 2026-08-29; with it on, 0.4 mrad.
    auto_inertia_ratio: bool = True


class MujocoPlant:
    """Expose drive targets and observations without opening a controller transport."""

    def __init__(self, config: CellConfig, urdf_joints: tuple[str, ...], signs: np.ndarray,
                 gear_ratio: np.ndarray, cache_dir: pathlib.Path) -> None:
        self.config = config
        n = LIVE_AXES
        self.n = n
        self.signs = np.asarray(signs, dtype=np.float64)
        self.gear = np.asarray(gear_ratio, dtype=np.float64)
        motors = [drive.MOTORS[name] for name in config.motor_classes]
        armature = {joint: (self.gear[i] ** 2) * motors[i].rotor_inertia_kgm2
                    for i, joint in enumerate(urdf_joints) if joint}
        self.cell = model.build(config.urdf_path, cache_dir, density_kg_m3=config.density_kg_m3,
                                armature_kgm2=armature, joint_damping_nms=config.joint_damping_nms,
                                timestep_s=config.physics_dt_s)
        self.model = mujoco.MjModel.from_xml_string(self.cell.xml)
        self.data = mujoco.MjData(self.model)
        # axis -> MuJoCo joint / actuator index (-1 for an axis the URDF lacks)
        self.jnt = np.full(n, -1, dtype=np.int64)
        self.act = np.full(n, -1, dtype=np.int64)
        for axis, joint in enumerate(urdf_joints[:n]):
            if not joint:
                continue
            j = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            a = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint)
            if j < 0 or a < 0:
                raise model.ModelError(f"axis {axis}: joint {joint!r} is not in {config.urdf_path}")
            self.jnt[axis] = j
            self.act[axis] = a
        self.present = self.jnt >= 0
        self.qadr = np.array([self.model.jnt_qposadr[j] if j >= 0 else 0 for j in self.jnt])
        self.vadr = np.array([self.model.jnt_dofadr[j] if j >= 0 else 0 for j in self.jnt])
        gains = list(config.gains)
        self.inertia_ratio_pct = np.array([g.inertia_ratio_pct for g in gains])
        if config.auto_inertia_ratio:
            self.inertia_ratio_pct = self.load_inertia_ratio_pct(motors)
            gains = [dataclasses.replace(g, inertia_ratio_pct=float(r)) for g, r in zip(gains, self.inertia_ratio_pct)]
        self.drives = drive.DriveBank(self.gear, motors, gains, config.physics_dt_s)
        self.substeps = max(1, round(config.cycle_s / config.physics_dt_s))
        self.enabled = np.zeros(n, dtype=bool)
        self.held_q = np.zeros(n)
        self.csp_mask = np.zeros(n, dtype=bool)
        self.csv_mask = np.zeros(n, dtype=bool)
        self.q_target = np.zeros(n)
        self.qd_target = np.zeros(n)
        self.torque = np.zeros(n)
        mujoco.mj_forward(self.model, self.data)
        self.held_q = self.q().copy()
        self.sim_time_s = 0.0

    def load_inertia_ratio_pct(self, motors: list[drive.MotorSpec]) -> np.ndarray:
        """What the drive's auto-tune would find: load inertia over rotor inertia, motor side.

        The joint-space mass matrix diagonal at the current pose, less the
        reflected rotor (`armature`), divided by N^2 is the load the motor
        sees; the real cell's tuning was done at some pose too, and which one
        is not recorded, so this is the URDF zero pose. Clamped to the
        parameter's range (0..12000 %).
        """
        mujoco.mj_forward(self.model, self.data)
        full = np.zeros((self.model.nv, self.model.nv))
        try:
            mujoco.mj_fullM(self.model, self.data, full)          # MuJoCo >= 3.4 takes MjData
        except TypeError:
            mujoco.mj_fullM(self.model, full, self.data.qM)      # older bindings take the sparse qM
        ratio = np.zeros(self.n)
        for axis in range(self.n):
            if not self.present[axis]:
                ratio[axis] = 100.0
                continue
            dof = self.vadr[axis]
            armature = self.model.dof_armature[dof]
            load_joint = max(0.0, full[dof, dof] - armature)
            load_motor = load_joint / (self.gear[axis] ** 2)
            ratio[axis] = min(12000.0, 100.0 * load_motor / motors[axis].rotor_inertia_kgm2)
        return ratio

    # ---- frames -------------------------------------------------------------

    def counts_per_rad(self) -> np.ndarray:
        return COUNTS_PER_REV * self.gear / (2 * math.pi)

    def frame_counts(self) -> np.ndarray:
        return np.round(COUNTS_PER_REV * self.gear)

    def q(self) -> np.ndarray:
        out = np.zeros(self.n)
        out[self.present] = self.data.qpos[self.qadr[self.present]]
        return out

    def qd(self) -> np.ndarray:
        out = np.zeros(self.n)
        out[self.present] = self.data.qvel[self.vadr[self.present]]
        return out

    def set_q(self, q: np.ndarray) -> None:
        self.data.qpos[self.qadr[self.present]] = q[self.present]
        self.data.qvel[self.vadr[self.present]] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.held_q = self.q().copy()

    def pos_counts(self) -> np.ndarray:
        """6064 per axis: joint zero at the frame midpoint, wrapped into 0..RM-1."""
        frame = self.frame_counts()
        raw = np.floor(frame * 0.5) + self.q() * self.counts_per_rad() * self.signs
        wrapped = np.mod(np.round(raw), np.maximum(frame, 1.0))
        return wrapped.astype(np.int64)

    def counts_to_q(self, counts: np.ndarray, near_q: np.ndarray) -> np.ndarray:
        """A 607A target, taken in the joint turn nearest the current position."""
        frame = self.frame_counts()
        here = np.floor(frame * 0.5) + near_q * self.counts_per_rad() * self.signs
        delta = counts - here
        delta -= np.round(delta / frame) * frame
        return near_q + delta / (self.counts_per_rad() * self.signs)

    # ---- plant interface -------------------------------------------------------

    def set_csp_targets(self, target_counts: np.ndarray, csp_mask: np.ndarray) -> None:
        self.csp_mask = np.asarray(csp_mask, dtype=bool) & self.present
        self.q_target = np.where(self.csp_mask, self.counts_to_q(np.asarray(target_counts, dtype=np.float64),
                                                                 self.q()), self.q())

    def set_csv_targets(self, target_counts_per_s: np.ndarray, csv_mask: np.ndarray) -> None:
        self.csv_mask = np.asarray(csv_mask, dtype=bool) & self.present
        self.qd_target = np.where(self.csv_mask,
                                  np.asarray(target_counts_per_s, dtype=np.float64)
                                  / (self.counts_per_rad() * self.signs), 0.0)

    def set_enabled(self, enabled: np.ndarray) -> None:
        enabled = np.asarray(enabled, dtype=bool) & self.present
        q = self.q()
        newly_on = enabled & ~self.enabled
        newly_off = ~enabled & self.enabled
        self.drives.enable(newly_on, q)
        self.drives.disable(newly_off)
        self.held_q[newly_off] = q[newly_off]
        self.enabled = enabled

    def fault_mask(self) -> int:
        return int(sum(1 << i for i in range(self.n) if self.drives.faulted[i]))

    def torque_actual_permille(self) -> np.ndarray:
        rated = np.array([m.rated_torque_nm for m in self.drives.motors]) * self.gear
        return np.round(1000.0 * self.torque / np.maximum(rated, 1e-9)).astype(np.int64)

    # ---- one 1 ms cycle -----------------------------------------------------

    def step_cycle(self) -> None:
        """Run the CSP cycle: the drives hold the new 607A for `substeps` physics steps."""
        # the position loop's reference for this cycle (ZOH; C01.20 average filter is off by default)
        self.drives.set_target(np.where(self.enabled, self.q_target, self.q()), self.config.cycle_s)
        cpr = self.counts_per_rad()
        for _ in range(self.substeps):
            q = self.q()
            qd = self.qd()
            tau = self.drives.step(q, qd, cpr, csv_mask=self.csv_mask, qd_target=self.qd_target)
            tau = np.where(self.enabled, tau, 0.0)
            self.torque = tau
            for axis in range(self.n):
                if self.present[axis]:
                    self.data.ctrl[self.act[axis]] = tau[axis]
            mujoco.mj_step(self.model, self.data)
            # holding brakes on the disabled axes
            braked = self.present & ~self.enabled
            if braked.any():
                self.data.qpos[self.qadr[braked]] = self.held_q[braked]
                self.data.qvel[self.vadr[braked]] = 0.0
        self.sim_time_s += self.config.cycle_s
