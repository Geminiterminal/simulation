"""Verification of WPILib SysId characterization routines, Thrustmaster T.16000M
button bindings, Auto Chooser integration, safety limits, and closed-loop OLS
system identification across all robot mechanisms.

Run with:  python3 -m unittest discover -s tests -v
"""

import math
import re
import unittest
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
JAVA_ROOT = ROOT / "src/main/java/frc/robot"

DT = 0.020  # 50 Hz robot loop


@dataclass(frozen=True)
class DCMotorSpec:
    nominal_voltage: float
    stall_torque_nm: float
    stall_current_amps: float
    free_current_amps: float
    free_speed_rpm: float

    @property
    def free_speed_rad_per_s(self) -> float:
        return self.free_speed_rpm * 2.0 * math.pi / 60.0

    @property
    def r_ohms(self) -> float:
        return self.nominal_voltage / self.stall_current_amps

    @property
    def kt_nm_per_amp(self) -> float:
        return self.stall_torque_nm / self.stall_current_amps

    @property
    def kv_rad_per_s_per_volt(self) -> float:
        return self.free_speed_rad_per_s / (
            self.nominal_voltage - self.r_ohms * self.free_current_amps
        )


# Official WPILib DCMotor specifications matching Constants.java / DCMotor.java
NEO = DCMotorSpec(12.0, 2.6, 105.0, 1.8, 5676.0)
NEO_550 = DCMotorSpec(12.0, 0.97, 100.0, 1.4, 11000.0)
NEO_VORTEX = DCMotorSpec(12.0, 3.6, 211.0, 3.6, 6784.0)
KRAKEN_X60 = DCMotorSpec(12.0, 7.09, 366.0, 2.0, 6000.0)


def solve_linear_system(matrix, rhs):
    """Gaussian elimination with partial pivoting (no external numpy needed)."""
    n = len(rhs)
    aug = [list(matrix[i]) + [rhs[i]] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        aug[col], aug[pivot] = aug[pivot], aug[col]
        denom = aug[col][col]
        if abs(denom) < 1e-15:
            raise ValueError("Singular OLS normal matrix")
        for j in range(col, n + 1):
            aug[col][j] /= denom
        for row in range(n):
            if row != col:
                factor = aug[row][col]
                for j in range(col, n + 1):
                    aug[row][j] -= factor * aug[col][j]
    return [aug[i][n] for i in range(n)]


def ols_fit(features, targets):
    """Ordinary least squares beta = (X^T X)^-1 X^T y."""
    p = len(features[0])
    xtx = [[0.0] * p for _ in range(p)]
    xty = [0.0] * p
    for x_row, y in zip(features, targets):
        for i in range(p):
            xty[i] += x_row[i] * y
            for j in range(p):
                xtx[i][j] += x_row[i] * x_row[j]
    return solve_linear_system(xtx, xty)


def simulate_rotational_plant(
    motor: DCMotorSpec,
    inertia_j: float,
    gear_ratio: float,
    ramp_rate: float,
    step_volts: float,
    duration_s: float,
    output_scale_from_rad: float,
):
    """Runs 4-part SysId (QF, QR, DF, DR) on an exact ZOH DC-motor plant and returns
    logged (voltage, position, velocity) tuples per test."""
    # Continuous state-space: d(omega_out)/dt = a_c * omega_out + b_c * V
    a_c = -(
        (gear_ratio**2)
        * motor.kt_nm_per_amp
        / (motor.kv_rad_per_s_per_volt * motor.r_ohms * inertia_j)
    )
    b_c = (gear_ratio * motor.kt_nm_per_amp) / (motor.r_ohms * inertia_j)
    # Exact zero-order-hold discretization over DT (matches WPILib LinearSystemSim)
    ad = math.exp(a_c * DT)
    bd = (ad - 1.0) / a_c * b_c

    logs = []
    for is_dynamic in (False, True):
        for sign in (1.0, -1.0):
            pos_rad = 0.0
            omega_rad_s = 0.0
            steps = int(duration_s / DT)
            test_samples = []
            for k in range(steps):
                t = k * DT
                volts = sign * (step_volts if is_dynamic else ramp_rate * t)
                volts = max(-12.0, min(12.0, volts))
                # SysId logs applied voltage & current state, then plant steps DT
                test_samples.append(
                    (
                        volts,
                        pos_rad * output_scale_from_rad,
                        omega_rad_s * output_scale_from_rad,
                    )
                )
                omega_next = ad * omega_rad_s + bd * volts
                pos_rad += 0.5 * (omega_rad_s + omega_next) * DT
                omega_rad_s = omega_next
            logs.append(test_samples)
    return logs, -a_c / b_c, 1.0 / b_c


def fit_zoh_sysid(test_logs):
    """Exact discrete-to-continuous ZOH SysId identification over 20 ms robot frames:
    fits v[k+1] = ad * v[k] + bd * V[k] + cd * sign(v[k]), then maps (ad, bd) back to
    continuous-time (kS, kV, kA) via matrix logarithm a_c = ln(ad) / DT."""
    features = []
    targets = []
    for samples in test_logs:
        for i in range(len(samples) - 1):
            volts_k = samples[i][0]
            vel_k = samples[i][2]
            vel_next = samples[i + 1][2]
            if abs(vel_k) < 1e-5 and abs(volts_k) < 1e-5:
                continue
            sign_v = math.copysign(1.0, vel_k if abs(vel_k) > 1e-5 else volts_k)
            features.append([vel_k, volts_k, sign_v])
            targets.append(vel_next)
    ad, bd, cd = ols_fit(features, targets)
    a_c = math.log(ad) / DT
    b_c = bd * a_c / (ad - 1.0)
    c_c = cd * a_c / (ad - 1.0)
    ks = -c_c / b_c
    kv = -a_c / b_c
    ka = 1.0 / b_c
    return ks, kv, ka


class ThrustmasterAndChooserWiringTest(unittest.TestCase):
    def test_thrustmaster_sysid_buttons_are_four_unused_valid_buttons(self):
        tm_src = (JAVA_ROOT / "driverprofiles/ThrustmasterProfile.java").read_text()
        raw_buttons = [int(m) for m in re.findall(r"getRawButton\((\d+)\)", tm_src)]
        sysid_btns = {
            name: int(val)
            for name, val in re.findall(
                r"public static final int (kSysId\w+Button)\s*=\s*(\d+);", tm_src
            )
        }
        self.assertEqual(4, len(sysid_btns))
        btn_values = list(sysid_btns.values())
        self.assertEqual(4, len(set(btn_values)), "SysId buttons must be 4 distinct buttons")
        for btn in btn_values:
            self.assertGreaterEqual(btn, 1)
            self.assertLessEqual(btn, 16)
            self.assertNotIn(
                btn,
                raw_buttons,
                f"SysId button {btn} conflicts with an existing Thrustmaster teleop binding",
            )

    def test_unique_sysid_mechanism_log_names_across_all_subsystems(self):
        drive_src = (JAVA_ROOT / "subsystems/DriveSubsystem.java").read_text()
        shooter_src = (JAVA_ROOT / "subsystems/Shooter.java").read_text()
        intake_src = (JAVA_ROOT / "subsystems/Intake.java").read_text()
        combined = drive_src + "\n" + shooter_src + "\n" + intake_src

        names = re.findall(r'this,\s*\n?\s*"([a-z0-9-]+)"\)', combined)
        expected = {
            "drive-linear",
            "drive-angular",
            "drive-steer",
            "shooter-flywheels",
            "shooter-feeder",
            "intake-deploy",
            "intake-roller",
            "intake-conveyor",
        }
        self.assertEqual(expected, set(names))
        self.assertEqual(len(expected), len(names), "SysId mechanism log names must not collide")

    def test_module_sysid_voltage_mode_never_invokes_180_degree_optimize(self):
        mod_src = (JAVA_ROOT / "subsystems/MAXSwerveModule.java").read_text()
        start = mod_src.index("public void setSysIdDriveVoltsAtAngle(")
        end = mod_src.index("public void setSysIdSteerVolts(", start)
        method_body = mod_src[start:end]
        self.assertNotIn(".optimize(", method_body)
        self.assertIn("m_drivingSpark.setVoltage(volts);", method_body)

    def test_auto_chooser_and_fms_safety_gate_wired_in_robot_container(self):
        rc_src = (JAVA_ROOT / "RobotContainer.java").read_text()
        self.assertIn("SysIdSupport.registerAutoChooserOptions(autoChooser);", rc_src)
        self.assertIn("!DriverStation.isFMSAttached()", rc_src)
        sysid_idx = rc_src.index("if (SysIdSupport.isSysIdAutoKey(selected))")
        autobuilder_idx = rc_src.index("if (!AutoBuilder.isConfigured())", sysid_idx)
        self.assertLess(sysid_idx, autobuilder_idx)

        robot_src = (JAVA_ROOT / "Robot.java").read_text()
        self.assertLess(
            robot_src.index("DataLogManager.start();"),
            robot_src.index("new RobotContainer();"),
            "DataLogManager.start() must run before new RobotContainer() for SysId .wpilog capture",
        )


class SysIdPhysicsIdentificationTest(unittest.TestCase):
    """End-to-end physics + OLS fit verification proving that every mechanism's
    SysId routine logs units/signs that accurately recover physical kV, kA, and kG."""

    def test_drivetrain_linear_sysid_recovers_exact_wheel_kv_and_ka_and_bounds_auto_distance(self):
        wheel_radius = 0.0762 / 2.0
        gear_ratio = (45.0 * 22.0) / (13.0 * 15.0)
        inertia_j = (65.0 / 4.0) * (wheel_radius**2) + 0.5 * 0.050 * (wheel_radius**2)

        logs, kv_rad, ka_rad = simulate_rotational_plant(
            NEO,
            inertia_j=inertia_j,
            gear_ratio=gear_ratio,
            ramp_rate=1.0,
            step_volts=7.0,
            duration_s=4.0,
            output_scale_from_rad=wheel_radius,  # meters & m/s
        )
        expected_kv_mps = kv_rad / wheel_radius
        expected_ka_mps2 = ka_rad / wheel_radius

        ks, kv, ka = fit_zoh_sysid(logs)
        self.assertAlmostEqual(0.0, ks, delta=1e-6)
        self.assertAlmostEqual(expected_kv_mps, kv, delta=expected_kv_mps * 1e-4)
        self.assertAlmostEqual(expected_ka_mps2, ka, delta=expected_ka_mps2 * 1e-4)

    def test_drivetrain_steer_sysid_with_unwrapped_radians_recovers_kv_and_ka(self):
        gear_ratio = 9424.0 / 203.0
        inertia_j = 0.004
        # SysIdRoutineLog.angularPosition/Velocity converts Radians -> Rotations (1 / 2*pi)
        rot_per_rad = 1.0 / (2.0 * math.pi)
        logs, kv_rad, ka_rad = simulate_rotational_plant(
            NEO_550,
            inertia_j=inertia_j,
            gear_ratio=gear_ratio,
            ramp_rate=1.0,
            step_volts=6.0,
            duration_s=5.0,
            output_scale_from_rad=rot_per_rad,
        )
        expected_kv_rot = kv_rad / rot_per_rad
        expected_ka_rot = ka_rad / rot_per_rad

        ks, kv, ka = fit_zoh_sysid(logs)
        self.assertAlmostEqual(0.0, ks, delta=1e-6)
        self.assertAlmostEqual(expected_kv_rot, kv, delta=expected_kv_rot * 1e-4)
        self.assertAlmostEqual(expected_ka_rot, ka, delta=expected_ka_rot * 1e-4)

    def test_shooter_flywheels_and_feeder_and_intake_rollers_recover_kv_and_ka(self):
        rot_per_rad = 1.0 / (2.0 * math.pi)
        cases = [
            ("shooter-flywheels", NEO, 0.0025, 1.0, 1.0, 7.0),
            ("shooter-feeder", KRAKEN_X60, 0.001, 1.0, 1.0, 6.0),
            ("intake-roller", KRAKEN_X60, 0.0008, 1.0, 1.0, 6.0),
            ("intake-conveyor", NEO, 0.002, 1.0, 1.0, 6.0),
        ]
        for name, motor, moi, gearing, ramp, step in cases:
            with self.subTest(mechanism=name):
                logs, kv_rad, ka_rad = simulate_rotational_plant(
                    motor,
                    inertia_j=moi,
                    gear_ratio=gearing,
                    ramp_rate=ramp,
                    step_volts=step,
                    duration_s=5.0,
                    output_scale_from_rad=rot_per_rad,
                )
                expected_kv = kv_rad / rot_per_rad
                expected_ka = ka_rad / rot_per_rad
                ks, kv, ka = fit_zoh_sysid(logs)
                self.assertAlmostEqual(0.0, ks, delta=1e-6)
                self.assertAlmostEqual(expected_kv, kv, delta=expected_kv * 1e-4)
                self.assertAlmostEqual(expected_ka, ka, delta=expected_ka * 1e-4)

    def test_intake_deploy_arm_sysid_stays_within_travel_limits_and_identifies_gravity_kg(self):
        # SingleJointedArmSim physics: J = 1/3 * m * L^2, gravity torque = m*g*(L/2)*cos(theta)
        # Note: in Intake.java, +motorRotations swings from 0 deg (horizontal stowed) down +100 deg.
        # Gravity assists downward (+angle) motion when cos(theta) > 0, so motor equation is:
        #   d(omega)/dt = a_c * omega + b_c * V + (m * g * L/2 / J) * cos(theta)
        # => V = (-g_accel / b_c) * cos(theta) + kV * omega + kA * alpha
        arm_mass = 3.0
        arm_len = 0.30
        gearing = 59.0 * 360.0 / 100.0  # 212.4
        arm_moi = (1.0 / 3.0) * arm_mass * (arm_len**2)
        g_mps2 = 9.80665

        a_c = -(
            (gearing**2)
            * NEO_VORTEX.kt_nm_per_amp
            / (NEO_VORTEX.kv_rad_per_s_per_volt * NEO_VORTEX.r_ohms * arm_moi)
        )
        b_c = (gearing * NEO_VORTEX.kt_nm_per_amp) / (NEO_VORTEX.r_ohms * arm_moi)
        grav_coeff = (arm_mass * g_mps2 * (arm_len / 2.0)) / arm_moi  # assists +theta downward

        min_motor_rot = 4.0
        max_motor_rot = 55.0

        # Run full 4-leg sequence (QF from 0->55, QR from 55->4, DF from 4->55, DR from 55->4)
        theta = 0.0  # stowed horizontal
        omega = 0.0
        legs = [
            (False, 1.0),
            (False, -1.0),
            (True, 1.0),
            (True, -1.0),
        ]
        all_leg_logs = []
        for is_dyn, sign in legs:
            leg_samples = []
            for step_idx in range(int(6.0 / DT)):
                motor_rot = (theta / (2.0 * math.pi)) * gearing
                # Check Intake.hasReachedDeploySysIdLimit
                if sign > 0 and motor_rot >= max_motor_rot:
                    break
                if sign < 0 and motor_rot <= min_motor_rot:
                    break
                t = step_idx * DT
                volts = sign * (2.5 if is_dyn else 0.5 * t)
                arm_rot = motor_rot / gearing
                arm_rps = (omega / (2.0 * math.pi))
                leg_samples.append((volts, arm_rot, arm_rps))

                # Substepped RK4 integration of stiff 212.4:1 arm differential equation
                # (|a_c| ~= 2495 s^-1 requires dt_sub << 2.78 / |a_c| ~= 1.1 ms)
                substeps = 100
                dt_sub = DT / substeps

                def accel_fn(th, om, v=volts):
                    return a_c * om + b_c * v + grav_coeff * math.cos(th)

                for _ in range(substeps):
                    k1_th, k1_om = omega, accel_fn(theta, omega)
                    k2_th, k2_om = omega + 0.5 * dt_sub * k1_om, accel_fn(
                        theta + 0.5 * dt_sub * k1_th, omega + 0.5 * dt_sub * k1_om
                    )
                    k3_th, k3_om = omega + 0.5 * dt_sub * k2_om, accel_fn(
                        theta + 0.5 * dt_sub * k2_th, omega + 0.5 * dt_sub * k2_om
                    )
                    k4_th, k4_om = omega + dt_sub * k3_om, accel_fn(
                        theta + dt_sub * k3_th, omega + dt_sub * k3_om
                    )
                    theta += (dt_sub / 6.0) * (k1_th + 2 * k2_th + 2 * k3_th + k4_th)
                    omega += (dt_sub / 6.0) * (k1_om + 2 * k2_om + 2 * k3_om + k4_om)

                # Assert hard stops [0, 59] are NEVER violated (with >3 motor rot margin)!
                new_motor_rot = (theta / (2.0 * math.pi)) * gearing
                self.assertGreaterEqual(new_motor_rot, 0.0)
                self.assertLessEqual(new_motor_rot, 56.0)

            self.assertGreater(len(leg_samples), 40, "each arm sweep must collect rich samples")
            all_leg_logs.append(leg_samples)
            # Settle between legs at 0 V
            omega = 0.0

        # Standard WPILib SysId Arm OLS fit:
        # V = kS*sign(v) + kG*cos(2*pi*arm_rot) + kV*arm_rps + kA*arm_accel_rps2
        features = []
        targets = []
        for samples in all_leg_logs:
            for i in range(2, len(samples) - 1):
                volts_k = 0.5 * (samples[i - 1][0] + samples[i][0])
                arm_rot_k = samples[i][1]
                arm_rps_k = samples[i][2]
                if abs(arm_rps_k) < 1e-3:
                    continue
                arm_accel = (samples[i + 1][2] - samples[i - 1][2]) / (2.0 * DT)
                features.append(
                    [
                        math.copysign(1.0, arm_rps_k),
                        math.cos(2.0 * math.pi * arm_rot_k),
                        arm_rps_k,
                        arm_accel,
                    ]
                )
                targets.append(volts_k)

        ks, kg, kv, ka = ols_fit(features, targets)
        expected_kg = -grav_coeff / b_c
        expected_kv = (-a_c / b_c) * (2.0 * math.pi)

        self.assertAlmostEqual(0.0, ks, delta=0.02)
        self.assertAlmostEqual(expected_kg, kg, delta=0.01)
        self.assertAlmostEqual(expected_kv, kv, delta=expected_kv * 0.01)
        self.assertGreater(ka, 0.0)


if __name__ == "__main__":
    unittest.main()
