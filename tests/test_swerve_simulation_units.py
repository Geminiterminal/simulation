"""Regression checks for swerve simulation units, REVLib 2026 feedforward scaling,
and PathPlanner discretization/feedforward wiring.

In REVLib 2026 (2026.0.3), SparkSim.iterate(velocity, vbus, dt) and
SparkAbsoluteEncoderSim.iterate(velocity, dt) expect velocities in the encoder's
configured post-conversion units (m/s for drive, rad/s for steer), and
FeedForwardConfig.kV expects Volts per unit/s (12.0 / freeSpeed).
Run with: python3 -m unittest discover -s tests
"""

import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIGS_SOURCE = (ROOT / "src/main/java/frc/robot/Configs.java").read_text()
MODULE_SOURCE = (ROOT / "src/main/java/frc/robot/subsystems/MAXSwerveModule.java").read_text()
DRIVE_SOURCE = (ROOT / "src/main/java/frc/robot/subsystems/DriveSubsystem.java").read_text()
SHOOTER_SOURCE = (ROOT / "src/main/java/frc/robot/subsystems/Shooter.java").read_text()
CONSTANTS_SOURCE = (ROOT / "src/main/java/frc/robot/Constants.java").read_text()

WHEEL_DIAMETER_M = 0.0762
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0
DRIVE_GEAR_RATIO = (45.0 * 22.0) / (13.0 * 15.0)
NEO_FREE_SPEED_RPM = 5676.0
DRIVE_FREE_SPEED_MPS = (NEO_FREE_SPEED_RPM / 60.0) * (WHEEL_DIAMETER_M * math.pi) / DRIVE_GEAR_RATIO


class SwerveSimulationUnitsTest(unittest.TestCase):
    def test_drive_sim_supplies_converted_wheel_mps_to_revlib_2026(self):
        wheel_rad_per_sec = 7.5
        wheel_mps = wheel_rad_per_sec * WHEEL_RADIUS_M
        velocity_conversion_factor = WHEEL_DIAMETER_M * math.pi / DRIVE_GEAR_RATIO / 60.0
        # REVLib 2026 divides by velocityConversionFactor inside SparkSim::iterate,
        # then multiplies by velocityConversionFactor when reading getVelocity().
        internal_motor_rpm = wheel_mps / velocity_conversion_factor
        reported_mps = internal_motor_rpm * velocity_conversion_factor

        self.assertAlmostEqual(wheel_mps, reported_mps, places=12)
        self.assertIn(
            "m_drivingSparkSim.iterate(wheelSpeedMetersPerSec, busVoltage, dtSeconds);",
            MODULE_SOURCE,
        )
        self.assertNotIn("mechanismRadiansPerSecondToMotorRpm", MODULE_SOURCE)

    def test_steer_sim_uses_positive_plant_voltage_and_rad_per_sec_inputs(self):
        self.assertIn("DCMotor.getNeo550(1)", MODULE_SOURCE)
        self.assertIn(
            "m_turnPhysicsSim.setInputVoltage(m_turningSparkSim.getAppliedOutput() * busVoltage);",
            MODULE_SOURCE,
        )
        self.assertIn(
            "m_turningSparkSim.iterate(moduleTurnRateRadPerSec, busVoltage, dtSeconds);",
            MODULE_SOURCE,
        )
        self.assertIn(
            "m_turningEncoderSim.iterate(moduleTurnRateRadPerSec, dtSeconds);",
            MODULE_SOURCE,
        )

    def test_revlib_2026_velocity_feedforward_uses_12v_kv_for_neo_drive_and_shooter(self):
        expected_drive_kv = 12.0 / DRIVE_FREE_SPEED_MPS
        expected_shooter_kv = 12.0 / NEO_FREE_SPEED_RPM
        self.assertAlmostEqual(2.6902079, expected_drive_kv, places=6)
        self.assertAlmostEqual(0.00211416, expected_shooter_kv, places=6)

        self.assertRegex(
            CONFIGS_SOURCE,
            re.compile(r"12\.0\s*/\s*ModuleConstants\.kDriveWheelFreeSpeedRps"),
        )
        self.assertIn(".feedForward", CONFIGS_SOURCE)
        self.assertNotIn(".velocityFF(", CONFIGS_SOURCE)

        self.assertRegex(
            SHOOTER_SOURCE,
            re.compile(r"12\.0\s*/\s*NeoMotorConstants\.kFreeSpeedRpm"),
        )
        self.assertIn(".feedForward", SHOOTER_SOURCE)
        self.assertNotIn(".velocityFF(", SHOOTER_SOURCE)

    def test_neo_max_speed_and_pathplanner_discretization_and_feedforwards(self):
        self.assertIn("DCMotor.getNEO(1)", MODULE_SOURCE)
        self.assertIn(
            "kMaxSpeedMetersPerSecond = ModuleConstants.kDriveWheelFreeSpeedRps",
            CONSTANTS_SOURCE,
        )
        discretize = DRIVE_SOURCE.index("ChassisSpeeds.discretize(speeds")
        kinematics = DRIVE_SOURCE.index(
            "DriveConstants.kDriveKinematics.toSwerveModuleStates(discretizedSpeeds)"
        )
        feedforwards = DRIVE_SOURCE.index("feedforwards.robotRelativeForcesXNewtons()")
        self.assertLess(discretize, kinematics)
        self.assertLess(kinematics, feedforwards)


if __name__ == "__main__":
    unittest.main()
