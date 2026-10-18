"""Regression checks for swerve simulation units and path discretization.

REVLib's SparkSim receives motor RPM, while the drivetrain odometry consumes
converted wheel m/s and module radians. These tests pin the conversion contract
and verify that the production Java simulation feeds the REV simulators in the
right places. Run with: python3 -m unittest discover -s tests
"""

import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_SOURCE = (ROOT / "src/main/java/frc/robot/subsystems/MAXSwerveModule.java").read_text()
DRIVE_SOURCE = (ROOT / "src/main/java/frc/robot/subsystems/DriveSubsystem.java").read_text()

WHEEL_DIAMETER_M = 0.0762
DRIVE_GEAR_RATIO = (45.0 * 22.0) / (13.0 * 15.0)
TURN_GEAR_RATIO = 9424.0 / 203.0


def radians_per_second_to_rpm(rate_rad_per_second):
    return rate_rad_per_second * 60.0 / (2.0 * math.pi)


def mechanism_rate_to_motor_rpm(rate_rad_per_second, signed_gear_ratio):
    return radians_per_second_to_rpm(rate_rad_per_second * signed_gear_ratio)


class SwerveSimulationUnitsTest(unittest.TestCase):
    def test_drive_sim_supplies_motor_rpm_and_encoder_reports_wheel_mps(self):
        rate = 7.5  # wheel rad/s from DCMotorSim's mechanism-side plant
        motor_rpm = mechanism_rate_to_motor_rpm(rate, DRIVE_GEAR_RATIO)
        encoder_mps = motor_rpm * (WHEEL_DIAMETER_M * math.pi / DRIVE_GEAR_RATIO / 60.0)

        self.assertAlmostEqual(rate * WHEEL_DIAMETER_M / 2.0, encoder_mps, places=12)
        self.assertRegex(
            MODULE_SOURCE,
            re.compile(
                r"m_drivingSparkSim\.iterate\(\s*"
                r"mechanismRadiansPerSecondToMotorRpm\(\s*"
                r"driveWheelRadPerSec, SimulationConstants\.kDriveGearRatio",
                re.MULTILINE,
            ),
        )

    def test_previous_meters_per_second_input_would_underreport_by_over_1000x(self):
        # This is the former bug: 1 m/s was passed as if it were 1 motor RPM,
        # then the Spark's m/s-per-RPM conversion factor was applied again.
        drive_velocity_factor = WHEEL_DIAMETER_M * math.pi / DRIVE_GEAR_RATIO / 60.0
        old_reported_mps = 1.0 * drive_velocity_factor

        self.assertAlmostEqual(0.0007858741623, old_reported_mps, places=10)
        self.assertGreater(1.0 / old_reported_mps, 1000.0)
        self.assertNotIn(
            "m_drivePhysicsSim.getAngularVelocityRadPerSec()\n            * SimulationConstants.kWheelRadiusMeters,",
            MODULE_SOURCE,
        )

    def test_steer_sim_uses_neo550_and_steps_reversed_absolute_encoder(self):
        self.assertIn("DCMotor.getNeo550(1)", MODULE_SOURCE)
        module_rate_rad_per_second = 1.2
        mechanism_rate_rad_per_second = -module_rate_rad_per_second
        motor_rpm = mechanism_rate_to_motor_rpm(
            mechanism_rate_rad_per_second, -TURN_GEAR_RATIO
        )
        absolute_encoder_rpm = radians_per_second_to_rpm(mechanism_rate_rad_per_second)
        reported_rate = -absolute_encoder_rpm * (2.0 * math.pi / 60.0)

        self.assertGreater(motor_rpm, 0.0)
        self.assertAlmostEqual(module_rate_rad_per_second, reported_rate, places=12)
        self.assertRegex(
            MODULE_SOURCE,
            re.compile(
                r"m_turningSparkSim\.iterate\(\s*"
                r"mechanismRadiansPerSecondToMotorRpm\(\s*"
                r"turnMechanismRadPerSec,\s*"
                r"-SimulationConstants\.kTurnGearRatio",
                re.MULTILINE,
            ),
        )
        self.assertRegex(
            MODULE_SOURCE,
            re.compile(
                r"m_turningEncoderSim\.iterate\(\s*"
                r"radiansPerSecondToRpm\(turnMechanismRadPerSec\)",
                re.MULTILINE,
            ),
        )

    def test_pathplanner_output_is_discretized_before_swerve_kinematics(self):
        discretize = DRIVE_SOURCE.index("ChassisSpeeds.discretize(speeds")
        kinematics = DRIVE_SOURCE.index(
            "DriveConstants.kDriveKinematics.toSwerveModuleStates(discretizedSpeeds)"
        )
        self.assertLess(discretize, kinematics)


if __name__ == "__main__":
    unittest.main()
