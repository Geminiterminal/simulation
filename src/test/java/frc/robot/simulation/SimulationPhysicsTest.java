package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimulationConstants;
import org.junit.jupiter.api.Test;

/**
 * Sanity checks on the simulation physics calibration. These tests run nothing
 * but WPILib math (no HAL, no vendor libraries), so they execute anywhere:
 * {@code ./gradlew test}.
 *
 * <p>If you change the physical robot (mass, gearing, wheel size, mechanism
 * MOIs) update {@link SimulationConstants}; these bounds keep the constants
 * honest (e.g. catching an inertia entered 10x too large).
 */
class SimulationPhysicsTest {
  private static final double DT = 0.001; // fine step so tests are not discrete-timing sensitive
  private static final double NOMINAL_VOLTAGE = 12.0;

  /** Simulate {@code seconds} of a model at constant voltage, return final output velocity (rad/s). */
  private static double simulateSeconds(DCMotorSim sim, double volts, double seconds) {
    sim.setInputVoltage(volts);
    int steps = (int) (seconds / DT);
    for (int i = 0; i < steps; i++) {
      sim.update(DT);
    }
    return sim.getAngularVelocityRadPerSec();
  }

  private static double simulateSeconds(FlywheelSim sim, double volts, double seconds) {
    sim.setInput(volts);
    int steps = (int) (seconds / DT);
    for (int i = 0; i < steps; i++) {
      sim.update(DT);
    }
    return sim.getAngularVelocityRadPerSec();
  }

  @Test
  void driveMotorModelReachesPlausibleFreeSpeed() {
    DCMotorSim driveModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1),
            SimulationConstants.kDriveWheelInertiaKgMetersSquared,
            SimulationConstants.kDriveGearRatio),
        DCMotor.getNEO(1));

    double wheelRadPerSec = simulateSeconds(driveModel, NOMINAL_VOLTAGE, 2.0);
    double freeSpeedMetersPerSecond = wheelRadPerSec * SimulationConstants.kWheelRadiusMeters;

    // NEO through 5.077:1 on 3" wheels => ~4.46 m/s theoretical. The robot's
    // command cap (DriveConstants) should be within ~35% of the physics model
    // so the sim robot behaves like the real one.
    double ratio = freeSpeedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
    assertTrue(ratio > 0.65 && ratio < 1.35,
        "drive free speed " + freeSpeedMetersPerSecond + " m/s vs allowed max "
            + DriveConstants.kMaxSpeedMetersPerSecond + " m/s");
  }

  @Test
  void driveMotorSpinsUpInPlausibleTime() {
    DCMotorSim driveModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1),
            SimulationConstants.kDriveWheelInertiaKgMetersSquared,
            SimulationConstants.kDriveGearRatio),
        DCMotor.getNEO(1));

    double twoSecondsVelocity = simulateSeconds(driveModel, NOMINAL_VOLTAGE, 2.0);
    DCMotorSim fresh = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1),
            SimulationConstants.kDriveWheelInertiaKgMetersSquared,
            SimulationConstants.kDriveGearRatio),
        DCMotor.getNEO(1));

    // Reach 63% (one time constant-ish) of the 2 s-velocity; expect between
    // 50 ms and 2 s for a ~16 kg-per-corner drivetrain.
    double target = 0.63 * twoSecondsVelocity;
    int steps = 0;
    fresh.setInputVoltage(NOMINAL_VOLTAGE);
    while (fresh.getAngularVelocityRadPerSec() < target && steps * DT < 5.0) {
      fresh.update(DT);
      steps++;
    }
    double spinUpSeconds = steps * DT;
    assertTrue(spinUpSeconds > 0.05 && spinUpSeconds < 2.0,
        "drive spin-up time implausible: " + spinUpSeconds + " s");
  }

  @Test
  void shooterFlywheelSpinsUpToMatchFeedforwardAssumptions() {
    FlywheelSim flywheel = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(1),
            SimulationConstants.kShooterFlywheelMoiKgMetersSquared,
            SimulationConstants.kShooterFlywheelGearRatio),
        DCMotor.getNEO(1));

    double velocityRadPerSec = simulateSeconds(flywheel, NOMINAL_VOLTAGE, 4.0);

    // A 1:1 NEO flywheel should settle near free speed (5676 RPM). The shooter's
    // default setpoints (1500-2000 RPM) must be comfortably reachable.
    double settledRpm = Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec);
    assertTrue(settledRpm > 5000.0 && settledRpm < 6200.0,
        "flywheel settled at implausible speed " + settledRpm + " RPM");
  }

  @Test
  void deployArmTravelsWithinExpectedRange() {
    SingleJointedArmSim arm = new SingleJointedArmSim(
        DCMotor.getNeoVortex(1),
        SimulationConstants.kDeployArmGearRatio,
        SingleJointedArmSim.estimateMOI(
            SimulationConstants.kDeployArmLengthMeters,
            SimulationConstants.kDeployArmMassKg),
        SimulationConstants.kDeployArmLengthMeters,
        Units.degreesToRadians(-95.0),
        SimulationConstants.kDeployArmMaxAngleRad,
        true,
        SimulationConstants.kDeployArmStowedAngleRad);

    arm.setInputVoltage(NOMINAL_VOLTAGE);
    int steps = (int) (3.0 / DT);
    for (int i = 0; i < steps; i++) {
      arm.update(DT);
    }

    // Full voltage for 3 s must swing past the deployed angle (~100 deg)
    // well clear of stowed, i.e. the gearing/MOI mapping is sane.
    assertTrue(arm.getAngleRads() > Units.degreesToRadians(75.0),
        "deploy arm did not approach deployed position; angle = "
            + Units.radiansToDegrees(arm.getAngleRads()) + " deg");
  }
}
