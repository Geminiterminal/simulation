// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Configs;
import frc.robot.Constants.SimulationConstants;

/**
 * A single REV MAXSwerve module (SPARK MAX + NEO 1.1 drive, SPARK MAX + NEO 550 steer).
 *
 * <p>Simulation physics (WPILib "Physics Simulation" + REVLib 2026 simulation API): each motor is
 * modelled by a {@link DCMotorSim} with the drive/steer reduction and mechanism-side inertia (see
 * {@link SimulationConstants}). Each loop the plant is driven by the emulated SPARK output, and its
 * mechanism velocity is passed directly in the encoder's configured post-conversion units to REVLib:
 * wheel linear velocity ({@code m/s}) for {@link SparkMaxSim#iterate} on the drive motor, and
 * module angular velocity ({@code rad/s}) for {@link SparkMaxSim#iterate} on the steer module.
 * REVLib divides internally by the configured velocity conversion factor to compute motor RPM,
 * integrates position, and emulates the onboard closed-loop firmware. The azimuth Through Bore
 * encoder state is then written straight from the steer plant (position wrapped to {@code [0, 2pi)},
 * velocity in {@code rad/s}) so the values SysId logs are the plant's, not a second integration of
 * them.
 *
 * <p>Simulation caveat for azimuth SysId: REVLib's {@code SparkSim} estimates motor current from the
 * selected sensor's RPM. The azimuth sensor sits on the output of a 46.42:1 gearbox, so the estimate
 * treats the module's ~13 rad/s as the motor's own speed and misses almost all of the back-EMF,
 * reporting tens of amps where the real motor draws a few. The turning SPARK's 20 A smart current
 * limit then throttles the simulated steer motor to roughly 2-3 V even when 6 V is commanded. The
 * logged voltage is the throttled one, so the position and velocity traces stay self-consistent, but
 * gains identified in simulation are not the real robot's. {@code Sim/Steer Motor Current (A)} on the
 * dashboard shows the estimate REVLib is limiting against.
 */
public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private double m_lastTurningEncoderRad = 0.0;
  private double m_continuousTurningAngleRad = 0.0;

  // --- Motor & simulation models ---
  private final DCMotor m_driveGearbox = DCMotor.getNEO(1);
  private final DCMotor m_turnGearbox = DCMotor.getNeo550(1);
  private final SparkMaxSim m_drivingSparkSim;
  private final SparkMaxSim m_turningSparkSim;
  private final SparkAbsoluteEncoderSim m_turningEncoderSim;
  private final DCMotorSim m_drivePhysicsSim;
  private final DCMotorSim m_turnPhysicsSim;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve module with a NEO drive motor,
   * NEO 550 steering motor, SPARK MAX controllers, and a Through Bore encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(
        Configs.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(
        Configs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    if (RobotBase.isSimulation()) {
      // Controller simulators: emulate the SPARK firmware (closed loops,
      // MAXMotion, current limiting, soft limits, encoders).
      m_drivingSparkSim = new SparkMaxSim(m_drivingSpark, m_driveGearbox);
      m_turningSparkSim = new SparkMaxSim(m_turningSpark, m_turnGearbox);

      // Physics models. LinearSystemId.createDCMotorSystem expects the load
      // inertia at the mechanism OUTPUT and the gearbox reduction; the
      // resulting model's position/velocity are at the output (wheel / module).
      m_drivePhysicsSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(
                  m_driveGearbox,
                  SimulationConstants.kDriveWheelInertiaKgMetersSquared,
                  SimulationConstants.kDriveGearRatio),
              m_driveGearbox);
      m_turnPhysicsSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(
                  m_turnGearbox,
                  SimulationConstants.kSteerInertiaKgMetersSquared,
                  SimulationConstants.kTurnGearRatio),
              m_turnGearbox);
      m_turningEncoderSim = m_turningSparkSim.getAbsoluteEncoderSim();
    } else {
      m_drivingSparkSim = null;
      m_turningSparkSim = null;
      m_drivePhysicsSim = null;
      m_turnPhysicsSim = null;
      m_turningEncoderSim = null;
    }

    m_chassisAngularOffset = chassisAngularOffset;
    m_lastTurningEncoderRad = m_turningEncoder.getPosition();
    m_continuousTurningAngleRad = m_lastTurningEncoderRad - m_chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_lastTurningEncoderRad);
    m_drivingEncoder.setPosition(0);
  }

  private void updateContinuousTurningAngle() {
    double currentEncoderRad = m_turningEncoder.getPosition();
    m_continuousTurningAngleRad =
        unwrapAngleStepRadians(
            m_continuousTurningAngleRad, m_lastTurningEncoderRad, currentEncoderRad);
    m_lastTurningEncoderRad = currentEncoderRad;
  }

  static double unwrapAngleStepRadians(
      double previousContinuousRad, double previousRawRad, double currentRawRad) {
    return previousContinuousRad + MathUtil.angleModulus(currentRawRad - previousRawRad);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    updateContinuousTurningAngle();
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    updateContinuousTurningAngle();
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module with zero additional force feedforward.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    setDesiredState(desiredState, 0.0, 0.0);
  }

  /**
   * Sets the desired state for the module with robot-relative wheel force feedforwards from
   * PathPlanner.
   *
   * <p>Projecting the 2D robot-relative force vector onto the optimized wheel direction guarantees
   * exact sign invariance under both inverse-kinematics angle wrapping and 180-degree module
   * state optimization flips.
   *
   * @param desiredState Desired state with speed and angle.
   * @param robotRelativeForceXNewtons X component of robot-relative wheel force feedforward (N).
   * @param robotRelativeForceYNewtons Y component of robot-relative wheel force feedforward (N).
   */
  public void setDesiredState(
      SwerveModuleState desiredState,
      double robotRelativeForceXNewtons,
      double robotRelativeForceYNewtons) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Project 2D force vector onto the optimized chassis-relative wheel heading.
    double arbFeedforwardVolts =
        calculateDriveForceFeedforwardVolts(
            m_driveGearbox,
            correctedDesiredState.angle,
            m_chassisAngularOffset,
            robotRelativeForceXNewtons,
            robotRelativeForceYNewtons);

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setSetpoint(
        correctedDesiredState.speedMetersPerSecond,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        arbFeedforwardVolts);
    m_turningClosedLoopController.setSetpoint(
        correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  /** Returns the most recently commanded state (chassis-relative). */
  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /**
   * Holds the module steering at {@code chassisAngle} (without 180-degree optimization flips) while
   * applying zero drive voltage. Used to pre-align modules before a SysId characterization run.
   */
  public void alignForSysId(Rotation2d chassisAngle) {
    setSysIdDriveVoltsAtAngle(0.0, chassisAngle);
  }

  /**
   * Returns true when the module's measured chassis-relative steering angle is within {@code
   * toleranceRad} of {@code chassisAngle}.
   */
  public boolean isAlignedToAngle(Rotation2d chassisAngle, double toleranceRad) {
    double errorRad =
        MathUtil.angleModulus(getState().angle.getRadians() - chassisAngle.getRadians());
    return Math.abs(errorRad) <= toleranceRad;
  }

  /**
   * Applies open-loop {@code volts} directly to the drive SPARK MAX while holding the steering
   * controller at {@code chassisAngle} with NO 180-degree state optimization, ensuring {@code
   * +volts} always drives along {@code +chassisAngle} and {@code -volts} along {@code
   * -chassisAngle}.
   */
  public void setSysIdDriveVoltsAtAngle(double volts, Rotation2d chassisAngle) {
    double targetEncoderAngleRad =
        MathUtil.inputModulus(
            chassisAngle.getRadians() + m_chassisAngularOffset, 0.0, 2.0 * Math.PI);
    m_turningClosedLoopController.setSetpoint(targetEncoderAngleRad, ControlType.kPosition);
    m_drivingSpark.setVoltage(volts);
    m_desiredState = new SwerveModuleState(0.0, chassisAngle);
  }

  /**
   * Applies open-loop {@code volts} directly to the steering SPARK MAX while keeping the drive
   * motor at 0 V, for azimuth SysId characterization.
   */
  public void setSysIdSteerVolts(double volts) {
    m_drivingSpark.setVoltage(0.0);
    m_turningSpark.setVoltage(volts);
  }

  /** Returns the applied output voltage (Volts) on the drive motor. */
  public double getDriveAppliedVolts() {
    return computeAppliedVolts(
        m_drivingSpark.getAppliedOutput(),
        m_drivingSpark.getBusVoltage(),
        RobotController.getBatteryVoltage(),
        RobotBase.isSimulation());
  }

  /** Returns the drive wheel linear position in meters. */
  public double getDrivePositionMeters() {
    return m_drivingEncoder.getPosition();
  }

  /** Returns the drive wheel linear velocity in meters per second. */
  public double getDriveVelocityMetersPerSecond() {
    return m_drivingEncoder.getVelocity();
  }

  /** Returns the applied output voltage (Volts) on the steering motor. */
  public double getSteerAppliedVolts() {
    return computeAppliedVolts(
        m_turningSpark.getAppliedOutput(),
        m_turningSpark.getBusVoltage(),
        RobotController.getBatteryVoltage(),
        RobotBase.isSimulation());
  }

  /**
   * Returns the continuous (unwrapped) chassis-relative steering angle in radians so multi-turn
   * azimuth SysId tests never jump across the [0, 2pi) absolute-encoder wrap boundary.
   */
  public double getContinuousSteerPositionRadians() {
    updateContinuousTurningAngle();
    return m_continuousTurningAngleRad;
  }

  /** Returns the steering angular velocity in radians per second. */
  public double getSteerVelocityRadiansPerSecond() {
    return m_turningEncoder.getVelocity();
  }

  public static double computeAppliedVolts(
      double appliedOutput, double sparkBusVolts, double batteryVolts, boolean isSimulation) {
    double busVolts = sparkBusVolts;
    if (isSimulation && (!Double.isFinite(busVolts) || busVolts <= 0.0)) {
      busVolts = (Double.isFinite(batteryVolts) && batteryVolts > 0.0) ? batteryVolts : 12.0;
    }
    return appliedOutput * busVolts;
  }

  /**
   * Steps the motor physics for this module. Called once per robot loop from {@link DriveSubsystem}
   * in simulation only.
   *
   * @param dtSeconds timestep in seconds
   */
  public void simulationPeriodic(double dtSeconds) {
    if (!RobotBase.isSimulation()) {
      return;
    }
    double rawBusVoltage = RobotController.getBatteryVoltage();
    double busVoltage =
        (Double.isFinite(rawBusVoltage) && rawBusVoltage > 1.0) ? rawBusVoltage : 12.0;

    // 1. Apply the emulated controller output to the mechanism physics plants.
    m_drivePhysicsSim.setInputVoltage(m_drivingSparkSim.getAppliedOutput() * busVoltage);
    m_turnPhysicsSim.setInputVoltage(m_turningSparkSim.getAppliedOutput() * busVoltage);

    // 2. Step physics. Both DCMotorSim plants output mechanism-side angular
    //    velocity (rad/s) because the gear reduction is embedded in each plant.
    m_drivePhysicsSim.update(dtSeconds);
    m_turnPhysicsSim.update(dtSeconds);

    double wheelSpeedMetersPerSec =
        wheelRadiansPerSecondToMetersPerSecond(m_drivePhysicsSim.getAngularVelocityRadPerSec());
    double moduleTurnRateRadPerSec = m_turnPhysicsSim.getAngularVelocityRadPerSec();

    // 3. REVLib 2026 SparkSim.iterate expects post-conversion units matching the
    //    configured encoder conversion factors: wheel linear speed (m/s) for
    //    drive, module angular rate (rad/s) for steer.
    m_drivingSparkSim.iterate(wheelSpeedMetersPerSec, busVoltage, dtSeconds);
    m_turningSparkSim.iterate(moduleTurnRateRadPerSec, busVoltage, dtSeconds);

    // 4. Publish the azimuth encoder state explicitly instead of re-integrating
    //    it with SparkAbsoluteEncoderSim.iterate():
    //
    //    a) SparkMaxSim.iterate() already mirrors its internal position and
    //       velocity into the selected feedback sensor (the absolute encoder,
    //       for the turning SPARK). Calling SparkAbsoluteEncoderSim.iterate()
    //       on top of that added a SECOND omega*dt step every loop, so the
    //       reported module angle led the physics plant by a full loop of
    //       travel (5-15 deg during an azimuth SysId sweep) and disagreed with
    //       the reported angular velocity - exactly the position/velocity
    //       inconsistency that makes an azimuth SysId trace useless.
    //    b) SparkAbsoluteEncoderSim.iterate() only integrates POSITION; it never
    //       writes the encoder VELOCITY. Writing both from the plant keeps the
    //       pair consistent no matter which REVLib code path is live.
    //
    //    The plant position is wrapped into [0, 2*pi) like the real Through Bore
    //    encoder, so the multi-turn unwrapping that getContinuousSteerPosition
    //    Radians() performs for SysId is exercised in simulation too.
    m_turningEncoderSim.setPosition(
        MathUtil.inputModulus(m_turnPhysicsSim.getAngularPositionRad(), 0.0, 2.0 * Math.PI));
    m_turningEncoderSim.setVelocity(moduleTurnRateRadPerSec);
    updateContinuousTurningAngle();
  }

  /** Converts wheel output angular rate (rad/s) to linear wheel speed (m/s) for SparkSim. */
  static double wheelRadiansPerSecondToMetersPerSecond(double wheelRadiansPerSecond) {
    return wheelRadiansPerSecond * SimulationConstants.kWheelRadiusMeters;
  }

  /** Computes arbitrary drive voltage feedforward for a 2D robot-relative wheel force vector. */
  static double calculateDriveForceFeedforwardVolts(
      DCMotor gearbox,
      Rotation2d optimizedEncoderAngle,
      double chassisAngularOffsetRad,
      double robotRelativeForceXNewtons,
      double robotRelativeForceYNewtons) {
    Rotation2d wheelDirection =
        optimizedEncoderAngle.minus(Rotation2d.fromRadians(chassisAngularOffsetRad));
    double forceAlongWheelNewtons =
        robotRelativeForceXNewtons * wheelDirection.getCos()
            + robotRelativeForceYNewtons * wheelDirection.getSin();
    double motorTorqueNm =
        (forceAlongWheelNewtons * SimulationConstants.kWheelRadiusMeters)
            / SimulationConstants.kDriveGearRatio;
    return gearbox.getVoltage(motorTorqueNm, 0.0);
  }

  /** Simulated positive battery supply current draw of both module motors, in amps. */
  public double getSimulatedCurrentDraw() {
    if (!RobotBase.isSimulation()) {
      return 0.0;
    }
    double driveSupplyAmps =
        m_drivingSparkSim.getAppliedOutput() * m_drivingSparkSim.getMotorCurrent();
    double turnSupplyAmps =
        m_turningSparkSim.getAppliedOutput() * m_turningSparkSim.getMotorCurrent();
    double total = 0.0;
    if (Double.isFinite(driveSupplyAmps) && driveSupplyAmps > 0.0) {
      total += driveSupplyAmps;
    }
    if (Double.isFinite(turnSupplyAmps) && turnSupplyAmps > 0.0) {
      total += turnSupplyAmps;
    }
    return total;
  }

  /**
   * Simulated steering-motor current in amps, exactly as REVLib's SPARK sim estimates it. Published
   * on the dashboard so the azimuth current-limit throttling described in the class javadoc is
   * visible while running a {@code Drivetrain Steer} SysId test.
   */
  public double getSimulatedSteerCurrentAmps() {
    if (!RobotBase.isSimulation()) {
      return 0.0;
    }
    double steerAmps = m_turningSparkSim.getMotorCurrent();
    return Double.isFinite(steerAmps) ? steerAmps : 0.0;
  }
}
