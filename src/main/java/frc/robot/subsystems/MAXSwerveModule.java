// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;
import frc.robot.Constants.SimulationConstants;

/**
 * A single REV MAXSwerve module.
 *
 * <p>Simulation physics (WPILib "Physics Simulation" docs + REV's REVLib sim
 * pattern): each motor is modelled with a {@link DCMotorSim} whose plant is the
 * drive/steer gearbox and load inertia (see {@link SimulationConstants}). Every
 * loop the physics model is driven by the <em>emulated</em> SPARK output
 * (REVLib simulates the onboard velocity/position closed loops, MAXMotion,
 * current limits and voltage compensation), then the resulting mechanism
 * velocity is fed back through {@link SparkMaxSim#iterate}, which updates the
 * simulated encoders that the robot code reads and that the emulated closed
 * loops use as feedback. The result is a closed motor loop in simulation that
 * responds exactly like the real controller: same PID gains, same feedforward,
 * same acceleration and steady-state behavior.
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

  // --- Simulation members (null on the real robot) ---
  private final DCMotor m_driveGearbox = DCMotor.getNEO(1);
  private final DCMotor m_turnGearbox = DCMotor.getNEO(1);
  private final SparkMaxSim m_drivingSparkSim;
  private final SparkMaxSim m_turningSparkSim;
  private final DCMotorSim m_drivePhysicsSim;
  private final DCMotorSim m_turnPhysicsSim;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
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
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    if (RobotBase.isSimulation()) {
      // Controller simulators: emulate the SPARK firmware (closed loops,
      // MAXMotion, current limiting, soft limits, encoders).
      m_drivingSparkSim = new SparkMaxSim(m_drivingSpark, m_driveGearbox);
      m_turningSparkSim = new SparkMaxSim(m_turningSpark, m_turnGearbox);

      // Physics models. LinearSystemId.createDCMotorSystem expects the load
      // inertia at the mechanism OUTPUT and the gearbox reduction; the
      // resulting model's position/velocity are at the output (wheel / module).
      m_drivePhysicsSim = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              m_driveGearbox,
              SimulationConstants.kDriveWheelInertiaKgMetersSquared,
              SimulationConstants.kDriveGearRatio),
          m_driveGearbox);
      m_turnPhysicsSim = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              m_turnGearbox,
              SimulationConstants.kSteerInertiaKgMetersSquared,
              SimulationConstants.kTurnGearRatio),
          m_turnGearbox);
    } else {
      m_drivingSparkSim = null;
      m_turningSparkSim = null;
      m_drivePhysicsSim = null;
      m_turnPhysicsSim = null;
    }

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

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
   * Steps the motor physics for this module. Called once per robot loop from
   * {@link DriveSubsystem} in simulation only.
   *
   * @param dtSeconds timestep in seconds
   */
  public void simulationPeriodic(double dtSeconds) {
    if (!RobotBase.isSimulation()) {
      return;
    }
    double busVoltage = RobotController.getBatteryVoltage();

    // 1. Apply the emulated controller output to the physics models.
    m_drivePhysicsSim.setInputVoltage(m_drivingSparkSim.getAppliedOutput() * busVoltage);
    m_turnPhysicsSim.setInputVoltage(m_turningSparkSim.getAppliedOutput() * busVoltage);

    // 2. Step physics.
    m_drivePhysicsSim.update(dtSeconds);
    m_turnPhysicsSim.update(dtSeconds);

    // 3. Feed the resulting mechanism velocities back to the emulated SPARKs.
    //    iterate() expects velocity in the encoder's configured (converted)
    //    units: drive encoder = meters/second at the wheel, turning absolute
    //    encoder = radians/second of the module azimuth.
    m_drivingSparkSim.iterate(
        m_drivePhysicsSim.getAngularVelocityRadPerSec()
            * SimulationConstants.kWheelRadiusMeters,
        busVoltage, dtSeconds);
    m_turningSparkSim.iterate(
        m_turnPhysicsSim.getAngularVelocityRadPerSec(),
        busVoltage, dtSeconds);
  }

  /** Simulated stator current draw of both module motors, in amps (0 on the real robot). */
  public double getSimulatedCurrentDraw() {
    if (!RobotBase.isSimulation()) {
      return 0.0;
    }
    return m_drivingSparkSim.getMotorCurrent() + m_turningSparkSim.getMotorCurrent();
  }
}
