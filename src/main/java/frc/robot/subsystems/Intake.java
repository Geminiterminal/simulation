package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.SimulationConstants;

public class Intake extends SubsystemBase {

    // --- Deploy PID & Position Constants ---
    private static final double DEPLOY_kP = 0.3;
    private static final double DEPLOY_kI = 0.0;
    private static final double DEPLOY_kD = 0.0;

    // --- Conveyor Velocity PID Constants (SPARK MAX + NEO 1.1) ---
    private static final double CONVEYOR_TARGET_RPM = 4500.0;
    private static final double CONVEYOR_kP = 0.0001;
    private static final double CONVEYOR_kI = 0.0;
    private static final double CONVEYOR_kD = 0.0;
    private static final double CONVEYOR_kV = 12.0 / NeoMotorConstants.kFreeSpeedRpm;

    // --- Roller Velocity Constants ---
    private static final double ROLLER_TARGET_RPM = 5600.0;
    private static final double ROLLER_TARGET_RPS = ROLLER_TARGET_RPM / 60.0;
    private static final double ROLLER_kP = 0.12;
    private static final double ROLLER_kI = 0.0;
    private static final double ROLLER_kD = 0.0;
    private static final double ROLLER_kV = 0.12;

    private static final double STOW_POSITION   = 0.0;
    private static final double DEPLOY_POSITION = 59;
    private static final double MID_POSITION = 10;
    private static final double CYCLE_LOW_POSITION = 30;
    private static final double DEPLOY_POSITION_TOLERANCE = 1.0;
    private static final double DEPLOY_MAX_MOTION_CRUISE_VELOCITY = 3000.0;
    private static final double DEPLOY_MAX_MOTION_ACCELERATION = 6000.0;
    private static final double SHOT_CYCLE_DWELL_SECONDS = 0;

    private static final double STOW_SOFT_LIMIT   = -0.5;
    private static final double DEPLOY_SOFT_LIMIT = (125.0 / 3.0) + 1.0;

    // --- Motors ---
    // IMPORTANT: If ID 9 is physically a SPARK MAX, change SparkFlex to SparkMax here!
    private final SparkFlex deployMotor;
    private final TalonFX rollerMotor;
    private final SparkMax conveyorMotor;

    private final DutyCycleOut rollerDutyCycleRequest = new DutyCycleOut(0.0);
    private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0.0);

    private final SparkClosedLoopController deployController;
    private final SparkClosedLoopController conveyorController;

    private boolean cyclingForShot = false;
    private boolean rollerRunning = false;
    private double deployTargetPosition = DEPLOY_POSITION;
    private double shotCycleDwellStartTime = -1.0;

    // --- Simulation members (null on the real robot) ---
    private final DCMotor m_deployGearbox = DCMotor.getNeoVortex(1);
    private final DCMotor m_conveyorGearbox = DCMotor.getNEO(1);
    private SparkFlexSim m_deploySparkSim;
    private SingleJointedArmSim m_deployArmPhysics;
    private TalonFXSimState m_rollerSimState;
    private DCMotorSim m_rollerPhysics;
    private SparkMaxSim m_conveyorSparkSim;
    private DCMotorSim m_conveyorPhysics;
    private MechanismLigament2d m_armViz;

    // --- Constants ---
    private static final int DEPLOY_MOTOR_ID   = 9;
    private static final int ROLLER_MOTOR_ID   = 1;
    private static final int CONVEYOR_MOTOR_ID = 19;

    private Intake() {
        deployMotor   = new SparkFlex(DEPLOY_MOTOR_ID, MotorType.kBrushless);
        rollerMotor   = new TalonFX(ROLLER_MOTOR_ID);
        conveyorMotor = new SparkMax(CONVEYOR_MOTOR_ID, MotorType.kBrushless);

        deployController = deployMotor.getClosedLoopController();
        conveyorController = conveyorMotor.getClosedLoopController();

        // 1. Deploy Motor Config
        SparkFlexConfig deployConfig = new SparkFlexConfig();
        deployConfig.idleMode(IdleMode.kBrake)
                    .inverted(false)
                    .smartCurrentLimit(40);

        deployConfig.closedLoop.pid(DEPLOY_kP, DEPLOY_kI, DEPLOY_kD);
        deployConfig.closedLoop.maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            .cruiseVelocity(DEPLOY_MAX_MOTION_CRUISE_VELOCITY)
            .maxAcceleration(DEPLOY_MAX_MOTION_ACCELERATION)
            .allowedProfileError(DEPLOY_POSITION_TOLERANCE);
        deployConfig.softLimit
            .forwardSoftLimitEnabled(false)
            .forwardSoftLimit(DEPLOY_SOFT_LIMIT)
            .reverseSoftLimitEnabled(false)
            .reverseSoftLimit(STOW_SOFT_LIMIT);

        deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        deployMotor.getEncoder().setPosition(0);

        // 2. Roller Motor Config
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfig.Slot0.kP = ROLLER_kP;
        rollerConfig.Slot0.kI = ROLLER_kI;
        rollerConfig.Slot0.kD = ROLLER_kD;
        rollerConfig.Slot0.kV = ROLLER_kV;
        rollerMotor.getConfigurator().apply(rollerConfig);
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            rollerMotor.getPosition(),
            rollerMotor.getVelocity(),
            rollerMotor.getMotorVoltage());

        // 3. Conveyor Motor Config
        SparkMaxConfig conveyorConfig = new SparkMaxConfig();
        conveyorConfig.idleMode(IdleMode.kBrake)
                      .inverted(false)
                      .smartCurrentLimit(40);
        conveyorConfig.closedLoop.pid(CONVEYOR_kP, CONVEYOR_kI, CONVEYOR_kD);
        conveyorConfig.closedLoop.feedForward.kV(CONVEYOR_kV);
        conveyorMotor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- Simulation setup: physics models + controller emulators ---
        if (RobotBase.isSimulation()) {
            // Deploy arm: single-jointed arm with gravity, Neo Vortex through
            // the calibrated total reduction.
            m_deploySparkSim = new SparkFlexSim(deployMotor, m_deployGearbox);
            m_deployArmPhysics = new SingleJointedArmSim(
                m_deployGearbox,
                SimulationConstants.kDeployArmGearRatio,
                SingleJointedArmSim.estimateMOI(
                    SimulationConstants.kDeployArmLengthMeters,
                    SimulationConstants.kDeployArmMassKg),
                SimulationConstants.kDeployArmLengthMeters,
                Units.degreesToRadians(-95.0),
                SimulationConstants.kDeployArmMaxAngleRad,
                true, // simulate gravity
                SimulationConstants.kDeployArmStowedAngleRad);

            // Roller (Kraken X60 / TalonFX).
            m_rollerSimState = rollerMotor.getSimState();
            m_rollerPhysics = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1),
                    SimulationConstants.kIntakeRollerMoiKgMetersSquared,
                    SimulationConstants.kIntakeRollerGearRatio),
                DCMotor.getKrakenX60(1));

            // Conveyor (NEO / SPARK MAX).
            m_conveyorSparkSim = new SparkMaxSim(conveyorMotor, m_conveyorGearbox);
            m_conveyorPhysics = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    m_conveyorGearbox,
                    SimulationConstants.kConveyorMoiKgMetersSquared,
                    SimulationConstants.kConveyorGearRatio),
                m_conveyorGearbox);

            // Visual widget for the deploy arm.
            Mechanism2d mech = new Mechanism2d(0.8, 0.8);
            MechanismRoot2d root = mech.getRoot("Intake Pivot", 0.4, 0.15);
            m_armViz = root.append(new MechanismLigament2d(
                "Deploy Arm", 0.35, 90.0)); // drawn pointing up when stowed
            SmartDashboard.putData("Intake Arm", mech);
        }
    }

    // --- Singleton ---
    public static class instanceHolder {
        private static final Intake INSTANCE = new Intake();
    }

    public static Intake getInstance() {
        return instanceHolder.INSTANCE;
    }

    // --- DEPLOY / STOW LOGIC ---
    public void setDeployMotorSpeed(double speed) {
        // The hardware soft limits protect this from crashing into the frame!
        cyclingForShot = false;
        shotCycleDwellStartTime = -1.0;
        deployMotor.set(speed);
    }

    public void stow() {
        cyclingForShot = false;
        shotCycleDwellStartTime = -1.0;
        setDeployPosition(STOW_POSITION);
    }

    public void deploy() {
        cyclingForShot = false;
        shotCycleDwellStartTime = -1.0;
        setDeployPosition(DEPLOY_POSITION);
    }

    /** Starts moving the deploy mechanism between its deployed and middle positions. */
    public void startShotCycle() {
        cyclingForShot = true;
        shotCycleDwellStartTime = -1.0;
        deployTargetPosition = MID_POSITION;
        setDeployPosition(deployTargetPosition);
    }

    /** Stops the shooting cycle and returns the deploy mechanism to its deployed position. */
    public void stopShotCycle() {
        cyclingForShot = false;
        shotCycleDwellStartTime = -1.0;
        deployTargetPosition = DEPLOY_POSITION;
        setDeployPosition(deployTargetPosition);
    }


    private void setDeployPosition(double position) {
        deployController.setReference(position, ControlType.kMAXMotionPositionControl);
    }

    // --- ROLLER & CONVEYOR LOGIC ---
    public void toggleRoller() {
        if (rollerRunning) stopRoller();
        else runRoller();
    }

    public void stopRoller() {
        rollerRunning = false;
        rollerMotor.setControl(rollerDutyCycleRequest.withOutput(0.0));
    }

    public void runRoller() {
        rollerRunning = true;
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(ROLLER_TARGET_RPS));
    }

    public void runConveyor()     { conveyorController.setSetpoint(CONVEYOR_TARGET_RPM, ControlType.kVelocity); }
    public void reverseConveyor() { conveyorMotor.set(-0.8); }
    public void stopConveyor()    { conveyorMotor.set(0.0);  }

    @Override
    public void periodic() {
        if (cyclingForShot) {
            boolean deployAtTarget = Math.abs(deployMotor.getEncoder().getPosition() - deployTargetPosition)
                <= DEPLOY_POSITION_TOLERANCE;

            if (deployAtTarget) {
                if (shotCycleDwellStartTime < 0.0) {
                    shotCycleDwellStartTime = Timer.getFPGATimestamp();
                } else if (Timer.getFPGATimestamp() - shotCycleDwellStartTime >= SHOT_CYCLE_DWELL_SECONDS) {
                    deployTargetPosition = deployTargetPosition == MID_POSITION
                        ? CYCLE_LOW_POSITION
                        : MID_POSITION;
                    shotCycleDwellStartTime = -1.0;
                    setDeployPosition(deployTargetPosition);
                }
            } else {
                shotCycleDwellStartTime = -1.0;
            }
        }

        SmartDashboard.putNumber("Conveyor current draw", conveyorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Conveyor output", conveyorMotor.getAppliedOutput());
        SmartDashboard.putNumber("Conveyor RPM", conveyorMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intake Roller Target RPM", ROLLER_TARGET_RPM);
        SmartDashboard.putNumber("Intake Roller RPM", rollerMotor.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("Deploy Position (Rotations)", deployMotor.getEncoder().getPosition());
    }

    /**
     * Simulation: deploy-arm physics (single-jointed arm with gravity, driven
     * through the calibrated gearbox) plus roller and conveyor drum physics.
     * The SPARK controller loops are emulated by REVLib (including MAXMotion);
     * the TalonFX runs Phoenix's own closed-loop sim.
     */
    @Override
    public void simulationPeriodic() {
        if (!RobotBase.isSimulation()) {
            return;
        }
        double dt = SimulationConstants.kLoopPeriodSeconds;
        double rawBusVoltage = RobotController.getBatteryVoltage();
        double busVoltage =
            (Double.isFinite(rawBusVoltage) && rawBusVoltage > 1.0) ? rawBusVoltage : 12.0;

        // Deploy arm. The SPARK encoder has no conversion factors configured,
        // so iterate() expects MOTOR rpm; convert arm velocity through the
        // gearbox.
        m_deployArmPhysics.setInput(m_deploySparkSim.getAppliedOutput() * busVoltage);
        m_deployArmPhysics.update(dt);
        m_deploySparkSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute(
                m_deployArmPhysics.getVelocityRadPerSec()
                    * SimulationConstants.kDeployArmGearRatio),
            busVoltage, dt);

        // Roller (Phoenix 6 sim).
        m_rollerSimState.setSupplyVoltage(busVoltage);
        m_rollerPhysics.setInputVoltage(m_rollerSimState.getMotorVoltage());
        m_rollerPhysics.update(dt);
        m_rollerSimState.setRawRotorPosition(
            m_rollerPhysics.getAngularPositionRotations()
                * SimulationConstants.kIntakeRollerGearRatio);
        m_rollerSimState.setRotorVelocity(
            m_rollerPhysics.getAngularVelocityRPM() / 60.0
                * SimulationConstants.kIntakeRollerGearRatio);

        // Conveyor (encoder units are native RPM).
        m_conveyorPhysics.setInputVoltage(m_conveyorSparkSim.getAppliedOutput() * busVoltage);
        m_conveyorPhysics.update(dt);
        m_conveyorSparkSim.iterate(m_conveyorPhysics.getAngularVelocityRPM(), busVoltage, dt);

        // Arm visualization: stowed = pointing up (90 deg), deployed swings down.
        double armAngleDeg = Units.radiansToDegrees(m_deployArmPhysics.getAngleRads());
        m_armViz.setAngle(90.0 - armAngleDeg);
        SmartDashboard.putNumber("Sim/Deploy Arm Angle (deg)", armAngleDeg);
    }

    /** Total simulated positive battery current draw of the intake, in amps. */
    public double getSimulatedCurrentDraw() {
        if (!RobotBase.isSimulation()) {
            return 0.0;
        }
        double deploySupply =
            m_deploySparkSim.getAppliedOutput() * m_deploySparkSim.getMotorCurrent();
        double conveyorSupply =
            m_conveyorSparkSim.getAppliedOutput() * m_conveyorSparkSim.getMotorCurrent();
        double rollerCurrent =
            DCMotor.getKrakenX60(1).getCurrent(
                m_rollerPhysics.getAngularVelocityRadPerSec()
                    * SimulationConstants.kIntakeRollerGearRatio,
                m_rollerSimState.getMotorVoltage());
        double total = 0.0;
        if (Double.isFinite(deploySupply) && deploySupply > 0.0) {
            total += deploySupply;
        }
        if (Double.isFinite(conveyorSupply) && conveyorSupply > 0.0) {
            total += conveyorSupply;
        }
        if (Double.isFinite(rollerCurrent) && rollerCurrent > 0.0) {
            total += rollerCurrent;
        }
        return total;
    }

    // =========================================================================
    // --- SYSTEM IDENTIFICATION (WPILib SysId) ---
    // =========================================================================

    /**
     * Software safety travel window (in motor rotations) for Intake Deploy Arm SysId routines. The
     * arm travels from {@code 0.0} (horizontal stowed) to {@code 59.0} (deployed); keeping a
     * 4-rotation buffer on both ends prevents driving into physical hard stops while hardware soft
     * limits are disabled.
     */
    static final double SYSID_DEPLOY_MIN_ROTATIONS = 4.0;
    static final double SYSID_DEPLOY_MAX_ROTATIONS = 55.0;

    private final MutVoltage m_sysIdVoltage = Volts.mutable(0.0);
    private final MutAngle m_sysIdAngle = Rotations.mutable(0.0);
    private final MutAngularVelocity m_sysIdVelocity = RotationsPerSecond.mutable(0.0);

    /**
     * Characterizes the Intake Deploy Arm (SparkFlex + NEO Vortex) with arm-safe ramp/step voltages
     * and logs arm-joint angle/velocity (with {@code 0} rotations = horizontal stowed) so WPILib
     * SysId's {@code Arm} gravity fit ({@code kG * cos(theta)}) uses true joint angles rather than
     * 59 motor shaft turns.
     */
    private final SysIdRoutine m_deploySysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),
                Volts.of(2.5),
                Seconds.of(6.0)),
            new SysIdRoutine.Mechanism(
                voltage -> setSysIdDeployVolts(voltage.in(Volts)),
                log -> {
                    double armRotations =
                        deployMotorRotationsToArmRotations(deployMotor.getEncoder().getPosition());
                    double armRps =
                        deployMotorRpmToArmRotationsPerSecond(deployMotor.getEncoder().getVelocity());
                    log.motor("intake-deploy")
                        .voltage(m_sysIdVoltage.mut_replace(getSparkAppliedVolts(deployMotor), Volts))
                        .angularPosition(m_sysIdAngle.mut_replace(armRotations, Rotations))
                        .angularVelocity(m_sysIdVelocity.mut_replace(armRps, RotationsPerSecond));
                },
                this,
                "intake-deploy"));

    /** Characterizes the Intake Roller (Kraken X60 / TalonFX) in rotations and rotations/sec. */
    private final SysIdRoutine m_rollerSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.0).per(Second),
                Volts.of(6.0),
                Seconds.of(10.0)),
            new SysIdRoutine.Mechanism(
                voltage -> setSysIdRollerVolts(voltage.in(Volts)),
                log -> {
                    log.motor("intake-roller")
                        .voltage(
                            m_sysIdVoltage.mut_replace(
                                rollerMotor.getMotorVoltage().getValueAsDouble(), Volts))
                        .angularPosition(
                            m_sysIdAngle.mut_replace(
                                rollerMotor.getPosition().getValueAsDouble(), Rotations))
                        .angularVelocity(
                            m_sysIdVelocity.mut_replace(
                                rollerMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
                },
                this,
                "intake-roller"));

    /** Characterizes the Intake Conveyor (SPARK MAX + NEO 1.1). */
    private final SysIdRoutine m_conveyorSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.0).per(Second),
                Volts.of(6.0),
                Seconds.of(10.0)),
            new SysIdRoutine.Mechanism(
                voltage -> conveyorMotor.setVoltage(voltage.in(Volts)),
                log -> {
                    log.motor("intake-conveyor")
                        .voltage(m_sysIdVoltage.mut_replace(getSparkAppliedVolts(conveyorMotor), Volts))
                        .angularPosition(
                            m_sysIdAngle.mut_replace(conveyorMotor.getEncoder().getPosition(), Rotations))
                        .angularVelocity(
                            m_sysIdVelocity.mut_replace(conveyorMotor.getEncoder().getVelocity(), RPM));
                },
                this,
                "intake-conveyor"));

    private static double getSparkAppliedVolts(SparkBase spark) {
        return MAXSwerveModule.computeAppliedVolts(
            spark.getAppliedOutput(),
            spark.getBusVoltage(),
            RobotController.getBatteryVoltage(),
            RobotBase.isSimulation());
    }

    static double deployMotorRotationsToArmRotations(double motorRotations) {
        return motorRotations / SimulationConstants.kDeployArmGearRatio;
    }

    static double deployMotorRpmToArmRotationsPerSecond(double motorRpm) {
        return (motorRpm / 60.0) / SimulationConstants.kDeployArmGearRatio;
    }

    static double clampDeploySysIdVolts(double requestedVolts, double motorRotations) {
        if (requestedVolts > 0.0 && motorRotations >= SYSID_DEPLOY_MAX_ROTATIONS) {
            return 0.0;
        }
        if (requestedVolts < 0.0 && motorRotations <= SYSID_DEPLOY_MIN_ROTATIONS) {
            return 0.0;
        }
        return requestedVolts;
    }

    static boolean hasReachedDeploySysIdLimit(
            SysIdRoutine.Direction direction, double motorRotations) {
        return direction == SysIdRoutine.Direction.kForward
            ? motorRotations >= SYSID_DEPLOY_MAX_ROTATIONS
            : motorRotations <= SYSID_DEPLOY_MIN_ROTATIONS;
    }

    private void setSysIdDeployVolts(double requestedVolts) {
        cyclingForShot = false;
        shotCycleDwellStartTime = -1.0;
        double safeVolts =
            clampDeploySysIdVolts(requestedVolts, deployMotor.getEncoder().getPosition());
        deployMotor.setVoltage(safeVolts);
    }

    private void setSysIdRollerVolts(double volts) {
        rollerRunning = false;
        rollerMotor.setControl(rollerVoltageRequest.withOutput(volts));
    }

    public double getDeployPositionRotations() {
        return deployMotor.getEncoder().getPosition();
    }

    public boolean isDeployArmStopped() {
        return Math.abs(deployMotor.getEncoder().getVelocity()) < 30.0;
    }

    public boolean isRollerStopped() {
        return Math.abs(rollerMotor.getVelocity().getValueAsDouble()) < 1.0;
    }

    public boolean isConveyorStopped() {
        return Math.abs(conveyorMotor.getEncoder().getVelocity()) < 60.0;
    }

    public Command sysIdDeployQuasistatic(SysIdRoutine.Direction direction) {
        return m_deploySysIdRoutine
            .quasistatic(direction)
            .until(() -> hasReachedDeploySysIdLimit(direction, getDeployPositionRotations()));
    }

    public Command sysIdDeployDynamic(SysIdRoutine.Direction direction) {
        return m_deploySysIdRoutine
            .dynamic(direction)
            .until(() -> hasReachedDeploySysIdLimit(direction, getDeployPositionRotations()));
    }

    public Command sysIdRollerQuasistatic(SysIdRoutine.Direction direction) {
        return m_rollerSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdRollerDynamic(SysIdRoutine.Direction direction) {
        return m_rollerSysIdRoutine.dynamic(direction);
    }

    public Command sysIdConveyorQuasistatic(SysIdRoutine.Direction direction) {
        return m_conveyorSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdConveyorDynamic(SysIdRoutine.Direction direction) {
        return m_conveyorSysIdRoutine.dynamic(direction);
    }
}