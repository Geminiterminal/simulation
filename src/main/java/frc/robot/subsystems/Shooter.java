package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.sim.SparkMaxSim;

// --- CTRE Phoenix 6 Imports ---
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

    // --- Motors ---
    private final SparkMax shooterFront;
    private final SparkMax shooterBack;
    private final TalonFX feederMain;  // Kraken X60

    // --- PID Controllers (REV) ---
    private final SparkClosedLoopController frontController;
    private final SparkClosedLoopController backController;

    // --- CTRE Control Requests ---
    // Reuse control request objects to avoid repeated allocations.
    private final DutyCycleOut feederDutyCycleRequest = new DutyCycleOut(0.0);
    private final VelocityVoltage feederVelocityRequest = new VelocityVoltage(0.0);

    // --- State ---
    private boolean systemEnabled    = false;
    private boolean waitingForSpinUp = false;
    private boolean alignmentOverride = false;

    // --- CAN IDs ---
    private static final int SHOOTER_ID_1 = 5;
    private static final int SHOOTER_ID_2 = 6;
    private static final int FEEDER_ID_1  = 7;

    // --- Velocity Setpoints ---
    public double m_fronttargetRPM       = 1500.0;
    public double m_backtargetRPM       = 2000.0;

    // --- Interpolation Map for Dynamic RPM ---
    private final InterpolatingDoubleTreeMap m_rpmMap = new InterpolatingDoubleTreeMap();

    // --- REV PID + Feedforward gains (SPARK MAX + NEO 1.1) ---
    // REVLib 2026 FeedForwardConfig.kV is in Volts / RPM when native RPM units are used.
    private static final double kP = 0.0009;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 12.0 / NeoMotorConstants.kFreeSpeedRpm;

    private static final double FEEDER_TARGET_RPM = 4500.0;
    private static final double FEEDER_TARGET_RPS = FEEDER_TARGET_RPM / 60.0;

    // CTRE velocity PID/feedforward constants for the feeder Kraken.
    private static final double FEEDER_kP = 0.12;
    private static final double FEEDER_kI = 0.0;
    private static final double FEEDER_kD = 0.0;
    private static final double FEEDER_kV = 0.12;

    // --- Simulation members (null on the real robot) ---
    private final DCMotor m_flywheelGearbox = DCMotor.getNEO(1);
    private SparkMaxSim m_frontSparkSim;
    private SparkMaxSim m_backSparkSim;
    private FlywheelSim m_frontPhysics;
    private FlywheelSim m_backPhysics;
    private TalonFXSimState m_feederSimState;
    private DCMotorSim m_feederPhysics;

    public Shooter() {

        m_rpmMap.put(215.0, 5000.0);
        SmartDashboard.putNumber("Front Shooter RPM Setpoint", m_fronttargetRPM);
        SmartDashboard.putNumber("Back Shooter RPM Setpoint", m_backtargetRPM);

        // --- Shooter Front Config (REV) ---
        SparkMaxConfig flyConfig = new SparkMaxConfig();
        flyConfig.idleMode(IdleMode.kCoast)
                 .inverted(false)
                 .smartCurrentLimit(60);
        flyConfig.closedLoop
            .pid(kP, kI, kD);
        flyConfig.closedLoop.feedForward
            .kV(kV);

        // --- Shooter Back Config (REV) ---
        SparkMaxConfig flyConfig2 = new SparkMaxConfig();
        flyConfig2.idleMode(IdleMode.kCoast)
                  .inverted(true)
                  .smartCurrentLimit(60);
        flyConfig2.closedLoop
            .pid(kP, kI, kD);
        flyConfig2.closedLoop.feedForward
            .kV(kV);

        // --- Shooter Front ---
        shooterFront = new SparkMax(SHOOTER_ID_1, MotorType.kBrushless);
        shooterFront.configure(flyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frontController = shooterFront.getClosedLoopController();

        // --- Shooter Back ---
        shooterBack = new SparkMax(SHOOTER_ID_2, MotorType.kBrushless);
        shooterBack.configure(flyConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        backController = shooterBack.getClosedLoopController();

        // --- Feeder (Kraken X60 / TalonFX) ---
        feederMain = new TalonFX(FEEDER_ID_1);

        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Match your old inverted(true) setting from the SparkMax config:
        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederConfig.Slot0.kP = FEEDER_kP;
        feederConfig.Slot0.kI = FEEDER_kI;
        feederConfig.Slot0.kD = FEEDER_kD;
        feederConfig.Slot0.kV = FEEDER_kV;

        feederMain.getConfigurator().apply(feederConfig);

        // --- Simulation setup: physics models + controller emulators ---
        if (RobotBase.isSimulation()) {
            m_frontSparkSim = new SparkMaxSim(shooterFront, m_flywheelGearbox);
            m_backSparkSim = new SparkMaxSim(shooterBack, m_flywheelGearbox);
            m_frontPhysics = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    m_flywheelGearbox,
                    SimulationConstants.kShooterFlywheelMoiKgMetersSquared,
                    SimulationConstants.kShooterFlywheelGearRatio),
                m_flywheelGearbox);
            m_backPhysics = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    m_flywheelGearbox,
                    SimulationConstants.kShooterFlywheelMoiKgMetersSquared,
                    SimulationConstants.kShooterFlywheelGearRatio),
                m_flywheelGearbox);

            // Phoenix 6 controllers simulate their onboard loops internally;
            // we supply the physics of the mechanism they are driving.
            m_feederSimState = feederMain.getSimState();
            m_feederPhysics = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1),
                    SimulationConstants.kFeederMoiKgMetersSquared,
                    SimulationConstants.kFeederGearRatio),
                DCMotor.getKrakenX60(1));
        }
    }

    // --- Singleton ---
    private static class InstanceHolder {
        private static final Shooter INSTANCE = new Shooter();
    }

    public static Shooter getInstance() {
        return InstanceHolder.INSTANCE;
    }

    // -------------------------------------------------------------------------
    // PRIVATE HELPERS — replace feederMain.set(speed)
    // -------------------------------------------------------------------------
    private void setFeederDutyCycle(double speed) {
        feederMain.setControl(feederDutyCycleRequest.withOutput(speed));
    }

    private void runFeederAtTargetSpeed() {
        feederMain.setControl(feederVelocityRequest.withVelocity(FEEDER_TARGET_RPS));
    }

    // -------------------------------------------------------------------------
    // PUBLIC API
    // -------------------------------------------------------------------------

    public void start() {
        systemEnabled    = true;
        waitingForSpinUp = true;

        frontController.setSetpoint(m_fronttargetRPM, ControlType.kVelocity);
        backController.setSetpoint(m_backtargetRPM,   ControlType.kVelocity);

        setFeederDutyCycle(0.0); // Hold feeder until up to speed (periodic handles handoff)
    }

    /** Spins up both shooter wheels without starting the feeder or conveyor. */
    public void spinUpToTargetPercentage(double targetPercentage) {
        frontController.setSetpoint(m_fronttargetRPM * targetPercentage, ControlType.kVelocity);
        backController.setSetpoint(m_backtargetRPM * targetPercentage, ControlType.kVelocity);
    }

    public void stopAll() {
        systemEnabled    = false;
        waitingForSpinUp = false;
        alignmentOverride = false;

        shooterFront.set(0);
        shooterBack.set(0);
        setFeederDutyCycle(0.0);

        Intake.getInstance().stopConveyor();
        Intake.getInstance().stopShotCycle();
    }

    public boolean isSystemEnabled() {
        return systemEnabled;
    }

    /**
     * Allows a held manual-shoot command to skip vision alignment checks. This
     * never skips the flywheel speed check in {@link #periodic()}.
     */
    public void setAlignmentOverride(boolean alignmentOverride) {
        this.alignmentOverride = alignmentOverride;
    }

    public boolean isAlignmentOverrideActive() {
        return alignmentOverride;
    }

    public void reverseFeeder() { setFeederDutyCycle(-0.8); }
    public void stopFeeder()    { setFeederDutyCycle(0.0); }

    public void setFrontTargetRPM(double rpm) {
        m_fronttargetRPM = rpm;
    }

    public void setBackTargetRPM(double rpm) {
        m_backtargetRPM = rpm;
    }


    public boolean isAtTargetSpeed() {
        double currentFrontRPM = shooterFront.getEncoder().getVelocity();
        double currentBackRPM  = shooterBack.getEncoder().getVelocity();

        double frontAllowedError = m_fronttargetRPM * 0.1;
        double backAllowedError  = m_backtargetRPM * 0.1;

        boolean frontReady = currentFrontRPM >= m_fronttargetRPM
                && currentFrontRPM <= m_fronttargetRPM + frontAllowedError;
        boolean backReady = currentBackRPM >= m_backtargetRPM
                && currentBackRPM <= m_backtargetRPM + backAllowedError;

        return frontReady && backReady;
    }

    static boolean isReadyToFeed(
            boolean atTargetSpeed,
            boolean autoAimFinished,
            boolean autoDistanceFinished,
            boolean alignmentOverride) {
        return atTargetSpeed
                && (alignmentOverride || (autoAimFinished && autoDistanceFinished));
    }

    @Override
    public void periodic() {
        m_fronttargetRPM = SmartDashboard.getNumber("Front Shooter RPM Setpoint", m_fronttargetRPM);
        m_backtargetRPM = SmartDashboard.getNumber("Back Shooter RPM Setpoint", m_backtargetRPM);

        if (waitingForSpinUp && isReadyToFeed(
                isAtTargetSpeed(),
                RobotContainer.isAutoAimFinished,
                RobotContainer.isAutoDistanceFinished,
                alignmentOverride)) {
            runFeederAtTargetSpeed();
            Intake.getInstance().runConveyor();
            waitingForSpinUp = false;
        }

        SmartDashboard.putNumber("Front Shooter Target RPM",  m_fronttargetRPM);
        SmartDashboard.putNumber("Back Shooter Target RPM",  m_backtargetRPM);

        SmartDashboard.putNumber("Shooter Back Actual RPM",  shooterBack.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Front Actual RPM", shooterFront.getEncoder().getVelocity());

        // TalonFX velocity is in rotations/sec — convert to RPM for consistency
        SmartDashboard.putNumber("Feeder Target RPM", FEEDER_TARGET_RPM);
        SmartDashboard.putNumber("Feeder RPM", feederMain.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("Front Shooter Current Draw", shooterFront.getOutputCurrent());
        SmartDashboard.putNumber("Back Shooter Current Draw", shooterBack.getOutputCurrent());
    }

    /**
     * Simulation: flywheel and feeder physics. The SPARK MAX velocity loops
     * are emulated by REVLib; the TalonFX runs Phoenix's own closed-loop sim;
     * the WPILib mechanism models provide the physics both feedback against.
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

        // Flywheels (SPARK MAX, encoder units are native RPM).
        m_frontPhysics.setInput(m_frontSparkSim.getAppliedOutput() * busVoltage);
        m_backPhysics.setInput(m_backSparkSim.getAppliedOutput() * busVoltage);
        m_frontPhysics.update(dt);
        m_backPhysics.update(dt);
        m_frontSparkSim.iterate(m_frontPhysics.getAngularVelocityRPM(), busVoltage, dt);
        m_backSparkSim.iterate(m_backPhysics.getAngularVelocityRPM(), busVoltage, dt);

        // Feeder (Kraken X60 / TalonFX). Phoenix sim provides the motor
        // voltage; we integrate the drum and report rotor position/velocity.
        m_feederSimState.setSupplyVoltage(busVoltage);
        m_feederPhysics.setInputVoltage(m_feederSimState.getMotorVoltage());
        m_feederPhysics.update(dt);
        m_feederSimState.setRawRotorPosition(
            m_feederPhysics.getAngularPositionRotations()
                * SimulationConstants.kFeederGearRatio);
        m_feederSimState.setRotorVelocity(
            m_feederPhysics.getAngularVelocityRPM() / 60.0
                * SimulationConstants.kFeederGearRatio);

        SmartDashboard.putNumber("Sim/Shooter Front Velocity (RPM)", m_frontPhysics.getAngularVelocityRPM());
        SmartDashboard.putNumber("Sim/Shooter Back Velocity (RPM)", m_backPhysics.getAngularVelocityRPM());
    }

    /** Total simulated positive battery current draw of the shooter, in amps. */
    public double getSimulatedCurrentDraw() {
        if (!RobotBase.isSimulation()) {
            return 0.0;
        }
        double frontSupply = m_frontSparkSim.getAppliedOutput() * m_frontSparkSim.getMotorCurrent();
        double backSupply = m_backSparkSim.getAppliedOutput() * m_backSparkSim.getMotorCurrent();
        double feederCurrent =
            DCMotor.getKrakenX60(1).getCurrent(
                m_feederPhysics.getAngularVelocityRadPerSec()
                    * SimulationConstants.kFeederGearRatio,
                m_feederSimState.getMotorVoltage());
        double total = 0.0;
        if (Double.isFinite(frontSupply) && frontSupply > 0.0) {
            total += frontSupply;
        }
        if (Double.isFinite(backSupply) && backSupply > 0.0) {
            total += backSupply;
        }
        if (Double.isFinite(feederCurrent) && feederCurrent > 0.0) {
            total += feederCurrent;
        }
        return total;
    }
}