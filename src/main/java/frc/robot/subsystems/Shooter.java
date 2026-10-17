package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

// --- CTRE Phoenix 6 Imports ---
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    // --- REV PID + Feedforward gains ---
    private static final double kP = 0.0009;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.000175;

    private static final double FEEDER_TARGET_RPM = 4500.0;
    private static final double FEEDER_TARGET_RPS = FEEDER_TARGET_RPM / 60.0;

    // CTRE velocity PID/feedforward constants for the feeder Kraken.
    private static final double FEEDER_kP = 0.12;
    private static final double FEEDER_kI = 0.0;
    private static final double FEEDER_kD = 0.0;
    private static final double FEEDER_kV = 0.12;

    public Shooter() {

        m_rpmMap.put(215.0, 5000.0);
        SmartDashboard.putNumber("Front Shooter RPM Setpoint", m_fronttargetRPM);
        SmartDashboard.putNumber("Back Shooter RPM Setpoint", m_backtargetRPM);

        // --- Shooter Front Config (REV) ---
        SparkMaxConfig flyConfig = new SparkMaxConfig();
        flyConfig.idleMode(IdleMode.kCoast)
                 .inverted(false)
                 .smartCurrentLimit(60);
        //         .closedLoopRampRate(0.4);
        flyConfig.closedLoop
            .pid(kP, kI, kD)
            .velocityFF(kF);

        // --- Shooter Back Config (REV) ---
        SparkMaxConfig flyConfig2 = new SparkMaxConfig();
        flyConfig2.idleMode(IdleMode.kCoast)
                  .inverted(true)
                  .smartCurrentLimit(60);
        //          .closedLoopRampRate(0.4);
        flyConfig2.closedLoop
            .pid(kP, kI, kD)
            .velocityFF(kF);

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

        frontController.setReference(m_fronttargetRPM, ControlType.kVelocity);
        backController.setReference(m_backtargetRPM,   ControlType.kVelocity);

        setFeederDutyCycle(0.0); // Hold feeder until up to speed (periodic handles handoff)
    }

    /** Spins up both shooter wheels without starting the feeder or conveyor. */
    public void spinUpToTargetPercentage(double targetPercentage) {
        frontController.setReference(m_fronttargetRPM * targetPercentage, ControlType.kVelocity);
        backController.setReference(m_backtargetRPM * targetPercentage, ControlType.kVelocity);
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
    }}