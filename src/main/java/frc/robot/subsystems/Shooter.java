package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.AutoTuning;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.ShotGate;
import frc.robot.ShotInputs;
import frc.robot.ShotMath;
import frc.robot.ShooterTuning;

import java.util.function.Supplier;

/**
 * Two-NEO differential flywheel plus a Kraken X60 feeder.
 *
 * <p>The shooter is commanded as a <em>shot cycle</em> rather than a raw
 * setpoint:
 *
 * <ol>
 *   <li>{@link #start()} enables the cycle and clears the readiness gate.
 *   <li>Every loop the flywheel targets are refreshed (fixed speed from the
 *       dashboard, or distance-table + trims when
 *       {@link ShooterTuning#useDistanceTable()} is on) and pushed to the
 *       SPARK MAX closed loops.
 *   <li>{@link ShotGate} answers "may I feed right now?" from flywheel speed,
 *       aim error, distance error and chassis motion - with settle times and
 *       hysteresis so the answer does not chatter.
 *   <li>The feeder runs only while that answer is yes, so every game piece
 *       leaves the wheel at the same speed, including the fifth ball of a
 *       burst. Between balls the flywheel dips; the gate pauses the feeder and
 *       resumes when the wheel is back on target.
 * </ol>
 *
 * <p>See SHOOTER_ACCURACY.md for the reasoning, the calibration procedure and
 * the telemetry that backs each decision.
 */
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
    private final VoltageOut feederVoltageRequest = new VoltageOut(0.0);

    // --- State ---
    private boolean systemEnabled    = false;
    private boolean alignmentOverride = false;

    // --- CAN IDs ---
    private static final int SHOOTER_ID_1 = 5;
    private static final int SHOOTER_ID_2 = 6;
    private static final int FEEDER_ID_1  = 7;

    // --- Velocity Setpoints ---
    public double m_fronttargetRPM       = ShooterConstants.kDefaultFrontRpm;
    public double m_backtargetRPM       = ShooterConstants.kDefaultBackRpm;

    // --- Dashboard keys ---
    private static final String FRONT_RPM_KEY = "Front Shooter RPM Setpoint";
    private static final String BACK_RPM_KEY  = "Back Shooter RPM Setpoint";
    private static final String[] TABLE_DISTANCE_KEYS =
        buildTableKeys("Shooter Table/%d Distance (m)");
    private static final String[] TABLE_RPM_KEYS =
        buildTableKeys("Shooter Table/%d Front RPM");

    // --- Targeting state ---
    private final ShotGate m_shotGate = new ShotGate();
    private Supplier<ShotInputs> m_shotInputsSupplier = () -> ShotInputs.NONE;
    private ShotGate.Status m_status = ShotGate.NOT_READY;
    private ShotInputs m_lastInputs = ShotInputs.NONE;

    private final double[] m_tableDistances = new double[ShooterConstants.kTablePoints];
    private final double[] m_tableFrontRpm = new double[ShooterConstants.kTablePoints];
    private boolean m_tableValid = true;
    private double m_lastKnownDistanceMeters = Double.NaN;
    private double m_tableRpm = Double.NaN;
    private double m_motionCompRpm = 0.0;
    private double m_lastDashboardFrontRpm = ShooterConstants.kDefaultFrontRpm;
    private double m_lastDashboardBackRpm = ShooterConstants.kDefaultBackRpm;
    private double m_lastLoopTimestampSeconds = Double.NaN;

    // --- Feed bookkeeping / telemetry ---
    private boolean m_feedActive = false;
    private int m_feedBursts = 0;
    private String m_feedStatus = "OFF";

    // --- REV PID + Feedforward gains (SPARK MAX + NEO 1.1) ---
    // REVLib 2026 FeedForwardConfig voltage gains are in Volts per velocity
    // unit, and the encoder reports native RPM, so these are V/RPM.
    private static final double kP = 0.0009;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    // Theoretical back-EMF only (12 V / free speed). The missing piece is the
    // constant friction term: a real flywheel needs a few extra tenths of a
    // volt, and without it P has to make up the difference and the wheel
    // settles below target. Identify kS (and a true kV) with the SysId routine
    // below, then enter them here. See SHOOTER_ACCURACY.md.
    private static final double kV = 12.0 / NeoMotorConstants.kFreeSpeedRpm;
    private static final double kS = 0.0;

    private static final double FEEDER_TARGET_RPM = 4500.0;
    private static final double FEEDER_TARGET_RPS = FEEDER_TARGET_RPM / 60.0;

    // CTRE velocity PID/feedforward constants for the feeder Kraken.
    private static final double FEEDER_kP = 0.12;
    private static final double FEEDER_kI = 0.0;
    private static final double FEEDER_kD = 0.0;
    private static final double FEEDER_kV = 0.12;
    // Static friction feedforward (volts). 0 preserves the previous behavior;
    // fill it in from SysId so the feeder actually reaches FEEDER_TARGET_RPM.
    private static final double FEEDER_kS = 0.0;

    // --- Simulation members (null on the real robot) ---
    private final DCMotor m_flywheelGearbox = DCMotor.getNEO(1);
    private SparkMaxSim m_frontSparkSim;
    private SparkMaxSim m_backSparkSim;
    private FlywheelSim m_frontPhysics;
    private FlywheelSim m_backPhysics;
    private TalonFXSimState m_feederSimState;
    private DCMotorSim m_feederPhysics;

    public Shooter() {

        ShooterTuning.publishDefaults();
        publishTableDefaults();

        SmartDashboard.putNumber(FRONT_RPM_KEY, m_fronttargetRPM);
        SmartDashboard.putNumber(BACK_RPM_KEY, m_backtargetRPM);
        m_lastDashboardFrontRpm = m_fronttargetRPM;
        m_lastDashboardBackRpm = m_backtargetRPM;

        // --- Shooter Front Config (REV) ---
        SparkMaxConfig flyConfig = new SparkMaxConfig();
        flyConfig.idleMode(IdleMode.kCoast)
                 .inverted(false)
                 .smartCurrentLimit(60);
        flyConfig.closedLoop
            .pid(kP, kI, kD);
        flyConfig.closedLoop.feedForward
            .kV(kV).kS(kS);

        // --- Shooter Back Config (REV) ---
        SparkMaxConfig flyConfig2 = new SparkMaxConfig();
        flyConfig2.idleMode(IdleMode.kCoast)
                  .inverted(true)
                  .smartCurrentLimit(60);
        flyConfig2.closedLoop
            .pid(kP, kI, kD);
        flyConfig2.closedLoop.feedForward
            .kV(kV).kS(kS);

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
        feederConfig.Slot0.kS = FEEDER_kS;

        feederMain.getConfigurator().apply(feederConfig);
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            feederMain.getPosition(),
            feederMain.getVelocity(),
            feederMain.getMotorVoltage());

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

        // Build SysId routines only after all motor fields have been initialized.
        m_flywheelSysIdRoutine = createFlywheelSysIdRoutine();
        m_feederSysIdRoutine = createFeederSysIdRoutine();
    }

    // --- Singleton ---
    private static class InstanceHolder {
        private static final Shooter INSTANCE = new Shooter();
    }

    public static Shooter getInstance() {
        return InstanceHolder.INSTANCE;
    }

    private static String[] buildTableKeys(String format) {
        String[] keys = new String[ShooterConstants.kTablePoints];
        for (int i = 0; i < keys.length; i++) {
            keys[i] = String.format(format, i + 1);
        }
        return keys;
    }

    private static void publishTableDefaults() {
        for (int i = 0; i < ShooterConstants.kTablePoints; i++) {
            ShooterTuning.putNumberDefault(
                TABLE_DISTANCE_KEYS[i], ShooterConstants.kDefaultTableDistancesMeters[i]);
            ShooterTuning.putNumberDefault(
                TABLE_RPM_KEYS[i], ShooterConstants.kDefaultTableFrontRpm[i]);
        }
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

    /**
     * Supplies the shooter with the hub geometry and chassis motion for this
     * loop. {@link frc.robot.RobotContainer} installs this once at boot; until then the
     * shooter behaves exactly as it did before (fixed speed, no aim gating).
     */
    public void setShotInputSupplier(Supplier<ShotInputs> supplier) {
        m_shotInputsSupplier = (supplier == null) ? () -> ShotInputs.NONE : supplier;
    }

    /** Enables the shot cycle. The feeder stays off until the gate opens. */
    public void start() {
        systemEnabled = true;
        m_shotGate.reset();
        setFeederDutyCycle(0.0);
        applyTargets();
    }

    public void stopAll() {
        systemEnabled    = false;
        alignmentOverride = false;

        shooterFront.set(0);
        shooterBack.set(0);
        setFeederDutyCycle(0.0);

        Intake.getInstance().stopConveyor();
        Intake.getInstance().stopShotCycle();

        m_shotGate.reset();
        m_status = ShotGate.NOT_READY;
        m_feedActive = false;
        m_feedStatus = "OFF";
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
        m_lastDashboardFrontRpm = rpm;
        SmartDashboard.putNumber(FRONT_RPM_KEY, rpm);
    }

    public void setBackTargetRPM(double rpm) {
        m_backtargetRPM = rpm;
        m_lastDashboardBackRpm = rpm;
        SmartDashboard.putNumber(BACK_RPM_KEY, rpm);
    }

    /** True when both flywheels are within the tuning tolerance of their target. */
    public boolean isAtTargetSpeed() {
        double tolerance = ShooterTuning.speedToleranceFraction();
        return isSpeedWithinTolerance(
                m_fronttargetRPM, shooterFront.getEncoder().getVelocity(), tolerance)
            && isSpeedWithinTolerance(
                m_backtargetRPM, shooterBack.getEncoder().getVelocity(), tolerance);
    }

    /**
     * Percentage-band speed check. A zero (or NaN) target is never "at speed",
     * which is what made a stopped wheel look ready in the old latch.
     */
    static boolean isSpeedWithinTolerance(
            double targetRpm, double measuredRpm, double toleranceFraction) {
        double ratio = ShotMath.speedRatio(measuredRpm, targetRpm);
        return Double.isFinite(ratio)
            && Math.abs(ratio - 1.0) <= Math.abs(toleranceFraction);
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
        double now = Timer.getFPGATimestamp();
        double dt = Double.isFinite(m_lastLoopTimestampSeconds)
            ? now - m_lastLoopTimestampSeconds
            : SimulationConstants.kLoopPeriodSeconds;
        m_lastLoopTimestampSeconds = now;

        ShotInputs inputs = m_shotInputsSupplier.get();
        if (inputs == null) {
            inputs = ShotInputs.NONE;
        }
        m_lastInputs = inputs;

        updateTargets(inputs);
        if (systemEnabled) {
            applyTargets();
        }

        m_status = m_shotGate.update(dt, gateInputs(inputs), gateConfig());
        boolean readyToFeed = isReadyToFeed(
            m_status.speedReady(), m_status.aimed(), m_status.atDistance(), alignmentOverride);
        boolean feedAllowed = systemEnabled
            && readyToFeed
            && (alignmentOverride || m_status.motionOk());

        if (systemEnabled) {
            updateFeeding(feedAllowed);
        } else {
            // Leave the feeder alone: SysId drives it directly with a voltage.
            m_feedActive = false;
            m_feedStatus = "OFF";
        }
        publishTelemetry(feedAllowed);

        SmartDashboard.putNumber("Shooter Back Actual RPM",  shooterBack.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Front Actual RPM", shooterFront.getEncoder().getVelocity());

        // TalonFX velocity is in rotations/sec — convert to RPM for consistency
        SmartDashboard.putNumber("Feeder Target RPM", FEEDER_TARGET_RPM);
        SmartDashboard.putNumber("Feeder RPM", feederMain.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("Front Shooter Current Draw", shooterFront.getOutputCurrent());
        SmartDashboard.putNumber("Back Shooter Current Draw", shooterBack.getOutputCurrent());
    }

    // -------------------------------------------------------------------------
    // TARGETING
    // -------------------------------------------------------------------------

    /**
     * Refreshes the flywheel targets for this loop.
     *
     * <p>When the distance table is on, the target comes from the current shot
     * distance (plus trim and optional motion compensation). Otherwise the
     * dashboard setpoints are used - but an operator edit is only accepted when
     * the value actually changes, so a command calling
     * {@link #setFrontTargetRPM(double)} is no longer overwritten by the stale
     * dashboard number on the next loop.
     */
    private void updateTargets(ShotInputs inputs) {
        m_motionCompRpm = 0.0;
        m_tableRpm = Double.NaN;

        if (!ShooterTuning.useDistanceTable()) {
            updateManualTargets();
            return;
        }

        double distance = Double.isFinite(inputs.distanceMeters())
            ? inputs.distanceMeters()
            : m_lastKnownDistanceMeters;
        if (!Double.isFinite(distance)) {
            updateManualTargets();
            return;
        }
        m_lastKnownDistanceMeters = distance;

        refreshTable();
        if (!m_tableValid) {
            updateManualTargets();
            return;
        }

        double front = ShotMath.interpolate(m_tableDistances, m_tableFrontRpm, distance);
        if (!Double.isFinite(front)) {
            updateManualTargets();
            return;
        }

        front *= ShooterTuning.tableTrim();
        double gain = ShooterTuning.motionCompensationGain();
        if (gain > 0.0) {
            m_motionCompRpm = -gain * inputs.closingSpeedMps();
            front += m_motionCompRpm;
        }
        front = clampRpm(front);
        m_tableRpm = front;

        m_fronttargetRPM = front;
        m_backtargetRPM = clampRpm(front * ShooterTuning.spinRatio());
    }

    private void updateManualTargets() {
        double dashFront = SmartDashboard.getNumber(FRONT_RPM_KEY, m_lastDashboardFrontRpm);
        if (Double.isFinite(dashFront) && dashFront != m_lastDashboardFrontRpm) {
            m_fronttargetRPM = dashFront;
            m_lastDashboardFrontRpm = dashFront;
        }
        double dashBack = SmartDashboard.getNumber(BACK_RPM_KEY, m_lastDashboardBackRpm);
        if (Double.isFinite(dashBack) && dashBack != m_lastDashboardBackRpm) {
            m_backtargetRPM = dashBack;
            m_lastDashboardBackRpm = dashBack;
        }
    }

    private void refreshTable() {
        boolean valid = true;
        for (int i = 0; i < ShooterConstants.kTablePoints; i++) {
            m_tableDistances[i] = SmartDashboard.getNumber(
                TABLE_DISTANCE_KEYS[i], ShooterConstants.kDefaultTableDistancesMeters[i]);
            m_tableFrontRpm[i] = SmartDashboard.getNumber(
                TABLE_RPM_KEYS[i], ShooterConstants.kDefaultTableFrontRpm[i]);
            if (!Double.isFinite(m_tableDistances[i]) || !Double.isFinite(m_tableFrontRpm[i])) {
                valid = false;
            }
        }
        m_tableValid = valid && ShotMath.isStrictlyAscending(m_tableDistances);
    }

    private void applyTargets() {
        if (Double.isFinite(m_fronttargetRPM)) {
            frontController.setSetpoint(clampRpm(m_fronttargetRPM), ControlType.kVelocity);
        }
        if (Double.isFinite(m_backtargetRPM)) {
            backController.setSetpoint(clampRpm(m_backtargetRPM), ControlType.kVelocity);
        }
    }

    private static double clampRpm(double rpm) {
        return MathUtil.clamp(rpm, 0.0, NeoMotorConstants.kFreeSpeedRpm);
    }

    // -------------------------------------------------------------------------
    // GATE
    // -------------------------------------------------------------------------

    private ShotGate.Inputs gateInputs(ShotInputs inputs) {
        return new ShotGate.Inputs(
            worstSpeedRatio(),
            inputs.headingErrorDeg(),
            distanceErrorMeters(inputs.distanceMeters()),
            inputs.linearSpeedMps(),
            inputs.yawRateDps());
    }

    private ShotGate.Config gateConfig() {
        return new ShotGate.Config(
            ShooterTuning.speedToleranceFraction(),
            ShooterTuning.speedSettleSeconds(),
            AutoTuning.aimHeadingToleranceDeg(),
            ShooterTuning.aimHysteresisDeg(),
            ShooterTuning.aimSettleSeconds(),
            AutoTuning.aimDistanceToleranceMeters(),
            Units.inchesToMeters(ShooterTuning.distanceHysteresisInches()),
            ShooterTuning.maxFireSpeedMps(),
            ShooterTuning.maxFireYawRateDps());
    }

    /** Measured/target ratio of whichever flywheel is furthest from its target. */
    private double worstSpeedRatio() {
        double front = ShotMath.speedRatio(
            shooterFront.getEncoder().getVelocity(), m_fronttargetRPM);
        double back = ShotMath.speedRatio(
            shooterBack.getEncoder().getVelocity(), m_backtargetRPM);
        if (!Double.isFinite(front) || !Double.isFinite(back)) {
            return Double.NaN;
        }
        return Math.abs(front - 1.0) >= Math.abs(back - 1.0) ? front : back;
    }

    /**
     * Distance gate. Returns NaN (gate disabled) in table mode, because the
     * table already adapts the shot to whatever distance the robot is at.
     */
    private double distanceErrorMeters(double distanceMeters) {
        if (ShooterTuning.useDistanceTable()) {
            return Double.NaN;
        }
        if (!Double.isFinite(distanceMeters)) {
            return 0.0;
        }
        return distanceMeters - Units.inchesToMeters(AutoTuning.aimTargetDistanceInches());
    }

    private void updateFeeding(boolean feedAllowed) {
        if (!systemEnabled) {
            setFeederDutyCycle(0.0);
            m_feedActive = false;
            m_feedStatus = "OFF";
            return;
        }

        // Keep game pieces staged at the feeder for the whole enabled period;
        // only the feeder itself is gated so the flywheel can recover between
        // balls without the upstream conveyor having to spin back up.
        Intake.getInstance().runConveyor();

        if (feedAllowed) {
            runFeederAtTargetSpeed();
            if (!m_feedActive) {
                m_feedBursts++;
            }
            m_feedActive = true;
            m_feedStatus = "FEEDING";
        } else {
            setFeederDutyCycle(0.0);
            m_feedActive = false;
            m_feedStatus = blockedReason();
        }
    }

    /** Human-readable reason the feeder is held, shown on the dashboard. */
    private String blockedReason() {
        if (!m_status.speedReady()) {
            return String.format("SPINNING UP (%.0f RPM)", speedErrorRpm());
        }
        if (!alignmentOverride) {
            if (!m_status.aimed()) {
                return String.format("AIMING (%.1f deg)", m_lastInputs.headingErrorDeg());
            }
            if (!m_status.atDistance()) {
                return "RANGING";
            }
            if (!m_status.motionOk()) {
                return String.format(
                    "MOVING (%.2f m/s, %.0f deg/s)",
                    m_lastInputs.linearSpeedMps(), m_lastInputs.yawRateDps());
            }
        }
        return "WAITING";
    }

    private double speedErrorRpm() {
        double front = Math.abs(shooterFront.getEncoder().getVelocity() - m_fronttargetRPM);
        double back = Math.abs(shooterBack.getEncoder().getVelocity() - m_backtargetRPM);
        return Double.isFinite(front) && Double.isFinite(back) ? Math.max(front, back) : Double.NaN;
    }

    private void publishTelemetry(boolean feedAllowed) {
        SmartDashboard.putNumber("Front Shooter Target RPM", m_fronttargetRPM);
        SmartDashboard.putNumber("Back Shooter Target RPM",  m_backtargetRPM);

        SmartDashboard.putBoolean("Shooter/Ready To Fire", feedAllowed);
        SmartDashboard.putString("Shooter/Feed Status", m_feedStatus);
        SmartDashboard.putBoolean("Shooter/Speed Ready", m_status.speedReady());
        SmartDashboard.putBoolean("Shooter/Aimed", m_status.aimed());
        SmartDashboard.putBoolean("Shooter/At Distance", m_status.atDistance());
        SmartDashboard.putBoolean("Shooter/Motion OK", m_status.motionOk());

        SmartDashboard.putNumber("Shooter/Front RPM Error", shooterFront.getEncoder().getVelocity() - m_fronttargetRPM);
        SmartDashboard.putNumber("Shooter/Back RPM Error", shooterBack.getEncoder().getVelocity() - m_backtargetRPM);
        SmartDashboard.putNumber("Shooter/Heading Error (deg)", m_lastInputs.headingErrorDeg());
        SmartDashboard.putNumber("Shooter/Distance (m)", m_lastInputs.distanceMeters());
        SmartDashboard.putNumber("Shooter/Feed Bursts", m_feedBursts);
        SmartDashboard.putString("Shooter/Target Source", ShooterTuning.useDistanceTable() ? "TABLE" : "MANUAL");

        SmartDashboard.putBoolean("Shooter/Table Valid", m_tableValid);
        SmartDashboard.putNumber("Shooter/Table RPM", m_tableRpm);
        SmartDashboard.putNumber("Shooter/Table Trim", ShooterTuning.tableTrim());
        SmartDashboard.putNumber("Shooter/Spin Ratio", ShooterTuning.spinRatio());
        SmartDashboard.putNumber("Shooter/Motion Comp (RPM)", m_motionCompRpm);
        if (m_tableValid && m_tableFrontRpm.length > 1 && Double.isFinite(m_lastKnownDistanceMeters)) {
            SmartDashboard.putNumber(
                "Shooter/Motion Gain (suggested)",
                ShotMath.suggestedMotionGain(
                    m_tableDistances,
                    m_tableFrontRpm,
                    m_lastKnownDistanceMeters,
                    ShooterConstants.kNominalTimeOfFlightSeconds));
        }
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

    // =========================================================================
    // --- SYSTEM IDENTIFICATION (WPILib SysId) ---
    // =========================================================================

    private final MutVoltage m_sysIdVoltage = Volts.mutable(0.0);
    private final MutAngle m_sysIdAngle = Rotations.mutable(0.0);
    private final MutAngularVelocity m_sysIdVelocity = RotationsPerSecond.mutable(0.0);

    private final SysIdRoutine m_flywheelSysIdRoutine;
    private final SysIdRoutine m_feederSysIdRoutine;

    /**
     * Characterizes both the front and back shooter NEO flywheels simultaneously while keeping the
     * feeder off. Note: WPILib SysId reports angular gains in {@code Rotations/sec}; divide
     * identified {@code kV} (and {@code kP}) by {@code 60.0} when entering REVLib's native RPM
     * closed-loop configuration.
     */
    private SysIdRoutine createFlywheelSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.0).per(Second),
                Volts.of(7.0),
                Seconds.of(10.0)),
            new SysIdRoutine.Mechanism(
                voltage -> setSysIdFlywheelVolts(voltage.in(Volts)),
                log -> {
                    log.motor("shooter-front")
                        .voltage(m_sysIdVoltage.mut_replace(getSparkAppliedVolts(shooterFront), Volts))
                        .angularPosition(
                            m_sysIdAngle.mut_replace(shooterFront.getEncoder().getPosition(), Rotations))
                        .angularVelocity(
                            m_sysIdVelocity.mut_replace(shooterFront.getEncoder().getVelocity(), RPM));
                    log.motor("shooter-back")
                        .voltage(m_sysIdVoltage.mut_replace(getSparkAppliedVolts(shooterBack), Volts))
                        .angularPosition(
                            m_sysIdAngle.mut_replace(shooterBack.getEncoder().getPosition(), Rotations))
                        .angularVelocity(
                            m_sysIdVelocity.mut_replace(shooterBack.getEncoder().getVelocity(), RPM));
                },
                this,
                "shooter-flywheels"));
    }

    /**
     * Characterizes the Kraken X60 feeder TalonFX in rotations and rotations/sec (matching Phoenix 6
     * {@code Slot0} gain units 1:1).
     */
    private SysIdRoutine createFeederSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.0).per(Second),
                Volts.of(6.0),
                Seconds.of(10.0)),
            new SysIdRoutine.Mechanism(
                voltage -> setSysIdFeederVolts(voltage.in(Volts)),
                log -> {
                    log.motor("shooter-feeder")
                        .voltage(
                            m_sysIdVoltage.mut_replace(
                                feederMain.getMotorVoltage().getValueAsDouble(), Volts))
                        .angularPosition(
                            m_sysIdAngle.mut_replace(
                                feederMain.getPosition().getValueAsDouble(), Rotations))
                        .angularVelocity(
                            m_sysIdVelocity.mut_replace(
                                feederMain.getVelocity().getValueAsDouble(), RotationsPerSecond));
                },
                this,
                "shooter-feeder"));
    }

    private static double getSparkAppliedVolts(SparkMax spark) {
        return MAXSwerveModule.computeAppliedVolts(
            spark.getAppliedOutput(),
            spark.getBusVoltage(),
            RobotController.getBatteryVoltage(),
            RobotBase.isSimulation());
    }

    private void setSysIdFlywheelVolts(double volts) {
        systemEnabled = false;
        setFeederDutyCycle(0.0);
        shooterFront.setVoltage(volts);
        shooterBack.setVoltage(volts);
    }

    private void setSysIdFeederVolts(double volts) {
        systemEnabled = false;
        shooterFront.set(0.0);
        shooterBack.set(0.0);
        feederMain.setControl(feederVoltageRequest.withOutput(volts));
    }

    /** Returns true once both coast-mode flywheels have spun down near rest (< 60 RPM). */
    public boolean areFlywheelsStopped() {
        return Math.abs(shooterFront.getEncoder().getVelocity()) < 60.0
            && Math.abs(shooterBack.getEncoder().getVelocity()) < 60.0;
    }

    /** Returns true once the feeder motor velocity is near rest (< 1 rot/s). */
    public boolean isFeederStopped() {
        return Math.abs(feederMain.getVelocity().getValueAsDouble()) < 1.0;
    }

    public Command sysIdFlywheelsQuasistatic(SysIdRoutine.Direction direction) {
        return m_flywheelSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdFlywheelsDynamic(SysIdRoutine.Direction direction) {
        return m_flywheelSysIdRoutine.dynamic(direction);
    }

    public Command sysIdFeederQuasistatic(SysIdRoutine.Direction direction) {
        return m_feederSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdFeederDynamic(SysIdRoutine.Direction direction) {
        return m_feederSysIdRoutine.dynamic(direction);
    }
}
