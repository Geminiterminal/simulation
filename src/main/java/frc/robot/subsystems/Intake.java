package frc.robot.subsystems;

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

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    // --- Deploy PID & Position Constants ---
    private static final double DEPLOY_kP = 0.3;
    private static final double DEPLOY_kI = 0.0;
    private static final double DEPLOY_kD = 0.0;

    // --- Conveyor Velocity PID Constants ---
    private static final double CONVEYOR_TARGET_RPM = 4500.0;
    private static final double CONVEYOR_kP = 0.0001;
    private static final double CONVEYOR_kI = 0.0;
    private static final double CONVEYOR_kD = 0.0;
    private static final double CONVEYOR_kV = 0.0;

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

    private final SparkClosedLoopController deployController;
    private final SparkClosedLoopController conveyorController;

    private boolean cyclingForShot = false;
    private boolean rollerRunning = false;
    private double deployTargetPosition = DEPLOY_POSITION;
    private double shotCycleDwellStartTime = -1.0;

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

        // 3. Conveyor Motor Config
        SparkMaxConfig conveyorConfig = new SparkMaxConfig();
        conveyorConfig.idleMode(IdleMode.kBrake)
                      .inverted(false)
                      .smartCurrentLimit(40);
        conveyorConfig.closedLoop.pid(CONVEYOR_kP, CONVEYOR_kI, CONVEYOR_kD);
        conveyorConfig.closedLoop.feedForward.kV(CONVEYOR_kV);
        conveyorMotor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    public void runConveyor()     { conveyorController.setReference(CONVEYOR_TARGET_RPM, ControlType.kVelocity); }
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
}