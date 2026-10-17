/* package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final SparkMax climberMotor;
    private final RelativeEncoder climberEncoder;

    // --- CAN ID ---
    private static final int CLIMBER_MOTOR_ID = 35;

    // --- SPEEDS ---
    // extend() = arm goes UP   = positive motor output
    // retract() = arm goes DOWN = negative motor output
    private static final double EXTEND_SPEED  =  0.50; // UP
    private static final double RETRACT_SPEED = -0.50; // DOWN

    // --- SOFT LIMITS ---
    private static final float TOP_LIMIT    = 57.426f;
    private static final float BOTTOM_LIMIT = 0.0f;

    public Climber() {
        climberMotor   = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();

        SparkMaxConfig climberConfig = new SparkMaxConfig();
        climberConfig.idleMode(IdleMode.kBrake);
        climberConfig.inverted(false);

        climberConfig.softLimit.forwardSoftLimit(TOP_LIMIT);
        climberConfig.softLimit.forwardSoftLimitEnabled(true);
        climberConfig.softLimit.reverseSoftLimit(BOTTOM_LIMIT);
        climberConfig.softLimit.reverseSoftLimitEnabled(true);

        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Robot MUST start with climber fully retracted every single power cycle!
        climberEncoder.setPosition(0);
    }

    // --- Singleton ---
    private static class InstanceHolder {
        private static final Climber INSTANCE = new Climber();
    }

    public static Climber getInstance() {
        return InstanceHolder.INSTANCE;
    }

    // --- MOVEMENT ---
    public void extend()  { climberMotor.set(EXTEND_SPEED);  }  // UP
    public void retract() { climberMotor.set(RETRACT_SPEED); }  // DOWN
    public void stop()    { climberMotor.set(0);              }

    // --- POSITION --- 
    // Used by AutoClimb to know when to switch phases
    public double getPosition() {
        return climberEncoder.getPosition();
    }

    // --- ENCODER RESET ---
    public void resetEncoder() {
        climberEncoder.setPosition(0);
        System.out.println("Climber encoder reset to 0 — make sure arm is fully DOWN!");
    }

    @Override
    public void periodic() {
        double pos = climberEncoder.getPosition();
        SmartDashboard.putNumber("Climber Position",   pos);
       // SmartDashboard.putBoolean("Climber At Top",    pos >= TOP_LIMIT);
       // SmartDashboard.putBoolean("Climber At Bottom", pos <= BOTTOM_LIMIT);

        if (pos < -1.0) {
            System.out.println("WARNING: Climber encoder is negative! Robot may not have started fully retracted.");
        }
    }
} */