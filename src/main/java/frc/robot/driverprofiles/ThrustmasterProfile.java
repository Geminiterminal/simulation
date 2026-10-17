package frc.robot.driverprofiles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;

public class ThrustmasterProfile implements DriverControlsProfile {
    public enum DriveSpeedMode {
        STANDARD,
        HALF_SPEED
    }

    public final Joystick joystick;
    private DriveSpeedMode driveSpeedMode = DriveSpeedMode.STANDARD;

    public ThrustmasterProfile(Joystick joystick) {
        this.joystick = joystick;
    }

    @Override
    public double getXSpeed() {
        return -MathUtil.applyDeadband(joystick.getY(), OIConstants.kDriveDeadband);
    }

    @Override
    public double getYSpeed() {
        return -MathUtil.applyDeadband(joystick.getX(), OIConstants.kDriveDeadband);
    }

    @Override
    public double getRotation() {
        return -MathUtil.applyDeadband(joystick.getZ(), OIConstants.kDriveDeadband);
    }

    public void setDriveSpeedMode(DriveSpeedMode driveSpeedMode) {
        this.driveSpeedMode = driveSpeedMode;
    }

    @Override
    public double getDriveSpeedMultiplier() {
        // Joystick throttle is typically in [-1, 1].
        double normalizedThrottle = -((joystick.getThrottle() - 1.0) / 2.0);

        if (driveSpeedMode == DriveSpeedMode.HALF_SPEED) {
            // Map to [0.1, 0.5] for half-speed mode.
            return 0.1 + (0.4 * normalizedThrottle);
        }

        // Standard mode maps to [0.1, 1.0].
        return 0.1 + (0.9 * normalizedThrottle);
    }

    @Override public Trigger shootTrigger() { return new Trigger(() -> joystick.getRawButton(1)); }
    @Override public Trigger headingUpTrigger() { return new Trigger(() -> joystick.getPOV() == 0); }
    @Override public Trigger headingRightTrigger() { return new Trigger(() -> joystick.getPOV() == 90); }
    @Override public Trigger headingDownTrigger() { return new Trigger(() -> joystick.getPOV() == 180); }
    @Override public Trigger headingLeftTrigger() { return new Trigger(() -> joystick.getPOV() == 270); }
    @Override public Trigger autoAimTrigger() { return new Trigger(() -> false); }
    @Override public Trigger disableAutoAimTrigger() { return new Trigger(() -> joystick.getRawButton(6)); }
    @Override public Trigger deployIntakeTrigger() { return new Trigger(() -> joystick.getRawButton(14)); }
    @Override public Trigger stowIntakeTrigger() { return new Trigger(() -> false); }
    @Override public Trigger manualIntakeUpTrigger() { return new Trigger(() -> joystick.getRawButton(5)); }
    @Override public Trigger manualIntakeDownTrigger() { return new Trigger(() -> joystick.getRawButton(10)); }
    @Override public Trigger toggleRollerTrigger() { return new Trigger(() -> joystick.getRawButton(2)); }
    @Override public Trigger thruRightTrench() { return new Trigger(() -> joystick.getRawButton(4));}
    @Override public Trigger thruLeftTrench() { return new Trigger(() -> joystick.getRawButton(3));}
    @Override public Trigger startShotCycle() { return new Trigger(() -> joystick.getRawButton(8));}
    // NEW: Using Button 9 on the Thrustmaster so it doesn't conflict with Deploy Intake (Button 7)
    @Override public Trigger forceReseedTrigger() { return new Trigger(() -> joystick.getRawButton(9)); }
    // Simulation helper: switch between the red and blue alliance.
    @Override public Trigger toggleAllianceTrigger() { return new Trigger(() -> joystick.getRawButton(12)); }


 public void periodic() {
    SmartDashboard.putNumber("velocity",getDriveSpeedMultiplier());
}
}