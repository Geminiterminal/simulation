package frc.robot.driverprofiles;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControlsProfile {
    double getXSpeed();
    double getYSpeed();
    double getRotation();
    double getDriveSpeedMultiplier();

    Trigger shootTrigger();
    Trigger headingUpTrigger();
    Trigger headingRightTrigger();
    Trigger headingDownTrigger();
    Trigger headingLeftTrigger();
    Trigger autoAimTrigger();
    Trigger disableAutoAimTrigger();
    Trigger deployIntakeTrigger();
    Trigger stowIntakeTrigger();
    Trigger manualIntakeUpTrigger();
    Trigger manualIntakeDownTrigger();
    Trigger toggleRollerTrigger();
    Trigger thruRightTrench();
    Trigger thruLeftTrench();
    Trigger startShotCycle();
    
    // NEW: Dynamic trigger for snapping odometry to vision targets
    Trigger forceReseedTrigger();
}