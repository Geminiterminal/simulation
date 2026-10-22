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

    /**
     * Switches between the red and blue alliance. Simulation only - on the real
     * robot the alliance comes from the FMS.
     */
    Trigger toggleAllianceTrigger();

    /** Holds a SysId quasistatic forward test on the selected mechanism. */
    Trigger sysIdQuasistaticForwardTrigger();

    /** Holds a SysId quasistatic reverse test on the selected mechanism. */
    Trigger sysIdQuasistaticReverseTrigger();

    /** Holds a SysId dynamic forward test on the selected mechanism. */
    Trigger sysIdDynamicForwardTrigger();

    /** Holds a SysId dynamic reverse test on the selected mechanism. */
    Trigger sysIdDynamicReverseTrigger();
}