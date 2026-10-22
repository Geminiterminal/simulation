package frc.robot.driverprofiles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;

public class XboxHanifStyleProfile implements DriverControlsProfile {
    private final CommandXboxController controller;

    public XboxHanifStyleProfile(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public double getXSpeed() {
        return -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband);
    }

    @Override
    public double getYSpeed() {
        return -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband);
    }

    @Override
    public double getRotation() {
        return -MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband);
    }

    @Override
    public double getDriveSpeedMultiplier() {
        // You can map this to right trigger depth or just return 1.0 for standard Xbox driving
        return 1.0; 
    }

    @Override public Trigger shootTrigger() { return controller.rightTrigger(); }
    @Override public Trigger headingUpTrigger() { return controller.povUp(); }
    @Override public Trigger headingRightTrigger() { return controller.povRight(); }
    @Override public Trigger headingDownTrigger() { return controller.povDown(); }
    @Override public Trigger headingLeftTrigger() { return controller.povLeft(); }
    @Override public Trigger autoAimTrigger() { return controller.leftTrigger(); }
    @Override public Trigger disableAutoAimTrigger() { return new Trigger(() -> false); }
    @Override public Trigger deployIntakeTrigger() { return controller.a().and(controller.back().negate()); }
    @Override public Trigger stowIntakeTrigger() { return controller.b().and(controller.back().negate()); }
    @Override public Trigger manualIntakeUpTrigger() { return controller.rightBumper(); }
    @Override public Trigger manualIntakeDownTrigger() { return controller.leftBumper(); }
    @Override public Trigger toggleRollerTrigger() { return controller.x().and(controller.back().negate()); }
    
    // Xbox Start requests a vision pose correction.
    @Override public Trigger forceReseedTrigger() { return controller.start(); }
    // Simulation helper: switch between the red and blue alliance.
    @Override public Trigger toggleAllianceTrigger() { return controller.y().and(controller.back().negate()); }

    // Hold Back (View) + A/B/X/Y to run SysId routines on Xbox without overlapping teleop buttons.
    @Override public Trigger sysIdQuasistaticForwardTrigger() { return controller.back().and(controller.a()); }
    @Override public Trigger sysIdQuasistaticReverseTrigger() { return controller.back().and(controller.b()); }
    @Override public Trigger sysIdDynamicForwardTrigger() { return controller.back().and(controller.x()); }
    @Override public Trigger sysIdDynamicReverseTrigger() { return controller.back().and(controller.y()); }
@Override
public Trigger thruRightTrench() {
    return new Trigger(() -> false);
}
@Override
public Trigger thruLeftTrench() {
    return new Trigger(() -> false);
}
@Override
public Trigger startShotCycle() {
    return new Trigger(() -> false);
} }