package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoDeployAndSpin extends Command {
    private final Intake intake;

    public AutoDeployAndSpin() {
        this.intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // 1. Drop the intake down (the SPARK Flex PID takes over to hold it)
        intake.deploy();
        
        // 2. Turn on the roller motor to suck the balls in
        intake.runRoller(); 
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        // 3. Return TRUE so the command finishes instantly.
        // PathPlanner will immediately move to the next path, leaving the intake down and spinning!
        return true; 
    }

    @Override
    public void end(boolean interrupted) {}
}