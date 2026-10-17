package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class SimpleAuto extends SequentialCommandGroup {

    public SimpleAuto() {
        Shooter shooter = Shooter.getInstance();

        addCommands(
            // Step 1: Wait 0.5s for robot to settle at start of auto
            new WaitCommand(0.5),

            // Step 2: Spin up flywheels + run conveyor + feed balls for 3 seconds
            // Change the withTimeout value if you need more or less shooting time
            new StartEndCommand(
                shooter::start,
                shooter::stopAll,
                shooter
            ).withTimeout(3.0)
        );
    }
}