package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.DriveSubsystem;

public class Auto_ScoreLoadScore extends SequentialCommandGroup {

  public Auto_ScoreLoadScore(DriveSubsystem drive) {
    addCommands(
        // 1. Move to (3, 4.75, 90°)
        new MoveToTarget(
            drive,
            new Pose2d(3.0, 4.75, Rotation2d.fromDegrees(90))
        ),

        // Pause 0.5 sec
        new WaitCommand(0.5),

        // 2. Move to (1, 2.25, 225°)
        new MoveToTarget(
            drive,
            new Pose2d(1.0, 2.25, Rotation2d.fromDegrees(225))
        ),

        // Pause 0.5 sec
        new WaitCommand(0.5),

        // 3. Move back to (3, 4.75, 90°)
        new MoveToTarget(
            drive,
            new Pose2d(3.0, 4.75, Rotation2d.fromDegrees(90))
        )
    );
  }
}