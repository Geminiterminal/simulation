package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.DriveSubsystem;

public class MoveToTarget extends Command {

  private final DriveSubsystem drive;
  private final Pose2d target;

  // Tunables
  private final double kPTrans = 1.5;
  private final double kPRot   = 3.0;
  private final double maxSpeed = 2.0;
  private final double maxOmega = Math.PI;

  private final double posToleranceMeters  = 0.03;
  private final double rotToleranceRadians = Units.degreesToRadians(2.0);

  public MoveToTarget(DriveSubsystem drive, Pose2d target) {
    this.drive = drive;
    this.target = target;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    Pose2d current = drive.getPose();

    // --- Translation error ---
    Translation2d transError =
        target.getTranslation().minus(current.getTranslation());

    double vx = MathUtil.clamp(kPTrans * transError.getX(),
                               -maxSpeed, maxSpeed);
    double vy = MathUtil.clamp(kPTrans * transError.getY(),
                               -maxSpeed, maxSpeed);

    // --- Rotation error (shortest path automatically wrapped) ---
    Rotation2d rotError =
        target.getRotation().minus(current.getRotation());

    double omega = MathUtil.clamp(kPRot * rotError.getRadians(),
                                  -maxOmega, maxOmega);

    // Field-relative drive
    drive.drive(vx, vy, omega);
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    Pose2d current = drive.getPose();

    double posError = current.getTranslation()
                             .getDistance(target.getTranslation());
    double rotError =
        Math.abs(target.getRotation().minus(current.getRotation()).getRadians());

    return posError < posToleranceMeters &&
           rotError < rotToleranceRadians;
  }
}