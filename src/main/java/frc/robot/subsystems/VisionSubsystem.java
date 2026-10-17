package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  // We can make this 'final' now since it's assigned once and never changes!
  private final AprilTagFieldLayout m_tagLayout;

  public VisionSubsystem() {
    // Automatically loads the official 2026 FIM Welded layout
    m_tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  }

  public AprilTagFieldLayout getTagLayout() {
    return m_tagLayout;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}