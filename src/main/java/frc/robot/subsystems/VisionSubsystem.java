package frc.robot.subsystems;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Owns the AprilTag field layout and, in simulation, the PhotonVision camera
 * simulation.
 *
 * <p>The simulation (PhotonVision "Simulation" docs): a {@link VisionSystemSim}
 * holds the field's AprilTags and one {@link PhotonCameraSim} per real camera,
 * each with calibrated intrinsics, latency and calibration-error statistics.
 * Every loop it is given the robot's ground-truth pose (from the drivetrain
 * physics) and publishes synthetic camera frames onto the same NetworkTables
 * topics the real PhotonVision coprocessors use. The robot's own
 * {@code PhotonCamera} objects and {@code PhotonPoseEstimator} fusion code
 * therefore run unchanged in simulation.
 */
public class VisionSubsystem extends SubsystemBase {

  // We can make this 'final' now since it's assigned once and never changes!
  private final AprilTagFieldLayout m_tagLayout;

  // --- Simulation (null on the real robot) ---
  private VisionSystemSim m_visionSystemSim;
  private PhotonCameraSim m_backCameraSim;
  private PhotonCameraSim m_sideCameraSim;
  private Supplier<Pose2d> m_groundTruthPose = () -> new Pose2d();

  public VisionSubsystem() {
    // Automatically loads the official 2026 FIM Welded layout
    m_tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  }

  public AprilTagFieldLayout getTagLayout() {
    return m_tagLayout;
  }

  /**
   * Enables camera simulation. Must be called (typically from RobotContainer)
   * with a supplier for the drivetrain's simulated ground-truth pose.
   *
   * @param groundTruthPose supplier of the true robot pose each cycle
   */
  public void setupSimulation(Supplier<Pose2d> groundTruthPose) {
    if (!RobotBase.isSimulation()) {
      return;
    }
    m_groundTruthPose = groundTruthPose;

    m_visionSystemSim = new VisionSystemSim("AprilTag Vision Sim");
    m_visionSystemSim.addAprilTags(m_tagLayout);

    SimCameraProperties backCamProps = createCameraProperties();
    SimCameraProperties sideCamProps = createCameraProperties();

    // Camera names must match the PhotonCamera objects in DriveSubsystem so
    // the sim publishes to the tables they subscribe to.
    m_backCameraSim = new PhotonCameraSim(
        new PhotonCamera(VisionConstants.kBackCameraName), backCamProps);
    m_sideCameraSim = new PhotonCameraSim(
        new PhotonCamera(VisionConstants.kSideCameraName), sideCamProps);

    m_backCameraSim.setMaxSightRange(VisionConstants.kSimCameraMaxSightRangeMeters);
    m_sideCameraSim.setMaxSightRange(VisionConstants.kSimCameraMaxSightRangeMeters);
    // Draws the camera FOV / detected tags on the debug field widget.
    m_backCameraSim.enableDrawWireframe(true);
    m_sideCameraSim.enableDrawWireframe(true);

    m_visionSystemSim.addCamera(m_backCameraSim, VisionConstants.kRobotToBackCamera);
    m_visionSystemSim.addCamera(m_sideCameraSim, VisionConstants.kRobotToSideCamera);

    SmartDashboard.putData("Vision Sim Field", m_visionSystemSim.getDebugField());
  }

  /** Camera intrinsics calibrated to the team's cameras (see VisionConstants). */
  private static SimCameraProperties createCameraProperties() {
    SimCameraProperties props = new SimCameraProperties();
    props.setCalibration(
        VisionConstants.kSimCameraWidthPx,
        VisionConstants.kSimCameraHeightPx,
        Rotation2d.fromDegrees(VisionConstants.kSimCameraDiagonalFovDeg));
    props.setCalibError(
        VisionConstants.kSimCameraCalibErrorPx,
        VisionConstants.kSimCameraCalibErrorStdPx);
    props.setFPS(VisionConstants.kSimCameraFps);
    props.setAvgLatencyMs(VisionConstants.kSimCameraAvgLatencyMs);
    props.setLatencyStdDevMs(VisionConstants.kSimCameraLatencyStdDevMs);
    return props;
  }

  public boolean isSimulationActive() {
    return m_visionSystemSim != null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    if (m_visionSystemSim == null) {
      return;
    }
    // Feed the true pose; the sim produces realistic camera frames from it.
    m_visionSystemSim.update(m_groundTruthPose.get());
  }
}
