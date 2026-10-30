package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder; // WPILib standard deviations

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.AutoTuning;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.ShotMath;
import frc.robot.ShooterCameraAim;
import frc.robot.ShooterTuning;
import frc.robot.driverprofiles.ThrustmasterProfile;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class DriveSubsystem extends SubsystemBase {
  private static final String PATHPLANNER_COMMANDED_OMEGA_KEY =
      "PathPlanner Commanded Omega (rad/s)";
  private final PIDController m_pathXFeedback = new PIDController(5.0, 0.0, 0.0);
  private final PIDController m_pathYFeedback = new PIDController(5.0, 0.0, 0.0);
  private final PIDController m_pathRotationFeedback = new PIDController(5.0, 0.0, 0.0);
  private Pose2d m_pathTargetPose;
  private boolean m_isFollowingPath = false;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor. On the real robot this is the navX MXP. In simulation it
  // is replaced by a sensor model (SimulatedGyro) that integrates the true
  // chassis yaw rate with realistic bias and noise, so heading estimation is
  // exercised exactly like on the real robot.
  private final AHRS m_gyro = RobotBase.isSimulation() ? null : new AHRS(NavXComType.kMXP_SPI);

  // --- Simulation ground truth (real hardware needs no analog) ---
  // NOTE: these are declared before m_poseEstimator, whose initializer reads
  // getGyroRotation() (which consults them in simulation).
  private final boolean m_isSim = RobotBase.isSimulation();
  private Pose2d m_simRobotPose = new Pose2d();
  private double m_simOmegaRadPerSecond = 0.0;
  private final SimulatedGyro m_simGyro =
      new SimulatedGyro(
          SimulationConstants.kSimGyroDriftDegreesPerSecond,
          SimulationConstants.kSimGyroNoiseDegreesPerSecond,
          SimulationConstants.kSimGyroRandomSeed);
  private final Field2d m_simField = new Field2d();

  // vision usage tracking
  private double m_lastVisionFuseTime = -1.0;
  private boolean m_visionActive = false;
  // Initial field position and vision heading converge independently. Vision
  // corrections are bounded while either is being acquired; gyro/alliance
  // heading remains continuous during that process.
  private boolean m_hasVisionPose = false;
  private boolean m_hasVisionHeading = false;
  private DriverStation.Alliance m_startupAlliance = DriverStation.Alliance.Blue;

  // The alliance the robot is competing on (FMS on the real robot, switchable
  // in simulation). Owned by the alliance subsystem so that every consumer -
  // hub location, path mirroring, heading defaults - sees the same value.
  private final AllianceSubsystem m_allianceSubsystem;

  // Driver motion is always field-relative. This source records how the field
  // heading was most recently established or corrected.
  private String m_fieldHeadingSource = "STARTUP ALLIANCE DEFAULT";

  private Rotation2d m_lastVisionRotation = new Rotation2d();

  // --- AUTO AIM VARIABLES ---
  private boolean m_isAutoAiming = false;

  private final Joystick m_Joystick = new Joystick(0);
  private final ThrustmasterProfile m_ThrustmasterProfile = new ThrustmasterProfile(m_Joystick);
  
  // NEW: Dynamically grabs the Hub location based on our current Alliance!
  public Translation2d getHubLocation() {
      return getHubLocation(m_allianceSubsystem.getAlliance());
  }

  /**
   * Returns the hub center for the given alliance.
   *
   * @param alliance the alliance the robot is on
   * @return the center of that alliance's hub
   */
  static Translation2d getHubLocation(DriverStation.Alliance alliance) {
      // If we are on the Red Alliance, return the Red coordinates (From your PathPlanner screenshot)
      if (alliance == DriverStation.Alliance.Red) {
          return new Translation2d(FieldConstants.kRedHubX, FieldConstants.kRedHubY);
      }

      // Default to your original Blue coordinates
      return new Translation2d(FieldConstants.kBlueHubX, FieldConstants.kBlueHubY);
  }

  // Pose estimator (odometry + vision fusion) -----------------------------------------------------------------
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          getGyroRotation(),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          },
          new Pose2d() // start at origin; you can override with resetOdometry
      );

  // === PhotonVision / AprilTag stuff ===

  // Camera and PhotonPoseEstimator ----------------------------
  // Names and transforms are shared with the vision simulation via
  // VisionConstants so the sim publishes frames to these exact cameras.
  // NOTE: the PhotonVision camera named "FrontCamera" is the unit mounted at
  // the REAR of the robot (see VisionConstants). Names always refer to the
  // PhotonVision topic, never the physical mounting.
  private final PhotonCamera m_backCamera = new PhotonCamera(VisionConstants.kBackCameraName);
  private final PhotonCamera m_sideCamera = new PhotonCamera(VisionConstants.kSideCameraName);
  private final PhotonPoseEstimator m_backCameraEstimator;
  private final PhotonPoseEstimator m_sideCameraEstimator;

  // Camera pose estimator ----------------------------------
  // transform for the rear-facing camera
  private final Transform3d m_robotToBackCamera = VisionConstants.kRobotToBackCamera;

  // transform for the side-facing camera
  private final Transform3d m_robotToSideCamera = VisionConstants.kRobotToSideCamera;

  // Frame-intake bookkeeping and measurement policy (de-dup, quality gates,
  // outlier rejection, trust scaling). See VisionFusion.
  private final VisionFusion m_visionFusion = new VisionFusion();

  // Most recent frame from each camera, captured by the once-per-loop intake
  // below. The manual "force reseed" button re-solves from these instead of
  // draining a camera's frame queue a second time in the same loop.
  private final Map<String, PhotonPipelineResult> m_latestFrameByCamera = new HashMap<>();

  // AprilTag layout, shared with the pose estimators. Used by the shooter
  // camera aim correction to compare a detection against the known tag pose.
  private final AprilTagFieldLayout m_tagLayout;

  /** Newest shooter-camera aim correction; recomputed once per loop in periodic(). */
  private ShooterCameraAim.Correction m_shooterCameraAim = ShooterCameraAim.NONE;

  /** Number of vision solutions rejected by the per-component outlier gates. */
  private int m_visionOutliersRejected = 0;

  //------------------------------------------------------------------------------------------------------

  /**
   * Drains both camera queues and fuses frames in capture-time order. This is
   * important because the two cameras have independent, variable latency:
   * applying a newer frame and then an older one can make a latency-compensated
   * pose estimator replay its history backwards.
   *
   * @return true if any camera contributed a measurement this loop
   */
  private boolean addPhotonVisionMeasurement() {
    // Heading data is interpolated to each frame's capture timestamp by the
    // heading-aware PhotonVision solvers, so publish it every robot loop.
    double now = Timer.getFPGATimestamp();
    Rotation2d fieldHeading = getPose().getRotation();
    m_backCameraEstimator.addHeadingData(now, fieldHeading);
    m_sideCameraEstimator.addHeadingData(now, fieldHeading);

    List<PendingVisionFrame> pendingFrames = new ArrayList<>();
    collectCameraFrames(
        VisionConstants.kBackCameraName, m_backCamera, m_backCameraEstimator, pendingFrames);
    collectCameraFrames(
        VisionConstants.kSideCameraName, m_sideCamera, m_sideCameraEstimator, pendingFrames);
    pendingFrames.sort(
        Comparator.comparingDouble(
            (PendingVisionFrame frame) -> frame.result.getTimestampSeconds()));

    boolean backAccepted = false;
    boolean sideAccepted = false;
    for (PendingVisionFrame pending : pendingFrames) {
      boolean accepted = fuseCameraFrame(pending.cameraName, pending.estimator, pending.result);
      if (VisionConstants.kBackCameraName.equals(pending.cameraName)) {
        backAccepted |= accepted;
      } else if (VisionConstants.kSideCameraName.equals(pending.cameraName)) {
        sideAccepted |= accepted;
      }
    }

    SmartDashboard.putBoolean("Vision/FrontCamera Accepted", backAccepted);
    SmartDashboard.putBoolean("Vision/GoodCamera_OldLeft Accepted", sideAccepted);
    return backAccepted || sideAccepted;
  }

  private void collectCameraFrames(
      String cameraName,
      PhotonCamera camera,
      PhotonPoseEstimator cameraEstimator,
      List<PendingVisionFrame> pendingFrames) {
    // getAllUnreadResults() drains each camera FIFO exactly once per loop.
    List<PhotonPipelineResult> frames = camera.getAllUnreadResults();
    if (frames.isEmpty()) {
      return;
    }

    m_latestFrameByCamera.put(cameraName, frames.get(frames.size() - 1));
    for (PhotonPipelineResult frame : frames) {
      pendingFrames.add(new PendingVisionFrame(cameraName, cameraEstimator, frame));
    }
  }

  private static final class PendingVisionFrame {
    final String cameraName;
    final PhotonPoseEstimator estimator;
    final PhotonPipelineResult result;

    PendingVisionFrame(
        String cameraName, PhotonPoseEstimator estimator, PhotonPipelineResult result) {
      this.cameraName = cameraName;
      this.estimator = estimator;
      this.result = result;
    }
  }

  private boolean fuseCameraFrame(
      String cameraName, PhotonPoseEstimator cameraEstimator, PhotonPipelineResult result) {
    if (!result.hasTargets()) return false;

    double frameTimestamp = result.getTimestampSeconds();
    // One capture must never be counted twice, and a late frame from either
    // camera must not move estimator history behind a newer fused measurement.
    if (!m_visionFusion.shouldProcessFrame(cameraName, frameTimestamp)) return false;
    if (!m_visionFusion.isFusionTimestampInOrder(frameTimestamp)) {
      return false;
    }

    var best = result.getBestTarget();

    // Reject highly ambiguous frames and degenerate near-field solves before
    // any pose math runs.
    double dist = best.getBestCameraToTarget().getTranslation().getNorm();
    if (!VisionFusion.isFrameAcceptable(best.getPoseAmbiguity(), dist)) return false;

    Optional<EstimatedRobotPose> estOpt = solveCameraPose(cameraEstimator, result);
    if (estOpt.isEmpty()) return false;

    var est = estOpt.get();
    Pose2d visionMeasurement = est.estimatedPose.toPose2d();
    if (!isFinite(visionMeasurement)
        || !m_visionFusion.isFusionTimestampInOrder(est.timestampSeconds)) {
      return false;
    }

    int targetsUsed = est.targetsUsed == null ? 1 : Math.max(1, est.targetsUsed.size());
    boolean multiTag = est.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    boolean hasMultiTagEvidence = multiTag || targetsUsed >= 2;

    // Compare against the estimator's pose at image-capture time, not the pose
    // now. This avoids mistaking normal robot motion during camera latency for
    // an outlier and keeps the innovation in the correct temporal frame.
    Optional<Pose2d> estimateAtCaptureOptional = m_poseEstimator.sampleAt(est.timestampSeconds);
    if (estimateAtCaptureOptional.isEmpty()) {
      // The capture is outside the estimator's retained odometry history (for
      // example, from before the first update after a reset). Never substitute
      // the present-time pose for a capture-time comparison.
      return false;
    }
    Pose2d estimateAtCapture = estimateAtCaptureOptional.get();
    double translationDelta =
        estimateAtCapture.getTranslation().getDistance(visionMeasurement.getTranslation());
    double estimatedHeadingDegrees = estimateAtCapture.getRotation().getDegrees();
    double measuredHeadingDegrees = visionMeasurement.getRotation().getDegrees();

    // During initial localization the prior position/heading may be wrong, so
    // don't discard it. Once each component has converged, its normal outlier
    // gate protects against bad solves. Every accepted frame is still bounded
    // below, so an acquisition frame cannot snap the displayed pose.
    if ((m_hasVisionPose
            && !VisionFusion.isTranslationConsistentWithEstimate(translationDelta))
        || (m_hasVisionHeading
            && !VisionFusion.isHeadingConsistentWithEstimate(
                estimatedHeadingDegrees, measuredHeadingDegrees))) {
      m_visionOutliersRejected++;
      SmartDashboard.putNumber("Vision/Outliers Rejected", m_visionOutliersRejected);
      return false;
    }

    double[] boundedMeasurement = VisionFusion.limitMeasurementInnovation(
        estimateAtCapture.getX(),
        estimateAtCapture.getY(),
        estimatedHeadingDegrees,
        visionMeasurement.getX(),
        visionMeasurement.getY(),
        measuredHeadingDegrees);
    Pose2d smoothMeasurement = new Pose2d(
        boundedMeasurement[0],
        boundedMeasurement[1],
        Rotation2d.fromDegrees(boundedMeasurement[2]));
    double[] stddevs =
        VisionFusion.measurementStdDevs(targetsUsed, best.getPoseAmbiguity(), dist, multiTag);
    m_poseEstimator.addVisionMeasurement(
        smoothMeasurement,
        est.timestampSeconds,
        VecBuilder.fill(stddevs[0], stddevs[1], stddevs[2]));
    m_visionFusion.recordFusionTimestamp(est.timestampSeconds);

    Pose2d fusedAtCapture =
        m_poseEstimator.sampleAt(est.timestampSeconds).orElse(estimateAtCapture);
    if (!m_hasVisionPose
        && fusedAtCapture.getTranslation().getDistance(visionMeasurement.getTranslation())
            <= VisionFusion.kVisionLocalizationToleranceMeters) {
      m_hasVisionPose = true;
    }
    if (!m_hasVisionHeading
        && hasMultiTagEvidence
        && VisionFusion.isHeadingConsistentWithEstimate(
            fusedAtCapture.getRotation().getDegrees(), measuredHeadingDegrees)) {
      m_hasVisionHeading = true;
    }

    m_fieldHeadingSource = m_hasVisionHeading
        ? "VISION FUSED"
        : (m_hasVisionPose ? "VISION POSITION LOCALIZED; HEADING ACQUIRING" : "VISION LOCALIZING");
    m_lastVisionRotation = visionMeasurement.getRotation();
    m_lastVisionFuseTime = Timer.getFPGATimestamp();
    SmartDashboard.putString("Vision/Last Accepted Camera", cameraName);
    return true;
  }

  /**
   * Solves the robot pose for one captured frame. Multi-tag PNP (when the
   * camera provides it) gives the strongest heading solution; otherwise the
   * tag closest to the current pose estimate is used.
   */
  private Optional<EstimatedRobotPose> solveCameraPose(
      PhotonPoseEstimator cameraEstimator, PhotonPipelineResult result) {
    Optional<EstimatedRobotPose> estOpt = cameraEstimator.estimateCoprocMultiTagPose(result);
    if (estOpt.isEmpty()) {
      estOpt = cameraEstimator.estimateClosestToReferencePose(result, new Pose3d(getPose()));
    }
    return estOpt;
  }

  private static boolean isFinite(Pose2d pose) {
    return Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }

  /**
   * Returns the gyroscope's robot heading in the WPILib convention
   * (counter-clockwise positive). This is the primary heading sensor fed to the
   * pose estimator every loop; the estimator fuses its angle deltas with wheel
   * odometry and corrects its drift with vision.
   *
   * @return the gyro heading
   */
  public Rotation2d getGyroRotation() {
      if (m_isSim) {
        // The simulated navX is a real sensor model (bias + noise) integrated
        // from the true chassis yaw rate - not the ground-truth angle - so the
        // estimator has to handle gyro drift here just like on the real robot.
        return Rotation2d.fromDegrees(m_simGyro.getAngleDegrees());
      }
      // navX yaw is clockwise-positive; WPILib headings are counter-clockwise
      // positive. kGyroReversed flips the sense for inverted board mounting.
      return Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? 1.0 : -1.0));
  }

  /** Constructor - Creates a new DriveSubsystem. */
  public DriveSubsystem(VisionSubsystem visionSystem, AllianceSubsystem allianceSubsystem) {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    SmartDashboard.putNumber(PATHPLANNER_COMMANDED_OMEGA_KEY, 0.0);

    m_allianceSubsystem = allianceSubsystem;
    m_startupAlliance = m_allianceSubsystem.getAlliance();
    // Move to the other side of the field when the alliance changes in
    // simulation, exactly like PathPlanner flips a blue-authored path.
    m_allianceSubsystem.addAllianceChangedListener(this::onAllianceChanged);

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportError(
          "PathPlanner configuration failed. Check deploy/pathplanner/settings.json: " + e,
          e.getStackTrace());
    }

    // PhotonVision 2026: the strategy-taking constructor and update() are
    // deprecated for removal; the estimation method called per frame selects
    // the strategy (see solveCameraPose).
    m_tagLayout = visionSystem.getTagLayout();
    m_backCameraEstimator = new PhotonPoseEstimator(
      m_tagLayout,
      m_robotToBackCamera
    );
    m_sideCameraEstimator = new PhotonPoseEstimator(
      m_tagLayout,
      m_robotToSideCamera
    );

    // Start with one consistent heading truth. In simulation the camera sees
    // m_simRobotPose, so it must use the same alliance-default heading as the
    // estimator or the first camera frame would appear to be a 180-degree
    // correction. On real hardware the alliance heading is the startup prior.
    Pose2d startupPose = withAllianceDefaultHeading(
        m_isSim ? m_simRobotPose : getPose(), m_startupAlliance);
    if (m_isSim) {
      m_simRobotPose = startupPose;
    }
    resetOdometryFromKnownFieldPose(startupPose, "STARTUP ALLIANCE DEFAULT");

    // A frame can already be waiting on a real camera, but the default pose is
    // established first so any first-vision localization preserves its heading.
    addPhotonVisionMeasurement();
    
    // Configure AutoBuilder last
    m_pathRotationFeedback.enableContinuousInput(-Math.PI, Math.PI);
    PPHolonomicDriveController.clearFeedbackOverrides();
    if (config != null) {
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (ChassisSpeeds speeds, DriveFeedforwards feedforwards) ->
              driveRobotRelative(speeds, feedforwards),
          new PPHolonomicDriveController(
              new PIDConstants(
                  AutoTuning.translationP(), AutoTuning.translationI(), AutoTuning.translationD()),
              new PIDConstants(
                  AutoTuning.rotationP(), AutoTuning.rotationI(), AutoTuning.rotationD())) {
            @Override
            public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
              super.reset(currentPose, currentSpeeds);
              m_pathXFeedback.reset();
              m_pathYFeedback.reset();
              m_pathRotationFeedback.reset();
            }

            @Override
            public ChassisSpeeds calculateRobotRelativeSpeeds(
                Pose2d currentPose, PathPlannerTrajectoryState targetState) {
              DriveSubsystem.this.updatePathFeedbackGains();
              double xFF = targetState.fieldSpeeds.vxMetersPerSecond;
              double yFF = targetState.fieldSpeeds.vyMetersPerSecond;
              double rotFF = targetState.fieldSpeeds.omegaRadiansPerSecond;
              Rotation2d targetRotation =
                  DriveSubsystem.this
                      .getRotationTargetOverride()
                      .orElse(targetState.pose.getRotation());
              double xFeedback =
                  m_pathXFeedback.calculate(currentPose.getX(), targetState.pose.getX());
              double yFeedback =
                  m_pathYFeedback.calculate(currentPose.getY(), targetState.pose.getY());
              double rotFeedback =
                  m_pathRotationFeedback.calculate(
                      currentPose.getRotation().getRadians(), targetRotation.getRadians());
              return ChassisSpeeds.fromFieldRelativeSpeeds(
                  xFF + xFeedback,
                  yFF + yFeedback,
                  rotFF + rotFeedback,
                  currentPose.getRotation());
            }
          },
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            return m_allianceSubsystem.isRed();
          },
          this // Reference to this subsystem to set requirements
      );
    }

    PathPlannerLogging.setLogTargetPoseCallback(pose -> m_pathTargetPose = pose);
    PathPlannerLogging.setLogActivePathCallback(poses -> {
      m_isFollowingPath = !poses.isEmpty();
      if (!m_isFollowingPath) m_pathTargetPose = null;
    });
    PathPlannerLogging.setLogCurrentPoseCallback(pose -> {
      SmartDashboard.putNumber("Auto/Current Pose X (m)", pose.getX());
      SmartDashboard.putNumber("Auto/Current Pose Y (m)", pose.getY());
      SmartDashboard.putNumber("Auto/Current Heading (deg)", pose.getRotation().getDegrees());
    });

    // Publish the simulated field so the robot pose can be watched in the sim
    // GUI / dashboards (Glass, AdvantageScope).
    if (m_isSim) {
      SmartDashboard.putData("Field", m_simField);
    }
  }

  private void updatePathFeedbackGains() {
    double kp = AutoTuning.translationP();
    double ki = AutoTuning.translationI();
    double kd = AutoTuning.translationD();
    m_pathXFeedback.setPID(kp, ki, kd);
    m_pathYFeedback.setPID(kp, ki, kd);
    m_pathRotationFeedback.setPID(
        AutoTuning.rotationP(), AutoTuning.rotationI(), AutoTuning.rotationD());
  }

  private static final double kHeadingToleranceDeg = 2.0;
  private static final double kMaxHeadingOutput = 0.45;

  private final PIDController m_headingPID =
    new PIDController(0.01, 0.0, 0.0001);   // tune later
  {
    m_headingPID.enableContinuousInput(-180, 180);
    m_headingPID.setTolerance(kHeadingToleranceDeg);
  }


  @Override
  public void periodic() {
    // Driver Station alliance data may arrive after robotInit. Until a trusted
    // vision heading has been established, update the startup heading prior.
    DriverStation.Alliance alliance = m_allianceSubsystem.getAlliance();
    if (!m_hasVisionHeading && alliance != m_startupAlliance) {
      m_startupAlliance = alliance;
      Pose2d startupPose = new Pose2d(
          getPose().getTranslation(), getAllianceDefaultHeading(alliance));
      if (m_isSim) {
        m_simRobotPose = withAllianceDefaultHeading(m_simRobotPose, alliance);
        if (!m_hasVisionPose) {
          startupPose = m_simRobotPose;
        }
      }
      resetOdometryFromKnownFieldPose(startupPose, "STARTUP ALLIANCE DEFAULT");
    }

    // 1. Update pose estimator with gyro + modules
    m_poseEstimator.update(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  
    // 2. Inject any new PhotonVision measurements
    addPhotonVisionMeasurement();

    // 2b. Turn the newest shooter-camera frame into an aim residual. Cheap
    // (a handful of tags), so it runs every loop even when the shooter is idle;
    // that keeps the dashboard diagnostic live while tuning.
    updateShooterCameraAim();
  
    // 3. Update pose indicator for debugging
    double now = Timer.getFPGATimestamp();
    m_visionActive = (m_lastVisionFuseTime >= 0.0)
                   && ((now - m_lastVisionFuseTime) < 0.25);  // 250 ms window


    // Pose data
    SmartDashboard.putNumber("Robot X (m)", getPose().getX());
    SmartDashboard.putNumber("Robot Y (m)", getPose().getY());
    SmartDashboard.putNumber("Robot Heading (deg)", getPose().getRotation().getDegrees());

    ChassisSpeeds measuredRobotSpeeds = getRobotRelativeSpeeds();
    SmartDashboard.putNumber("Auto/Measured Vx (mps)", measuredRobotSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Auto/Measured Vy (mps)", measuredRobotSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Auto/Measured Omega (radps)", measuredRobotSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putBoolean("Auto/Following Path", m_isFollowingPath);
    if (m_isFollowingPath && m_pathTargetPose != null) {
      SmartDashboard.putNumber("Auto/Target Pose X (m)", m_pathTargetPose.getX());
      SmartDashboard.putNumber("Auto/Target Pose Y (m)", m_pathTargetPose.getY());
      SmartDashboard.putNumber("Auto/Target Heading (deg)", m_pathTargetPose.getRotation().getDegrees());
      SmartDashboard.putNumber("Auto/Translation Error X (m)", m_pathTargetPose.getX() - getPose().getX());
      SmartDashboard.putNumber("Auto/Translation Error Y (m)", m_pathTargetPose.getY() - getPose().getY());
      SmartDashboard.putNumber("Auto/Heading Error (deg)", MathUtil.inputModulus(
          m_pathTargetPose.getRotation().getDegrees() - getPose().getRotation().getDegrees(),
          -180.0, 180.0));
    }

    // Gyro data
    if (!m_isSim) {
      SmartDashboard.putNumber("Raw Gyro Y", m_gyro.getRawGyroY());
      SmartDashboard.putNumber("NavX Angle", m_gyro.getAngle());
      SmartDashboard.putNumber("NavX Rotation", m_gyro.getRotation2d().getDegrees());
      SmartDashboard.putNumber("NavX Raw Yaw", m_gyro.getYaw());
    }

    // Vision status
    SmartDashboard.putBoolean("Vision Active", m_visionActive);
    SmartDashboard.putBoolean("Field Heading Initialized", true);
    SmartDashboard.putString("Field Heading Source", m_fieldHeadingSource);

    // Distance to hub center
    // UPDATED: Now uses getHubLocation()
    double dx = getHubLocation().getX() - getPose().getX();
    double dy = getHubLocation().getY() - getPose().getY();
    double distanceToHub = Math.hypot(dx, dy);

    SmartDashboard.putNumber("Distance to Hub (m)", distanceToHub);
    SmartDashboard.putNumber("Gyro Rotation Used", getGyroRotation().getDegrees());
    
    SmartDashboard.putNumber("Odometry: ", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("Vision Heading", m_lastVisionRotation.getDegrees());

      SmartDashboard.putNumber("velocity",m_ThrustmasterProfile.getDriveSpeedMultiplier());
}
  

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * <p>This is the "the robot is now at this pose" signal. PathPlanner calls it
   * at autonomous start with the authored start pose (translation AND rotation,
   * already mirrored for the alliance); on the real robot the driver physically
   * places the robot there first. The full pose is authoritative: WPILib's
   * odometry offsets the gyro internally, so the reset heading never needs to
   * match the raw gyro reading, and discarding the rotation would leave the
   * robot disagreeing with the path it is about to follow.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    if (m_isSim) {
      // On the real robot the driver physically places the robot at the start
      // of the auto; in simulation we move the ground-truth pose instead -
      // translation AND rotation - so the physics, the vision simulation and
      // the estimator all agree with where the robot actually is.
      m_simRobotPose = pose;
    }
    resetOdometryFromKnownFieldPose(pose, "ODOMETRY RESET");
    // PathPlanner's authored start pose is authoritative, so camera data must
    // fuse as a correction rather than restart localization after a reset.
    m_hasVisionPose = true;
    m_hasVisionHeading = true;
  }

  private void resetOdometryFromKnownFieldPose(Pose2d pose, String source) {
    m_poseEstimator.resetPosition(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
    // The heading-aware vision solvers interpolate the field heading to each
    // frame's capture time; a pose reset discontinues that series.
    double now = Timer.getFPGATimestamp();
    m_backCameraEstimator.resetHeadingData(now, pose.getRotation());
    m_sideCameraEstimator.resetHeadingData(now, pose.getRotation());
    // A reset starts a new estimator history. Do not replay camera frames that
    // were captured before that history existed, including a manual correction
    // from the cached latest-frame path.
    m_visionFusion.resetFusionTimestamp(now);
    m_latestFrameByCamera.clear();
    m_fieldHeadingSource = source;
  }

  static Rotation2d getAllianceDefaultHeading(DriverStation.Alliance alliance) {
    return Rotation2d.fromDegrees(
        alliance == DriverStation.Alliance.Red ? 0.0 : 180.0);
  }

  static Pose2d withAllianceDefaultHeading(
      Pose2d pose, DriverStation.Alliance alliance) {
    return new Pose2d(pose.getTranslation(), getAllianceDefaultHeading(alliance));
  }

  private Rotation2d getAllianceDefaultHeading() {
    m_startupAlliance = m_allianceSubsystem.getAlliance();
    return getAllianceDefaultHeading(m_startupAlliance);
  }

  /**
   * Mirrors a field pose to the other side of the field.
   *
   * <p>The 2026 field is rotationally symmetric, so the red half is the blue
   * half rotated 180 degrees about the field center. This is the same
   * transform PathPlanner applies when it flips a blue-authored path for the
   * red alliance, which keeps a mirrored robot and a flipped path in the same
   * place.
   *
   * @param pose the pose to mirror
   * @return the mirrored pose
   */
  static Pose2d mirrorPoseAcrossField(Pose2d pose) {
    return new Pose2d(
        new Translation2d(
            FieldConstants.kFieldLengthMeters - pose.getX(),
            FieldConstants.kFieldWidthMeters - pose.getY()),
        pose.getRotation().minus(Rotation2d.kPi));
  }

  /**
   * Called when the alliance changes (simulation only). The robot is moved to
   * the matching spot on the other side of the field and the heading is
   * re-seeded from the new alliance, so an auto authored on the blue side can
   * be tested on the red side without re-authoring anything.
   *
   * @param alliance the new alliance
   */
  private void onAllianceChanged(DriverStation.Alliance alliance) {
    m_startupAlliance = alliance;

    if (m_isSim && m_allianceSubsystem.shouldMirrorRobot()) {
      // Move the ground truth to the mirrored spot ...
      Pose2d mirrored = mirrorPoseAcrossField(m_simRobotPose);
      // ... and point the robot at the alliance wall, like the startup default.
      m_simRobotPose = new Pose2d(
          mirrored.getTranslation(), getAllianceDefaultHeading(alliance));
      resetOdometryFromKnownFieldPose(
          m_simRobotPose,
          "ALLIANCE CHANGED: " + alliance.name());
      // This is a known pose reset. New camera frames correct it gradually;
      // they must not trigger another initialization snap.
      m_hasVisionPose = true;
      m_hasVisionHeading = true;
    } else if (m_isSim) {
      // With mirroring disabled, keep the simulated robot exactly where it is
      // and keep the estimate aligned to that unchanged ground-truth pose.
      if (!m_hasVisionPose) {
        resetOdometryFromKnownFieldPose(
            m_simRobotPose,
            "ALLIANCE CHANGED WITHOUT MIRROR: " + alliance.name());
        m_hasVisionPose = true;
        m_hasVisionHeading = true;
      }
    } else if (!m_hasVisionHeading) {
      resetOdometryFromKnownFieldPose(
          new Pose2d(getPose().getTranslation(), getAllianceDefaultHeading(alliance)),
          "STARTUP ALLIANCE DEFAULT");
    }

    System.out.println("Alliance changed to " + alliance.name() + ", pose: " + getPose());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    double rotation = applyHeadingController(rot);

    // Convert to drivetrain units
    final double xSpeedDelivered  = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    final double ySpeedDelivered  = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    final double rotDelivered     = rotation * DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds speeds = calculateFieldRelativeSpeeds(
        xSpeedDelivered,
        ySpeedDelivered,
        rotDelivered,
        getPose().getRotation());

    setChassisSpeeds(speeds);
  }

  /**
   * Drives using normalized commands expressed in the robot frame.
   *
   * <p>This applies the same speed scaling and heading-controller policy as
   * {@link #drive(double, double, double)}, but intentionally does not rotate
   * translation using the estimated field heading. Positive X is robot-front.
   *
   * @param xSpeed normalized robot-forward speed
   * @param ySpeed normalized robot-left speed
   * @param rot normalized counter-clockwise angular rate
   */
  public void driveRobotRelativeNormalized(double xSpeed, double ySpeed, double rot) {
    ChassisSpeeds speeds = calculateRobotRelativeNormalizedSpeeds(
        xSpeed, ySpeed, applyHeadingController(rot));
    setChassisSpeeds(speeds);
  }

  private double applyHeadingController(double rot) {
    // Decide rotation to use (heading-hold vs driver)
    double rotation;
  if (m_useHeadingController && Math.abs(rot) < 0.05) {
    // Heading targets are field-relative, so compare them against the
    // estimator's field-aligned heading rather than the navX startup frame.
    double currentYaw = getPose().getRotation().getDegrees();

    rotation = m_headingPID.calculate(currentYaw);

    rotation = MathUtil.clamp(
      rotation,
      -kMaxHeadingOutput,
      kMaxHeadingOutput
    );

  if (m_headingPID.atSetpoint()) {
      rotation = 0.0;
      m_useHeadingController = false;
  }
    } else {
      rotation = rot;
      if (Math.abs(rot) > 0.05) {
        m_useHeadingController = false; // driver is actively rotating
      }
    }
    return rotation;
  }

  static ChassisSpeeds calculateRobotRelativeNormalizedSpeeds(
      double xSpeed, double ySpeed, double rot) {
    return new ChassisSpeeds(
        xSpeed * DriveConstants.kMaxSpeedMetersPerSecond,
        ySpeed * DriveConstants.kMaxSpeedMetersPerSecond,
        rot * DriveConstants.kMaxAngularSpeed);
  }

  private void setChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
  
    // Apply to modules
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Converts driver commands from the field frame into the robot frame.
   * There is deliberately no robot-relative fallback.
   */
  static ChassisSpeeds calculateFieldRelativeSpeeds(
      double xSpeed,
      double ySpeed,
      double rot,
      Rotation2d estimatorRotation) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, rot, estimatorRotation);
  }
  
  /** Drive the robot using robot-relative chassis speeds (convenience overload). */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    driveRobotRelative(speeds, DriveFeedforwards.zeros(4));
  }

  /**
   * Drive the robot using robot-relative chassis speeds and per-module force feedforwards from
   * PathPlanner's {@link AutoBuilder}.
   */
  public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    // This consumer receives PathPlanner's controller output before wheel-speed
    // desaturation, so it is the requested angular velocity rather than a
    // measured or subsequently limited value.
    SmartDashboard.putNumber(
        PATHPLANNER_COMMANDED_OMEGA_KEY, speeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Auto/Commanded Vx (mps)", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Auto/Commanded Vy (mps)", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Auto/Commanded Omega (radps)", speeds.omegaRadiansPerSecond);

    // PathPlanner supplies robot-relative speeds. Discretize over the robot
    // loop before inverse kinematics so simultaneous translation and rotation
    // do not introduce a repeatable lateral/longitudinal error each 20 ms.
    ChassisSpeeds discretizedSpeeds =
        ChassisSpeeds.discretize(speeds, SimulationConstants.kLoopPeriodSeconds);

    // Convert chassis speeds to module states
    SwerveModuleState[] states =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(discretizedSpeeds);

    // Desaturate to respect max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, AutoTuning.maxModuleSpeed());

    double[] fx =
        feedforwards != null ? feedforwards.robotRelativeForcesXNewtons() : new double[4];
    double[] fy =
        feedforwards != null ? feedforwards.robotRelativeForcesYNewtons() : new double[4];

    // Apply states and force feedforwards to modules (FL, FR, BL, BR order)
    m_frontLeft.setDesiredState(
        states[0], fx.length > 0 ? fx[0] : 0.0, fy.length > 0 ? fy[0] : 0.0);
    m_frontRight.setDesiredState(
        states[1], fx.length > 1 ? fx[1] : 0.0, fy.length > 1 ? fy[1] : 0.0);
    m_rearLeft.setDesiredState(
        states[2], fx.length > 2 ? fx[2] : 0.0, fy.length > 2 ? fy[2] : 0.0);
    m_rearRight.setDesiredState(
        states[3], fx.length > 3 ? fx[3] : 0.0, fy.length > 3 ? fy[3] : 0.0);
  }

  // Target heading the PID should drive toward
  private double m_headingTarget = 0.0;

  // Whether the drive command should use heading-hold
  private boolean m_useHeadingController = false;

  /** Set a desired robot heading in degrees (field-relative). */
  public void setHeading(double degrees) {
    m_headingTarget = MathUtil.inputModulus(degrees, -180.0, 180.0);
    m_headingPID.reset();
    m_headingPID.setSetpoint(m_headingTarget);
    m_useHeadingController = true;
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Returns the turn rate of the robot in degrees per second
   * (counter-clockwise positive), matching {@link #getGyroRotation()}.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    if (m_isSim) {
      return m_simGyro.getRateDegreesPerSecond();
    }
    // Same sign convention as getGyroRotation(): navX rates are clockwise-positive.
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? 1.0 : -1.0);
  }

  /** * Returns the current robot-relative chassis speeds
   * based on the measured module states.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    );
  }

  /** Corrects the field pose from a high-confidence vision result. */
  public boolean correctTranslationFromVision() {
      // Snapshot both cached frames because the first correction resets the
      // estimator and clears the cache. Do not short-circuit: either camera can
      // provide the manual correction.
      PhotonPipelineResult backFrame = m_latestFrameByCamera.get(VisionConstants.kBackCameraName);
      PhotonPipelineResult sideFrame = m_latestFrameByCamera.get(VisionConstants.kSideCameraName);
      boolean backCorrected = correctTranslationFromCamera(
          VisionConstants.kBackCameraName, m_backCameraEstimator, backFrame);
      boolean sideCorrected = correctTranslationFromCamera(
          VisionConstants.kSideCameraName, m_sideCameraEstimator, sideFrame);
      return backCorrected | sideCorrected;
  }

  private boolean correctTranslationFromCamera(
      String cameraName,
      PhotonPoseEstimator cameraEstimator,
      PhotonPipelineResult result) {
      // Re-solve from the most recent captured frame (taken in by the single
      // once-per-loop intake). Draining the camera queue here would steal the
      // frames the fusion path is supposed to see.
      if (result == null || !result.hasTargets()) return false;

      var best = result.getBestTarget();

      // Strict Ambiguity Filter: Only seed if the calculation is highly precise
      if (best.getPoseAmbiguity() > 0.15 && best.getPoseAmbiguity() >= 0.0) return false;

      // Distance Guard: Don't seed from completely across the stadium floor
      double dist = best.getBestCameraToTarget().getTranslation().getNorm();
      if (!Double.isFinite(dist) || dist > 4.0
          || dist < VisionFusion.kMinCameraToTagDistanceMeters) return false;

      var update = solveCameraPose(cameraEstimator, result);
      if (update.isPresent()) {
          Pose2d visionPose = update.get().estimatedPose.toPose2d();
          if (!isFinite(visionPose)) return false;

          resetOdometryFromKnownFieldPose(
              visionPose, "MANUAL VISION CORRECTION: " + cameraName);
          m_hasVisionPose = true;
          m_hasVisionHeading = true;
          m_lastVisionRotation = visionPose.getRotation();
          m_lastVisionFuseTime = Timer.getFPGATimestamp();
          SmartDashboard.putString("Vision/Last Accepted Camera", cameraName);

          System.out.println(
              "STATUS: Vision pose correction applied from " + cameraName + ": " + visionPose);
          return true;
      }

      return false;
  }

  /** Turns Auto-Aim on or off for both TeleOp and PathPlanner */
  public void setAutoAiming(boolean isAiming) {
      m_isAutoAiming = isAiming;
  }

  // =========================================================================
  // --- ODOMETRY AIMING FOR TELEOP & PATHPLANNER ---
  // =========================================================================

  /**
   * Full aim solution for the current pose: the heading that points the
   * shooter at the hub, plus the distance from the muzzle (not the robot
   * center) to the hub.
   *
   * <p>Uses {@link ShotMath#solveAim} so the shooter's mounting offset and the
   * mechanical heading trim are applied consistently everywhere. With the
   * offsets left at zero this reduces exactly to the old
   * {@code atan2(dy, dx) - 180} formula.
   */
  /**
   * Newest shooter-camera aim correction.
   *
   * <p>This is a <em>residual</em>: the difference between where the camera
   * actually sees an AprilTag and where the fused pose says it should be. It is
   * not a second pose; it is the number the shooter should add to its own aim
   * error and distance. {@link ShooterCameraAim} documents the sign convention.
   *
   * @return the last computed correction; {@link ShooterCameraAim#NONE} when the
   *     camera has not produced a frame with a target
   */
  public ShooterCameraAim.Correction getShooterCameraAim() {
    return m_shooterCameraAim;
  }

  /** Recomputes and publishes the shooter-camera aim residual. */
  private void updateShooterCameraAim() {
    m_shooterCameraAim = computeShooterCameraAim();

    ShooterCameraAim.Correction correction = m_shooterCameraAim;
    SmartDashboard.putBoolean("Vision/ShooterCam Aim Valid", correction.valid());
    SmartDashboard.putString("Vision/ShooterCam Status", correction.reason());
    SmartDashboard.putNumber("Vision/ShooterCam Aim Error (deg)", correction.aimErrorDeg());
    SmartDashboard.putNumber("Vision/ShooterCam Range Error (m)", correction.rangeErrorMeters());
    SmartDashboard.putNumber("Vision/ShooterCam Frame Age (s)", correction.ageSeconds());
    SmartDashboard.putNumber("Vision/ShooterCam Tags Used", correction.acceptedTargets());
  }

  /**
   * Builds {@link ShooterCameraAim.Target}s from the newest frame and evaluates
   * them against the fused pose. The frame is never drained here: it was cached
   * by the once-per-loop camera intake, so the fusion path still sees it.
   */
  private ShooterCameraAim.Correction computeShooterCameraAim() {
    PhotonPipelineResult result = m_latestFrameByCamera.get(VisionConstants.kShooterCameraName);
    if (result == null || !result.hasTargets() || m_tagLayout == null) {
      return ShooterCameraAim.NONE;
    }

    List<ShooterCameraAim.Target> targets = new ArrayList<>();
    for (PhotonTrackedTarget target : result.getTargets()) {
      Optional<Pose3d> tagPose = m_tagLayout.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) {
        continue;
      }
      Transform3d cameraToTag = target.getBestCameraToTarget();
      targets.add(new ShooterCameraAim.Target(
          target.getFiducialId(),
          cameraToTag.getX(),
          cameraToTag.getY(),
          target.getPoseAmbiguity(),
          tagPose.get().getX(),
          tagPose.get().getY()));
    }

    Transform3d mount = VisionConstants.kRobotToShooterCamera;
    ShooterCameraAim.Mount cameraMount = new ShooterCameraAim.Mount(
        mount.getX(),
        mount.getY(),
        mount.getRotation().toRotation2d().getDegrees());

    ShooterCameraAim.Config config = new ShooterCameraAim.Config(
        ShooterTuning.cameraAimMaxAgeSeconds(),
        ShooterTuning.cameraAimMaxAmbiguity(),
        VisionConstants.kCameraAimMinRangeMeters,
        VisionConstants.kCameraAimMaxRangeMeters,
        VisionConstants.kCameraAimMaxCorrectionDeg,
        VisionConstants.kCameraAimMaxRangeCorrectionMeters);

    Pose2d pose = getPose();
    double age = Timer.getFPGATimestamp() - result.getTimestampSeconds();
    return ShooterCameraAim.evaluate(
        targets,
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees(),
        cameraMount,
        age,
        config);
  }

  public ShotMath.AimSolution getAimSolution() {
      ShotMath.AimSolution base =
          ShotMath.solveAim(
              getPose(),
              getHubLocation(),
              ShooterConstants.kMuzzleOffsetMeters,
              ShooterConstants.kShooterFacingOffsetDegrees,
              ShooterTuning.headingTrimDegrees(),
              ShotMath.kDefaultIterations);

      // The shooter camera sees the aim error the pose cannot: fold its residual
      // into the one number everything reads (the drive's aim command, the shot
      // gate and the muzzle distance), so the robot turns until the tag is
      // actually centred instead of until the pose thinks it is.
      if (!ShooterTuning.useShooterCameraAim()) {
          return base;
      }
      ShooterCameraAim.Correction correction = m_shooterCameraAim;
      if (!correction.valid()) {
          return base;
      }
      return new ShotMath.AimSolution(
          MathUtil.inputModulus(base.headingDegrees() + correction.aimErrorDeg(), -180.0, 180.0),
          Math.max(0.0, base.distanceMeters() + correction.rangeErrorMeters()));
  }

  /** Calculates the exact field-relative angle the robot needs to face to shoot at the Hub */
  public double getAngleToHub() {
      return getAimSolution().headingDegrees();
  }

  /** Distance from the shooter muzzle to the hub center, in meters. */
  public double getShotDistanceMeters() {
      return getAimSolution().distanceMeters();
  }

  /**
   * PathPlanner asks this method every loop: "Should I override the path's rotation?"
   * If we return Optional.empty(), it follows the path. 
   * If we return a Rotation2d, it snaps to that angle!
   */
  public Optional<Rotation2d> getRotationTargetOverride() {
      if (m_isAutoAiming) {
          // We unified this! Now it just calls the exact same math the Left Trigger uses.
          return Optional.of(Rotation2d.fromDegrees(getAngleToHub()));
      }
      // Not auto-aiming? Just follow the PathPlanner trajectory normally.
      return Optional.empty();
  }

  /**
   * Returns whether the camera has recently seen and fused an AprilTag.
   * Used by RobotContainer to gate the Auto-Aim trigger.
   */
  public boolean isVisionActive() {
      return m_visionActive;
  }

  public void stopAll() {
    m_useHeadingController = false;
    m_headingPID.reset();

    drive(0.0, 0.0, 0.0);
  }

  // =========================================================================
  // --- SIMULATION: chassis-level physics integration ---
  // =========================================================================

  /**
   * Steps all four module physics models and integrates the resulting measured
   * chassis speed into the ground-truth robot pose. The pose estimator then
   * consumes the same module states (through the simulated encoders) and the
   * simulated gyro (through {@link #getGyroRotation()}), so estimation and
   * ground truth track each other just like on the real robot - modulo the
   * gyro's simulated drift and the vision corrections that reject it.
   */
  @Override
  public void simulationPeriodic() {
    double dt = SimulationConstants.kLoopPeriodSeconds;

    // 1. Step the motor physics of every module (closed motor loops are
    //    emulated by REVLib; see MAXSwerveModule).
    m_frontLeft.simulationPeriodic(dt);
    m_frontRight.simulationPeriodic(dt);
    m_rearLeft.simulationPeriodic(dt);
    m_rearRight.simulationPeriodic(dt);

    // 2. Compute measured chassis speeds from the simulated encoders. Reading
    //    the encoders (rather than the physics models directly) means anything
    //    that would break odometry on the real robot shows up here too.
    ChassisSpeeds measuredSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
    m_simOmegaRadPerSecond = measuredSpeeds.omegaRadiansPerSecond;

    // 2b. Step the simulated gyro: it integrates the true yaw rate with its
    //     bias and noise, so it drifts like the real navX and the estimator's
    //     vision fusion gets to prove it can reject that drift.
    m_simGyro.update(Math.toDegrees(measuredSpeeds.omegaRadiansPerSecond), dt);

    // 3. Integrate ground-truth pose (exponential update, same math as the
    //    pose estimator's update step).
    Twist2d twist = new Twist2d(
        measuredSpeeds.vxMetersPerSecond * dt,
        measuredSpeeds.vyMetersPerSecond * dt,
        measuredSpeeds.omegaRadiansPerSecond * dt);
    m_simRobotPose = m_simRobotPose.exp(twist);

    // 4. Visualization: ground truth (robot) vs. pose estimate.
    m_simField.setRobotPose(m_simRobotPose);
    m_simField.getObject("Pose Estimator").setPose(getPose());

    SwerveModuleState[] desiredStates = {
        m_frontLeft.getDesiredState(), m_frontRight.getDesiredState(),
        m_rearLeft.getDesiredState(), m_rearRight.getDesiredState()};
    SmartDashboard.putNumber("Sim/FL Desired Speed (mps)", desiredStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Sim/Ground Truth X (m)", m_simRobotPose.getX());
    SmartDashboard.putNumber("Sim/Ground Truth Y (m)", m_simRobotPose.getY());
    SmartDashboard.putNumber("Sim/Ground Truth Heading (deg)", m_simRobotPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Sim/Pose Estimate Error X (m)", getPose().getX() - m_simRobotPose.getX());
    SmartDashboard.putNumber("Sim/Pose Estimate Error Y (m)", getPose().getY() - m_simRobotPose.getY());
    SmartDashboard.putNumber("Sim/Pose Estimate Error Heading (deg)", MathUtil.inputModulus(
        getPose().getRotation().getDegrees() - m_simRobotPose.getRotation().getDegrees(),
        -180.0, 180.0));
  }

  /**
   * Returns the simulated robot's ground-truth field pose. Used by the vision
   * simulation to compute what the cameras would see, and by dashboards.
   */
  public Pose2d getSimPose() {
    return m_simRobotPose;
  }

  /** Total simulated stator current draw of the drivetrain, in amps. */
  public double getSimulatedCurrentDraw() {
    if (!m_isSim) {
      return 0.0;
    }
    return m_frontLeft.getSimulatedCurrentDraw()
        + m_frontRight.getSimulatedCurrentDraw()
        + m_rearLeft.getSimulatedCurrentDraw()
        + m_rearRight.getSimulatedCurrentDraw();
  }

  // =========================================================================
  // --- SYSTEM IDENTIFICATION (WPILib SysId) ---
  // =========================================================================

  private static final double kSysIdAlignToleranceRad = Math.toRadians(5.0);
  private static final double kSysIdAlignTimeoutSeconds = 0.5;
  private static final Rotation2d[] kAngularSysIdModuleAngles = computeAngularSysIdModuleAngles();

  // Mutable unit holders reused every loop to avoid heap allocations during logging.
  private final MutVoltage m_sysIdVoltage = Volts.mutable(0.0);
  private final MutDistance m_sysIdDistance = Meters.mutable(0.0);
  private final MutLinearVelocity m_sysIdLinearVelocity = MetersPerSecond.mutable(0.0);
  private final MutAngle m_sysIdAngle = Radians.mutable(0.0);
  private final MutAngularVelocity m_sysIdAngularVelocity = RadiansPerSecond.mutable(0.0);

  /**
   * Linear (translation) SysId routine: holds all four module steering angles straight forward
   * ({@code 0 deg}, no 180-degree optimization flips) and drives all four NEO drive motors with the
   * requested open-loop voltage.
   */
  private final SysIdRoutine m_linearSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1.0).per(Second),
              Volts.of(7.0),
              Seconds.of(10.0)),
          new SysIdRoutine.Mechanism(
              voltage -> setSysIdLinearVolts(voltage.in(Volts)),
              log -> {
                logDriveModuleLinear(log, "drive-front-left", m_frontLeft);
                logDriveModuleLinear(log, "drive-front-right", m_frontRight);
                logDriveModuleLinear(log, "drive-rear-left", m_rearLeft);
                logDriveModuleLinear(log, "drive-rear-right", m_rearRight);
              },
              this,
              "drive-linear"));

  /**
   * Angular (chassis rotation / MOI) SysId routine: holds each module tangent to the chassis circle
   * for pure counter-clockwise rotation ({@code +Volts} -> {@code +CCW}) and logs mean drive
   * voltage against continuous gyro heading and yaw rate.
   */
  private final SysIdRoutine m_angularSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1.0).per(Second),
              Volts.of(6.0),
              Seconds.of(10.0)),
          new SysIdRoutine.Mechanism(
              voltage -> setSysIdAngularVolts(voltage.in(Volts)),
              log -> {
                double meanDriveVolts =
                    0.25
                        * (m_frontLeft.getDriveAppliedVolts()
                            + m_frontRight.getDriveAppliedVolts()
                            + m_rearLeft.getDriveAppliedVolts()
                            + m_rearRight.getDriveAppliedVolts());
                log.motor("drive-angular")
                    .voltage(m_sysIdVoltage.mut_replace(meanDriveVolts, Volts))
                    .angularPosition(m_sysIdAngle.mut_replace(getContinuousGyroDegrees(), Degrees))
                    .angularVelocity(
                        m_sysIdAngularVelocity.mut_replace(getTurnRate(), DegreesPerSecond));
              },
              this,
              "drive-angular"));

  /**
   * Module steering (azimuth) SysId routine: holds drive wheels at {@code 0 V} and characterizes
   * all four NEO 550 steering motors against continuous unwrapped Through Bore encoder angles.
   */
  private final SysIdRoutine m_steerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1.0).per(Second),
              Volts.of(6.0),
              Seconds.of(8.0)),
          new SysIdRoutine.Mechanism(
              voltage -> setSysIdSteerVolts(voltage.in(Volts)),
              log -> {
                logSteerModule(log, "steer-front-left", m_frontLeft);
                logSteerModule(log, "steer-front-right", m_frontRight);
                logSteerModule(log, "steer-rear-left", m_rearLeft);
                logSteerModule(log, "steer-rear-right", m_rearRight);
              },
              this,
              "drive-steer"));

  private void logDriveModuleLinear(
      edu.wpi.first.wpilibj.sysid.SysIdRoutineLog log, String name, MAXSwerveModule module) {
    log.motor(name)
        .voltage(m_sysIdVoltage.mut_replace(module.getDriveAppliedVolts(), Volts))
        .linearPosition(m_sysIdDistance.mut_replace(module.getDrivePositionMeters(), Meters))
        .linearVelocity(
            m_sysIdLinearVelocity.mut_replace(
                module.getDriveVelocityMetersPerSecond(), MetersPerSecond));
  }

  private void logSteerModule(
      edu.wpi.first.wpilibj.sysid.SysIdRoutineLog log, String name, MAXSwerveModule module) {
    log.motor(name)
        .voltage(m_sysIdVoltage.mut_replace(module.getSteerAppliedVolts(), Volts))
        .angularPosition(
            m_sysIdAngle.mut_replace(module.getContinuousSteerPositionRadians(), Radians))
        .angularVelocity(
            m_sysIdAngularVelocity.mut_replace(
                module.getSteerVelocityRadiansPerSecond(), RadiansPerSecond));
  }

  /**
   * Computes the four chassis-relative module angles (FL, FR, RL, RR) tangent to the module radius
   * vectors so {@code +Volts} on all drive motors produces pure counter-clockwise ({@code +omega})
   * rotation with zero radial wheel scrub.
   */
  static Rotation2d[] computeAngularSysIdModuleAngles() {
    double halfLength = DriveConstants.kWheelBase / 2.0;
    double halfWidth = DriveConstants.kTrackWidth / 2.0;
    // For module position (x, y), the CCW tangent direction is (-y, +x).
    return new Rotation2d[] {
      new Rotation2d(-halfWidth, halfLength),   // FL (+x, +y)
      new Rotation2d(halfWidth, halfLength),    // FR (+x, -y)
      new Rotation2d(-halfWidth, -halfLength),  // RL (-x, +y)
      new Rotation2d(halfWidth, -halfLength)    // RR (-x, -y)
    };
  }

  private void clearHeadingHoldForSysId() {
    m_useHeadingController = false;
    m_headingPID.reset();
  }

  private void alignModulesForLinearSysId() {
    clearHeadingHoldForSysId();
    m_frontLeft.alignForSysId(Rotation2d.kZero);
    m_frontRight.alignForSysId(Rotation2d.kZero);
    m_rearLeft.alignForSysId(Rotation2d.kZero);
    m_rearRight.alignForSysId(Rotation2d.kZero);
  }

  private boolean areModulesAlignedForLinearSysId() {
    return m_frontLeft.isAlignedToAngle(Rotation2d.kZero, kSysIdAlignToleranceRad)
        && m_frontRight.isAlignedToAngle(Rotation2d.kZero, kSysIdAlignToleranceRad)
        && m_rearLeft.isAlignedToAngle(Rotation2d.kZero, kSysIdAlignToleranceRad)
        && m_rearRight.isAlignedToAngle(Rotation2d.kZero, kSysIdAlignToleranceRad);
  }

  private void alignModulesForAngularSysId() {
    clearHeadingHoldForSysId();
    m_frontLeft.alignForSysId(kAngularSysIdModuleAngles[0]);
    m_frontRight.alignForSysId(kAngularSysIdModuleAngles[1]);
    m_rearLeft.alignForSysId(kAngularSysIdModuleAngles[2]);
    m_rearRight.alignForSysId(kAngularSysIdModuleAngles[3]);
  }

  private boolean areModulesAlignedForAngularSysId() {
    return m_frontLeft.isAlignedToAngle(kAngularSysIdModuleAngles[0], kSysIdAlignToleranceRad)
        && m_frontRight.isAlignedToAngle(kAngularSysIdModuleAngles[1], kSysIdAlignToleranceRad)
        && m_rearLeft.isAlignedToAngle(kAngularSysIdModuleAngles[2], kSysIdAlignToleranceRad)
        && m_rearRight.isAlignedToAngle(kAngularSysIdModuleAngles[3], kSysIdAlignToleranceRad);
  }

  private void setSysIdLinearVolts(double volts) {
    clearHeadingHoldForSysId();
    m_frontLeft.setSysIdDriveVoltsAtAngle(volts, Rotation2d.kZero);
    m_frontRight.setSysIdDriveVoltsAtAngle(volts, Rotation2d.kZero);
    m_rearLeft.setSysIdDriveVoltsAtAngle(volts, Rotation2d.kZero);
    m_rearRight.setSysIdDriveVoltsAtAngle(volts, Rotation2d.kZero);
  }

  private void setSysIdAngularVolts(double volts) {
    clearHeadingHoldForSysId();
    m_frontLeft.setSysIdDriveVoltsAtAngle(volts, kAngularSysIdModuleAngles[0]);
    m_frontRight.setSysIdDriveVoltsAtAngle(volts, kAngularSysIdModuleAngles[1]);
    m_rearLeft.setSysIdDriveVoltsAtAngle(volts, kAngularSysIdModuleAngles[2]);
    m_rearRight.setSysIdDriveVoltsAtAngle(volts, kAngularSysIdModuleAngles[3]);
  }

  private void setSysIdSteerVolts(double volts) {
    clearHeadingHoldForSysId();
    m_frontLeft.setSysIdSteerVolts(volts);
    m_frontRight.setSysIdSteerVolts(volts);
    m_rearLeft.setSysIdSteerVolts(volts);
    m_rearRight.setSysIdSteerVolts(volts);
  }

  /**
   * Returns the continuous (unwrapped) counter-clockwise gyro angle in degrees, suitable for
   * multi-turn angular SysId characterization without +/-180 deg wrap discontinuities.
   */
  public double getContinuousGyroDegrees() {
    if (m_isSim) {
      return m_simGyro.getAngleDegrees();
    }
    return m_gyro.getAngle() * (DriveConstants.kGyroReversed ? 1.0 : -1.0);
  }

  /** Mean linear wheel position (meters) across all four drive modules. */
  public double getAverageDrivePositionMeters() {
    return 0.25
        * (m_frontLeft.getDrivePositionMeters()
            + m_frontRight.getDrivePositionMeters()
            + m_rearLeft.getDrivePositionMeters()
            + m_rearRight.getDrivePositionMeters());
  }

  /** Returns true when chassis translational and angular velocities have settled near zero. */
  public boolean isStopped() {
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    return Math.abs(speeds.vxMetersPerSecond) < 0.05
        && Math.abs(speeds.vyMetersPerSecond) < 0.05
        && Math.abs(speeds.omegaRadiansPerSecond) < 0.08;
  }

  /** Pre-aligns modules to 0 deg and runs a linear quasistatic SysId test. */
  public Command sysIdLinearQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.run(this::alignModulesForLinearSysId, this)
        .until(this::areModulesAlignedForLinearSysId)
        .withTimeout(kSysIdAlignTimeoutSeconds)
        .andThen(m_linearSysIdRoutine.quasistatic(direction));
  }

  /** Pre-aligns modules to 0 deg and runs a linear dynamic SysId test. */
  public Command sysIdLinearDynamic(SysIdRoutine.Direction direction) {
    return Commands.run(this::alignModulesForLinearSysId, this)
        .until(this::areModulesAlignedForLinearSysId)
        .withTimeout(kSysIdAlignTimeoutSeconds)
        .andThen(m_linearSysIdRoutine.dynamic(direction));
  }

  /** Pre-aligns modules tangent to the chassis circle and runs an angular quasistatic SysId test. */
  public Command sysIdAngularQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.run(this::alignModulesForAngularSysId, this)
        .until(this::areModulesAlignedForAngularSysId)
        .withTimeout(kSysIdAlignTimeoutSeconds)
        .andThen(m_angularSysIdRoutine.quasistatic(direction));
  }

  /** Pre-aligns modules tangent to the chassis circle and runs an angular dynamic SysId test. */
  public Command sysIdAngularDynamic(SysIdRoutine.Direction direction) {
    return Commands.run(this::alignModulesForAngularSysId, this)
        .until(this::areModulesAlignedForAngularSysId)
        .withTimeout(kSysIdAlignTimeoutSeconds)
        .andThen(m_angularSysIdRoutine.dynamic(direction));
  }

  /** Runs a steering-motor quasistatic SysId test across all four swerve modules. */
  public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(this::clearHeadingHoldForSysId)
        .andThen(m_steerSysIdRoutine.quasistatic(direction));
  }

  /** Runs a steering-motor dynamic SysId test across all four swerve modules. */
  public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(this::clearHeadingHoldForSysId)
        .andThen(m_steerSysIdRoutine.dynamic(direction));
  }
}