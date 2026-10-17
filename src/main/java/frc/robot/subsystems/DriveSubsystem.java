package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

import java.util.Optional;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.driverprofiles.ThrustmasterProfile;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class DriveSubsystem extends SubsystemBase {
  private static final String PATHPLANNER_COMMANDED_OMEGA_KEY =
      "PathPlanner Commanded Omega (rad/s)";

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

  // The gyro sensor. On the real robot this is the navX MXP. In simulation
  // there is no navX hardware, so the heading is integrated from the swerve
  // physics instead (see simulationPeriodic and getSimPose).
  private final AHRS m_gyro = RobotBase.isSimulation() ? null : new AHRS(NavXComType.kMXP_SPI);

  // --- Simulation ground truth (real hardware needs no analog) ---
  // NOTE: declared before m_poseEstimator, whose initializer reads
  // getGyroRotation() (which consults these in simulation).
  private final boolean m_isSim = RobotBase.isSimulation();
  private Pose2d m_simRobotPose = new Pose2d();
  private double m_simOmegaRadPerSecond = 0.0;
  private final Field2d m_simField = new Field2d();

  // vision usage tracking
  private double m_lastVisionFuseTime = -1.0;
  private boolean m_visionActive = false;
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
  private final PhotonCamera m_oldLeftCamera = new PhotonCamera(VisionConstants.kBackCameraName); // now our back camera
  private final PhotonCamera m_frontCamera = new PhotonCamera(VisionConstants.kSideCameraName); // actually side camera now
  private final PhotonPoseEstimator m_oldLeftCameraEstimator;
  private final PhotonPoseEstimator m_frontCameraEstimator;

  // Camera pose estimator ----------------------------------
  // transform for current back camera
  private final Transform3d m_robotToCamera = VisionConstants.kRobotToBackCamera;

  // transform for current side camera
  private final Transform3d m_robotToFrontCamera = VisionConstants.kRobotToSideCamera;

  //------------------------------------------------------------------------------------------------------

  private boolean addPhotonVisionMeasurement() {
    // Do not short-circuit: each camera must get a chance to contribute every loop.
    boolean oldLeftAccepted = fuseCameraMeasurement(
        "GoodCamera_OldLeft", m_oldLeftCamera, m_oldLeftCameraEstimator);
    boolean frontAccepted = fuseCameraMeasurement(
        "FrontCamera", m_frontCamera, m_frontCameraEstimator);
    SmartDashboard.putBoolean("Vision/GoodCamera_OldLeft Accepted", oldLeftAccepted);
    SmartDashboard.putBoolean("Vision/FrontCamera Accepted", frontAccepted);
    return oldLeftAccepted | frontAccepted;
  }

  private boolean fuseCameraMeasurement(
      String cameraName, PhotonCamera camera, PhotonPoseEstimator cameraEstimator) {
    var result = camera.getLatestResult();

    // ---- THE SAFETY SHIELD ----
    if (!result.hasTargets()) return false;

    var best = result.getBestTarget();

    // 1. AMBIGUITY FILTER: Reject highly ambiguous frames
    if (best.getPoseAmbiguity() > 0.2 && best.getPoseAmbiguity() != -1) return false;

    cameraEstimator.setReferencePose(getPose());
    var estOpt = cameraEstimator.update(result);
    if (estOpt.isEmpty()) return false;

    var est = estOpt.get();
    m_lastVisionRotation = est.estimatedPose.toPose2d().getRotation();

    // ---- SAFETY GATE: reject degenerate / impossible camera->tag solves ----
    double dist = best.getBestCameraToTarget().getTranslation().getNorm();
    if (!Double.isFinite(dist) || dist < 0.22) return false;

    Pose2d visionMeasurement = est.estimatedPose.toPose2d();
    if (!isFinite(visionMeasurement)) return false;
    if (!m_hasVisionHeading) {
      // The first good solution establishes the complete pose rather than
      // gradually blending from the alliance-only startup assumption.
      resetOdometryFromKnownFieldPose(visionMeasurement, "VISION INITIAL POSE");
      m_hasVisionHeading = true;
    } else {
      m_poseEstimator.addVisionMeasurement(
          visionMeasurement,
          est.timestampSeconds,
          VecBuilder.fill(0.5, 0.5, 0.5)
      );
      m_fieldHeadingSource = "VISION FUSED";
    }

    // Mark that vision was used this loop
    m_lastVisionFuseTime = Timer.getFPGATimestamp();
    SmartDashboard.putString("Vision/Last Accepted Camera", cameraName);
    return true;
  }

  private static boolean isFinite(Pose2d pose) {
    return Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }

  public Rotation2d getGyroRotation() {
      if (m_isSim) {
        // Heading integrated from the simulated swerve modules (CCW positive),
        // taking the place of the navX in the pose estimator.
        return m_simRobotPose.getRotation();
      }
      return Rotation2d.fromDegrees(-m_gyro.getAngle());
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

    m_oldLeftCameraEstimator = new PhotonPoseEstimator(
      visionSystem.getTagLayout(),
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
      m_robotToCamera
    );
    m_frontCameraEstimator = new PhotonPoseEstimator(
      visionSystem.getTagLayout(),
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
      m_robotToFrontCamera
    );

    // Prefer a trustworthy camera pose at startup. If none is available, seed
    // a field-relative heading from the alliance and let vision correct it as
    // soon as a good AprilTag solution arrives.
    if (!addPhotonVisionMeasurement()) {
      resetOdometryFromKnownFieldPose(
          new Pose2d(getPose().getTranslation(), getAllianceDefaultHeading()),
          "STARTUP ALLIANCE DEFAULT");
    }
    
    // Configure AutoBuilder last
    if (config != null) {
          AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative,      // Consumer<ChassisSpeeds> for output
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.01, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.01, 0.0, 0.0) // Rotation PID constants
            ),
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
    
    // THE SECRET SAUCE: Tell PathPlanner to check our override method before rotating
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

    // Publish the simulated field so the robot pose can be watched in the sim
    // GUI / dashboards (Glass, AdvantageScope).
    if (m_isSim) {
      SmartDashboard.putData("Field", m_simField);
    }
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
    // Driver Station alliance data may arrive after robotInit. Until vision has
    // established heading, correct the startup default when that data arrives.
    DriverStation.Alliance alliance = m_allianceSubsystem.getAlliance();
    if (!m_hasVisionHeading && alliance != m_startupAlliance) {
      m_startupAlliance = alliance;
      resetOdometryFromKnownFieldPose(
          new Pose2d(getPose().getTranslation(), getAllianceDefaultHeading(alliance)),
          "STARTUP ALLIANCE DEFAULT");
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
  
    // 3. Update pose indicator for debugging
    double now = Timer.getFPGATimestamp();
    m_visionActive = (m_lastVisionFuseTime >= 0.0)
                   && ((now - m_lastVisionFuseTime) < 0.25);  // 250 ms window


    // Pose data
    SmartDashboard.putNumber("Robot X (m)", getPose().getX());
    SmartDashboard.putNumber("Robot Y (m)", getPose().getY());
    SmartDashboard.putNumber("Robot Heading (deg)", getPose().getRotation().getDegrees());

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
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    if (m_isSim) {
      // On the real robot the driver physically places the robot at the start
      // of the auto; in simulation we move the ground-truth pose instead so
      // PathPlanner starts from its authored pose.
      m_simRobotPose = new Pose2d(pose.getTranslation(), m_simRobotPose.getRotation());
    }
    // PathPlanner may reset translation at the beginning of an auto, but its
    // authored rotation must not replace the vision/alliance-established heading.
    resetOdometryFromKnownFieldPose(
        new Pose2d(pose.getTranslation(), getPose().getRotation()),
        m_fieldHeadingSource);
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
    m_fieldHeadingSource = source;
  }

  static Rotation2d getAllianceDefaultHeading(DriverStation.Alliance alliance) {
    return Rotation2d.fromDegrees(
        alliance == DriverStation.Alliance.Red ? 0.0 : 180.0);
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
      // Let vision re-establish the heading from the new spot on the field.
      m_hasVisionHeading = false;
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
  
  /** Drive the robot using robot-relative chassis speeds (used by AutoBuilder). */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // This consumer receives PathPlanner's controller output before wheel-speed
    // desaturation, so it is the requested angular velocity rather than a
    // measured or subsequently limited value.
    SmartDashboard.putNumber(
        PATHPLANNER_COMMANDED_OMEGA_KEY, speeds.omegaRadiansPerSecond);

    // Convert chassis speeds to module states
    SwerveModuleState[] states =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    // Desaturate to respect max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, DriveConstants.kMaxSpeedMetersPerSecond);

    // Apply to modules
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_rearLeft.setDesiredState(states[2]);
    m_rearRight.setDesiredState(states[3]);
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
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    if (m_isSim) {
      return Math.toDegrees(m_simOmegaRadPerSecond);
    }
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
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
      // Do not short-circuit, so either camera can provide the correction.
      return correctTranslationFromCamera(
          "GoodCamera_OldLeft", m_oldLeftCamera, m_oldLeftCameraEstimator)
          | correctTranslationFromCamera(
              "FrontCamera", m_frontCamera, m_frontCameraEstimator);
  }

  private boolean correctTranslationFromCamera(
      String cameraName, PhotonCamera camera, PhotonPoseEstimator cameraEstimator) {
      var result = camera.getLatestResult();
      
      // Safety shield: Reject if there are no targets
      if (!result.hasTargets()) return false;

      var best = result.getBestTarget();
      
      // Strict Ambiguity Filter: Only seed if the calculation is highly precise
      if (best.getPoseAmbiguity() > 0.15 && best.getPoseAmbiguity() != -1) return false;

      // Distance Guard: Don't seed from completely across the stadium floor
      double dist = best.getBestCameraToTarget().getTranslation().getNorm();
      if (!Double.isFinite(dist) || dist > 4.0 || dist < 0.22) return false;

      cameraEstimator.setReferencePose(getPose());
      var update = cameraEstimator.update(result);
      if (update.isPresent()) {
          Pose2d visionPose = update.get().estimatedPose.toPose2d();
          if (!isFinite(visionPose)) return false;

          resetOdometryFromKnownFieldPose(
              visionPose, "MANUAL VISION CORRECTION: " + cameraName);
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

  /** Calculates the exact field-relative angle the robot needs to face to shoot at the Hub */
  public double getAngleToHub() {
      // 1. Calculate distance from the ROBOT to the HUB
      // UPDATED: Now uses getHubLocation()
      double xDist = getHubLocation().getX() - getPose().getX();
      double yDist = getHubLocation().getY() - getPose().getY();
      
      // 2. Find the angle and apply the -180 degree offset
      return Math.toDegrees(Math.atan2(yDist, xDist)) - 180.0;
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
   * integrated heading (through {@link #getGyroRotation()}), so estimation and
   * ground truth track each other exactly as they would on the real robot.
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
}