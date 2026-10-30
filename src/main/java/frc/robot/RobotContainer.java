package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// --- PATHPLANNER IMPORTS ---
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AllianceSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.AutoAimAndShoot;
import frc.robot.commands.StowIntake;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.driverprofiles.DriverControlsProfile;
import frc.robot.driverprofiles.ThrustmasterProfile;
import frc.robot.driverprofiles.XboxHanifStyleProfile;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;


public class RobotContainer {
    public static boolean isAutoAimFinished = false;
    public static boolean isAutoDistanceFinished = false;

    private static final double MANUAL_INTAKE_SPEED_DOWN = 0.2;
    private static final double MANUAL_INTAKE_SPEED_UP = 0.2;
    // Auto aim points the shooter/back side at the hub, so negative robot X moves closer.
    private static final double AUTO_DISTANCE_DIRECTION = -1.0;

    public enum DriverProfile {
        THRUSTMASTER("Thrustmaster"),
        XBOX_HANIF_STYLE("XBox-HanifStyle");

        public final String displayName;
        DriverProfile(String displayName) { this.displayName = displayName; }
    }

    private final VisionSubsystem visionSystem = new VisionSubsystem();
    private final AllianceSubsystem allianceSubsystem = new AllianceSubsystem();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem(visionSystem, allianceSubsystem);
    private final Shooter shooter = Shooter.getInstance();
    private final Intake intake = Intake.getInstance();
    private final SysIdSupport m_sysIdSupport = new SysIdSupport(m_robotDrive, shooter, intake);

    private final CommandXboxController xboxController = new CommandXboxController(OIConstants.kDriverControllerPort);
    private final Joystick thrustmasterController = new Joystick(OIConstants.kDriverControllerPort);
    private final ThrustmasterProfile thrustmasterProfile = new ThrustmasterProfile(thrustmasterController);
    private final DriverControlsProfile xboxHanifStyleProfile = new XboxHanifStyleProfile(xboxController);
    
    // CHOOSERS
    private final SendableChooser<DriverProfile> profileChooser = new SendableChooser<>();
    private final SendableChooser<ThrustmasterProfile.DriveSpeedMode> thrustmasterSpeedChooser = new SendableChooser<>();
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter rotLimiter    = new SlewRateLimiter(2.0);
    private final PIDController m_visionPID = new PIDController(0.03, 0.0, 0.001);
    private final PIDController m_headingSnapPID = new PIDController(0.03, 0.0, 0.001);

    private DriverProfile m_lastProfile = null;
    private boolean m_isSlowMode = false;

    /** Latched once the robot has lined up; ranging then happens while still tracking the hub. */
    private boolean m_rangingEngaged = false;

    public RobotContainer() {
        m_visionPID.enableContinuousInput(-180.0, 180.0);
        m_headingSnapPID.enableContinuousInput(-180.0, 180.0);
        m_headingSnapPID.setTolerance(2.0);

        AutoTuning.publishDefaults();
        ShooterTuning.publishDefaults();

        // Hand the shooter a live view of the aim geometry and chassis motion.
        // This is what lets it hold the feeder if the shot is not lined up.
        shooter.setShotInputSupplier(this::shotInputs);

        // In simulation: give the camera sim the drivetrain's ground truth so
        // the PhotonVision fusion pipeline runs against synthetic AprilTag
        // frames. No-op on the real robot.
        visionSystem.setupSimulation(m_robotDrive::getSimPose);

        configurePathPlanner();
        configureProfileChooser();
        configureThrustmasterSpeedChooser();
        configureDefaultDriveCommand();
        configureBindings();
    }

    /**
     * Simulation-only hook, called from Robot.simulationPeriodic (runs after
     * the command scheduler, i.e. after every subsystem's simulationPeriodic).
     * Aggregates all mechanism current draws and updates the simulated battery
     * bus voltage, producing realistic voltage sag under load.
     */
    public void simulationPeriodic() {
        double rawCurrentDrawAmps =
            m_robotDrive.getSimulatedCurrentDraw()
                + shooter.getSimulatedCurrentDraw()
                + intake.getSimulatedCurrentDraw();
        double clampedCurrentAmps =
            Double.isFinite(rawCurrentDrawAmps) ? Math.max(0.0, rawCurrentDrawAmps) : 0.0;
        double loadedBusVoltage =
            MathUtil.clamp(
                BatterySim.calculateDefaultBatteryLoadedVoltage(clampedCurrentAmps),
                6.0,
                12.5);

        RoboRioSim.setVInVoltage(loadedBusVoltage);
    }

    private void configurePathPlanner() {
        NamedCommands.registerCommand("DeployIntake", new DeployIntake(intake));
        NamedCommands.registerCommand("StowIntake",   new StowIntake(intake));
        NamedCommands.registerCommand("StopRoller", new InstantCommand(intake::stopRoller, intake));
        NamedCommands.registerCommand("StartRoller", new InstantCommand(intake::runRoller, intake));
        
        NamedCommands.registerCommand("AutoDeployAndSpin", 
            Commands.parallel(new DeployIntake(intake), new InstantCommand(intake::runRoller))
        );

        // Register AutoAimAndShoot for 20 seconds
    NamedCommands.registerCommand("AutoAimAndShoot", new AutoAimAndShoot(m_robotDrive, shooter, intake, 20.0));

    // Also update ShootForXseconds (already used in your .auto files) to use it
    NamedCommands.registerCommand("ShootForXseconds", new AutoAimAndShoot(m_robotDrive, shooter, intake, 20.0));

        autoChooser.setDefaultOption("Do Nothing", "NONE");
        autoChooser.addOption("xy changes", "xy changes");
        autoChooser.addOption("Blue left", "Blue left");
        autoChooser.addOption("Blue right", "Blue right");
        autoChooser.addOption("y changes", "y changes");
        autoChooser.addOption("x changes", "x changes");
        SysIdSupport.registerAutoChooserOptions(autoChooser);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureProfileChooser() {
        profileChooser.setDefaultOption(DriverProfile.THRUSTMASTER.displayName, DriverProfile.THRUSTMASTER);
        profileChooser.addOption(DriverProfile.XBOX_HANIF_STYLE.displayName, DriverProfile.XBOX_HANIF_STYLE);
        SmartDashboard.putData("Driver Profile", profileChooser);
    }

    private void configureThrustmasterSpeedChooser() {
        thrustmasterSpeedChooser.setDefaultOption("Standard", ThrustmasterProfile.DriveSpeedMode.STANDARD);
        thrustmasterSpeedChooser.addOption("Half-speed", ThrustmasterProfile.DriveSpeedMode.HALF_SPEED);
        SmartDashboard.putData("Thrustmaster Speed Mode", thrustmasterSpeedChooser);
    }

    private void configureDefaultDriveCommand() {
        m_robotDrive.setDefaultCommand(
            new RunCommand(() -> {
                DriverProfile currentProfile = profileChooser.getSelected();
                if (currentProfile == null) currentProfile = DriverProfile.THRUSTMASTER;

                if (currentProfile != m_lastProfile) {
                    xSpeedLimiter.reset(0); ySpeedLimiter.reset(0); rotLimiter.reset(0);
                    m_lastProfile = currentProfile;
                }

                DriverControlsProfile profile = getActiveProfile(currentProfile);
                thrustmasterProfile.setDriveSpeedMode(getSelectedThrustmasterSpeedMode());
                double multiplier = currentProfile == DriverProfile.THRUSTMASTER
                    ? profile.getDriveSpeedMultiplier()
                    : (m_isSlowMode ? 0.3 : 1.0);
                double x = profile.getXSpeed();
                double y = profile.getYSpeed();
                double rot = profile.getRotation();

                m_robotDrive.drive(
                    xSpeedLimiter.calculate(x * multiplier),
                    ySpeedLimiter.calculate(y * multiplier),
                    rotLimiter.calculate(rot * multiplier)
                );
            }, m_robotDrive)
        );
    }

    private void configureBindings() {
        // ALLIANCE (simulation only): switch between the red and blue alliance.
        // Paths authored on the blue side are flipped for the red alliance, and
        // the robot is moved to the matching spot on the other side of the
        // field, so a red-alliance match can be rehearsed from the sim.
        if (RobotBase.isSimulation()) {
            activeProfileTrigger(DriverControlsProfile::toggleAllianceTrigger).onTrue(
                new InstantCommand(allianceSubsystem::toggleAlliance, allianceSubsystem)
            );
        }

        // Manual full-pose correction from vision, including field heading.
        activeProfileTrigger(DriverControlsProfile::forceReseedTrigger).onTrue(
            new InstantCommand(m_robotDrive::correctTranslationFromVision, m_robotDrive)
        );

        // Find start pose and follow path through right trench
        activeProfileTrigger(DriverControlsProfile::thruRightTrench).onTrue(driveToRightTrench());

        // Find start pose and follow path through left trench
        activeProfileTrigger(DriverControlsProfile::thruLeftTrench).onTrue(driveToLeftTrench());

        // SHOOTER and AUTO-AIM
        Trigger shootTrigger = activeProfileTrigger(DriverControlsProfile::shootTrigger);
        Trigger alignmentOverrideTrigger =
            activeProfileTrigger(DriverControlsProfile::disableAutoAimTrigger);

        shootTrigger.whileTrue(new FunctionalCommand(
            () -> {
                shooter.setFrontTargetRPM(SmartDashboard.getNumber("Front Shooter RPM Setpoint", shooter.m_fronttargetRPM));
                shooter.setBackTargetRPM(SmartDashboard.getNumber("Back Shooter RPM Setpoint", shooter.m_backtargetRPM));
                shooter.setAlignmentOverride(alignmentOverrideTrigger.getAsBoolean());
                shooter.start();
                intake.stopRoller();
            },
            () -> shooter.setAlignmentOverride(alignmentOverrideTrigger.getAsBoolean()),
            interrupted -> {
                shooter.setAlignmentOverride(false);
                shooter.stopAll();
            },
            () -> false,
            shooter
        ));

        // Thrustmaster button 6 is a hold-to-disable override. Including it in
        // this trigger also cancels an active alignment command immediately,
        // returning drivetrain control to the driver while the shooter spins.
        shootTrigger
            .and(alignmentOverrideTrigger.negate())
            .whileTrue(autoAimThenDistanceCommand());

        // Heading snap-tos
        activeProfileTrigger(DriverControlsProfile::headingUpTrigger).onTrue(snapToHeadingCommand(0.0));
        activeProfileTrigger(DriverControlsProfile::headingRightTrigger).onTrue(snapToHeadingCommand(-90.0));
        activeProfileTrigger(DriverControlsProfile::headingDownTrigger).onTrue(snapToHeadingCommand(180.0));
        activeProfileTrigger(DriverControlsProfile::headingLeftTrigger).onTrue(snapToHeadingCommand(90.0));

        // INTAKE
        activeProfileTrigger(DriverControlsProfile::deployIntakeTrigger).onTrue(new DeployIntake(intake));
        activeProfileTrigger(DriverControlsProfile::stowIntakeTrigger).onTrue(new StowIntake(intake));
        activeProfileTrigger(DriverControlsProfile::manualIntakeUpTrigger).whileTrue(
            new StartEndCommand(
                () -> intake.setDeployMotorSpeed(MANUAL_INTAKE_SPEED_DOWN),
                () -> intake.setDeployMotorSpeed(0.0),
                intake
            )
        );
        activeProfileTrigger(DriverControlsProfile::manualIntakeDownTrigger).whileTrue(
            new StartEndCommand(
                () -> intake.setDeployMotorSpeed(-MANUAL_INTAKE_SPEED_UP),
                () -> intake.setDeployMotorSpeed(0.0),
                intake
            )
        );
        activeProfileTrigger(DriverControlsProfile::startShotCycle)
            .onTrue(new InstantCommand(intake::startShotCycle, intake))
            .onFalse(new InstantCommand(intake::stopShotCycle, intake));

        // SHARED
        activeProfileTrigger(DriverControlsProfile::toggleRollerTrigger).onTrue(new InstantCommand(intake::toggleRoller));

        // SYSTEM IDENTIFICATION (WPILib SysId):
        // Hold Thrustmaster unused right-bank buttons 11 / 13 / 15 / 16 (or Xbox Back + A/B/X/Y)
        // to run Quasistatic Fwd/Rev and Dynamic Fwd/Rev on the mechanism chosen in "SysId Mechanism".
        // Releasing the button immediately stops the mechanism. Suppressed when attached to FMS.
        Trigger sysIdAllowed = new Trigger(() -> !DriverStation.isFMSAttached());
        activeProfileTrigger(DriverControlsProfile::sysIdQuasistaticForwardTrigger)
            .and(sysIdAllowed)
            .whileTrue(
                m_sysIdSupport.createDeferredTestCommand(
                    SysIdSupport.TestType.QUASISTATIC_FORWARD));
        activeProfileTrigger(DriverControlsProfile::sysIdQuasistaticReverseTrigger)
            .and(sysIdAllowed)
            .whileTrue(
                m_sysIdSupport.createDeferredTestCommand(
                    SysIdSupport.TestType.QUASISTATIC_REVERSE));
        activeProfileTrigger(DriverControlsProfile::sysIdDynamicForwardTrigger)
            .and(sysIdAllowed)
            .whileTrue(
                m_sysIdSupport.createDeferredTestCommand(
                    SysIdSupport.TestType.DYNAMIC_FORWARD));
        activeProfileTrigger(DriverControlsProfile::sysIdDynamicReverseTrigger)
            .and(sysIdAllowed)
            .whileTrue(
                m_sysIdSupport.createDeferredTestCommand(
                    SysIdSupport.TestType.DYNAMIC_REVERSE));
        };

    private Command autoAimThenDistanceCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                isAutoAimFinished = false;
                isAutoDistanceFinished = false;
                m_rangingEngaged = false;
                m_visionPID.reset();
                xSpeedLimiter.reset(0.0);
                ySpeedLimiter.reset(0.0);
                rotLimiter.reset(0.0);
            }),
            // One command that keeps aiming for as long as the button is held.
            // The old sequence stopped correcting heading the moment it moved
            // on to ranging, so a push from defense or a bump during the drive
            // left the robot firing several degrees off the hub.
            new RunCommand(this::trackHubAndRange, m_robotDrive)
        );
    }

    /**
     * Aim at the hub and, once lined up, range to the calibrated shooting
     * distance - while continuing to correct heading. With the distance table
     * enabled the robot shoots from where it stands instead of driving.
     */
    private void trackHubAndRange() {
        m_visionPID.setPID(AutoTuning.aimHeadingP(), 0.0, 0.0);

        double currentHeading = m_robotDrive.getPose().getRotation().getDegrees();
        double targetHeading = m_robotDrive.getAngleToHub();
        double headingError = MathUtil.inputModulus(targetHeading - currentHeading, -180.0, 180.0);

        SmartDashboard.putNumber("Auto Aim Error (deg)", headingError);
        isAutoAimFinished = Math.abs(headingError) <= AutoTuning.aimHeadingToleranceDeg();

        double rot = m_visionPID.calculate(currentHeading, targetHeading);

        if (!m_rangingEngaged) {
            if (!isAutoAimFinished) {
                isAutoDistanceFinished = false;
                m_robotDrive.drive(0.0, 0.0, rot);
                return;
            }
            m_rangingEngaged = true;
            xSpeedLimiter.reset(0.0);
        }

        double distanceError = getAutoDistanceErrorMeters();
        SmartDashboard.putNumber("Auto Distance Error (m)", distanceError);

        if (ShooterTuning.useDistanceTable()) {
            // The table adapts the shot to the current distance, so there is
            // nothing to drive to - hold position and keep tracking.
            isAutoDistanceFinished = true;
            m_robotDrive.drive(0.0, 0.0, rot);
            return;
        }

        isAutoDistanceFinished =
            Math.abs(distanceError) <= AutoTuning.aimDistanceToleranceMeters();
        if (isAutoDistanceFinished) {
            m_robotDrive.drive(0.0, 0.0, rot);
            return;
        }

        double autoSpeed = AUTO_DISTANCE_DIRECTION * MathUtil.clamp(
            distanceError * AutoTuning.aimDistanceKp(),
            -AutoTuning.aimDistanceMaxSpeed(),
            AutoTuning.aimDistanceMaxSpeed());
        SmartDashboard.putNumber("Auto Distance Speed", autoSpeed);

        // Range with a robot-relative translation, expressed in the field
        // frame so the same call can also keep the heading locked on.
        double headingRadians = m_robotDrive.getPose().getRotation().getRadians();
        double limitedSpeed = xSpeedLimiter.calculate(autoSpeed);
        m_robotDrive.drive(
            limitedSpeed * Math.cos(headingRadians),
            limitedSpeed * Math.sin(headingRadians),
            rot);
    }

    /** Distance error to the calibrated shooting distance, measured from the muzzle. */
    private double getAutoDistanceErrorMeters() {
        double targetDistanceMeters = Units.inchesToMeters(AutoTuning.aimTargetDistanceInches());
        return m_robotDrive.getShotDistanceMeters() - targetDistanceMeters;
    }

    /**
     * Live shooter inputs: hub geometry from the muzzle, chassis speed and yaw
     * rate. Read by {@link Shooter#periodic()} every loop through the supplier
     * installed in the constructor.
     */
    private ShotInputs shotInputs() {
        Pose2d pose = m_robotDrive.getPose();
        ShotMath.AimSolution aim = m_robotDrive.getAimSolution();

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            m_robotDrive.getRobotRelativeSpeeds(), pose.getRotation());
        Translation2d muzzle = ShotMath.muzzlePosition(
            pose.getTranslation(), ShooterConstants.kMuzzleOffsetMeters, pose.getRotation());
        double closingSpeed = ShotMath.closingSpeed(
            muzzle,
            m_robotDrive.getHubLocation(),
            fieldSpeeds.vxMetersPerSecond,
            fieldSpeeds.vyMetersPerSecond);

        double headingError = MathUtil.inputModulus(
            aim.headingDegrees() - pose.getRotation().getDegrees(), -180.0, 180.0);
        // The shooter camera measures the angle to the AprilTag the ball has to
        // go through, in the frame the ball leaves from. getAimSolution() has
        // already folded its residual in when the feature is enabled, so this
        // block only mirrors it onto the dashboard. Off by default: the shot is
        // unchanged until ShooterTuning says to use it.
        if (ShooterTuning.useShooterCameraAim()) {
            ShooterCameraAim.Correction correction = m_robotDrive.getShooterCameraAim();
            SmartDashboard.putBoolean("Shooter/Camera Aim Used", correction.valid());
            SmartDashboard.putString("Shooter/Camera Aim Status", correction.reason());
            SmartDashboard.putNumber("Shooter/Camera Aim Correction (deg)", correction.aimErrorDeg());
            SmartDashboard.putNumber(
                "Shooter/Camera Range Correction (m)", correction.rangeErrorMeters());
        }

        return new ShotInputs(
            aim.distanceMeters(),
            headingError,
            Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond),
            m_robotDrive.getTurnRate(),
            closingSpeed);
    }

    private Command snapToHeadingCommand(double targetDegrees) {
        return Commands.sequence(
            Commands.runOnce(() -> m_headingSnapPID.reset()),
            new RunCommand(() -> {
                DriverProfile selected = profileChooser.getSelected();
                DriverControlsProfile profile = getActiveProfile(selected);

                double multiplier = selected == DriverProfile.THRUSTMASTER
                    ? profile.getDriveSpeedMultiplier()
                    : (m_isSlowMode ? 0.3 : 1.0);

                double x = profile.getXSpeed() * multiplier;
                double y = profile.getYSpeed() * multiplier;
                double rot = m_headingSnapPID.calculate(
                    m_robotDrive.getPose().getRotation().getDegrees(),
                    targetDegrees
                );

                m_robotDrive.drive(
                    xSpeedLimiter.calculate(x),
                    ySpeedLimiter.calculate(y),
                    rot
                );
            }, m_robotDrive).until(m_headingSnapPID::atSetpoint)
        );
    }


    private Command driveToRightTrench() {
        return Commands.defer(() -> loadTrenchPath("blue right trench"), java.util.Set.of(m_robotDrive));
    }

    private Command driveToLeftTrench() {
        return Commands.defer(() -> loadTrenchPath("blue left trench"), java.util.Set.of(m_robotDrive));
    }

    private Command loadTrenchPath(String pathName) {
        if (!AutoBuilder.isConfigured()) {
            DriverStation.reportError("Cannot follow path: AutoBuilder is not configured.", false);
            return Commands.none();
        }
        PathConstraints constraints = new PathConstraints(
            AutoTuning.pathMaxSpeed(),
            AutoTuning.pathMaxAcceleration(),
            AutoTuning.pathMaxAngularSpeedRadPerSec(),
            AutoTuning.pathMaxAngularAccelerationRadPerSecSq()
        );

        try {
            // 1. ALWAYS load the path authored on the Blue side.
            // PathPlanner will automatically flip it for you if you are on Red.
            PathPlannerPath path =
                PathPlannerPath.fromPathFile(pathName);

            // 2. Use pathfindThenFollowPath!
            // This automatically handles flipping the pathfinding destination AND the path itself.
            return AutoBuilder.pathfindThenFollowPath(path, constraints);

        } catch (Exception e) {
            DriverStation.reportError(
                "Unable to load path: " + pathName + ": " + e,
                e.getStackTrace());

            return Commands.none();
        }
    }

    private DriverControlsProfile getActiveProfile(DriverProfile profile) {
        DriverProfile selected = profile == null ? DriverProfile.THRUSTMASTER : profile;
        return selected == DriverProfile.XBOX_HANIF_STYLE
            ? xboxHanifStyleProfile
            : thrustmasterProfile;
    }

    private ThrustmasterProfile.DriveSpeedMode getSelectedThrustmasterSpeedMode() {
        ThrustmasterProfile.DriveSpeedMode selected = thrustmasterSpeedChooser.getSelected();
        return selected == null ? ThrustmasterProfile.DriveSpeedMode.STANDARD : selected;
    }

    private edu.wpi.first.wpilibj2.command.button.Trigger activeProfileTrigger(java.util.function.Function<DriverControlsProfile, edu.wpi.first.wpilibj2.command.button.Trigger> selector) {
        return new edu.wpi.first.wpilibj2.command.button.Trigger(() -> selector.apply(getActiveProfile(profileChooser.getSelected())).getAsBoolean());
    }

    public Command getAutonomousCommand() {
        String selected = autoChooser.getSelected();
        // "Do Nothing" (and any unbuilt/missing auto name) must not construct
        // PathPlannerAuto, which throws when the .auto file does not exist —
        // especially important in simulation where the default option runs
        // every time autonomous is enabled.
        if (selected == null || "NONE".equals(selected)) {
            return Commands.none();
        }
        if (SysIdSupport.isSysIdAutoKey(selected)) {
            return m_sysIdSupport.createAutoCommand(selected);
        }
        if (!AutoBuilder.isConfigured()) {
            DriverStation.reportError(
                "Cannot run autonomous: AutoBuilder is not configured. Check deploy/pathplanner/settings.json.",
                false);
            return Commands.none();
        }
        try {
            return new PathPlannerAuto(selected);
        } catch (Exception e) {
            DriverStation.reportError("Unable to load autonomous '" + selected + "': " + e,
                e.getStackTrace());
            return Commands.none();
        }
    }
}