package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    private static final String AUTO_AIM_DISTANCE_IN_KEY = "Auto Aim Target Distance (in)";
    private static final double AUTO_AIM_DEFAULT_DISTANCE_IN = 100.0;
    private static final double AUTO_DISTANCE_KP = 1.0;
    private static final double AUTO_DISTANCE_MAX_SPEED = 0.35;
    private static final double AUTO_AIM_TOLERANCE_DEG = 2.0;
    private static final double AUTO_DISTANCE_TOLERANCE_IN = 2.0;
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

    public RobotContainer() {
        m_visionPID.enableContinuousInput(-180.0, 180.0);
        m_headingSnapPID.enableContinuousInput(-180.0, 180.0);
        m_headingSnapPID.setTolerance(2.0);

        SmartDashboard.putNumber(AUTO_AIM_DISTANCE_IN_KEY, AUTO_AIM_DEFAULT_DISTANCE_IN);

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
        double totalCurrentDrawAmps = m_robotDrive.getSimulatedCurrentDraw()
            + shooter.getSimulatedCurrentDraw()
            + intake.getSimulatedCurrentDraw();

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentDrawAmps));
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
        autoChooser.addOption("1st Try Kawser", "1st Try Kawser");
        autoChooser.addOption("Autov1", "Autov1");
        autoChooser.addOption("right auto", "right auto");
        autoChooser.addOption("2m and turn", "2m and turn");
        autoChooser.addOption("Blue left", "Blue left");
        autoChooser.addOption("Blue right", "Blue right");
        autoChooser.addOption("New Auto", "New Auto");
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
        };

    private Command autoAimThenDistanceCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                isAutoAimFinished = false;
                isAutoDistanceFinished = false;
                m_visionPID.reset();
                xSpeedLimiter.reset(0.0);
                ySpeedLimiter.reset(0.0);
                rotLimiter.reset(0.0);
            }),
            new RunCommand(this::runAutoAim, m_robotDrive).until(this::isAutoAimFinished),
            Commands.runOnce(() -> {
                xSpeedLimiter.reset(0.0);
                ySpeedLimiter.reset(0.0);
            }),
            new RunCommand(this::runAutoDistance, m_robotDrive).until(this::isAutoDistanceFinished),
            new RunCommand(this::holdAutoDistance, m_robotDrive)
        );
    }

    private void runAutoAim() {
        double rot = m_visionPID.calculate(
            m_robotDrive.getPose().getRotation().getDegrees(),
            m_robotDrive.getAngleToHub());

        m_robotDrive.drive(
            xSpeedLimiter.calculate(0.0),
            ySpeedLimiter.calculate(0.0),
            rot);
    }

    private boolean isAutoAimFinished() {
        double headingError = MathUtil.inputModulus(
            m_robotDrive.getAngleToHub() - m_robotDrive.getPose().getRotation().getDegrees(),
            -180.0,
            180.0);

        SmartDashboard.putNumber("Auto Aim Error (deg)", headingError);
        isAutoAimFinished = Math.abs(headingError) <= AUTO_AIM_TOLERANCE_DEG;
        return isAutoAimFinished;
    }

    private void runAutoDistance() {
        double distanceError = getAutoDistanceErrorMeters();
        double autoSpeed = AUTO_DISTANCE_DIRECTION * MathUtil.clamp(
            distanceError * AUTO_DISTANCE_KP,
            -AUTO_DISTANCE_MAX_SPEED,
            AUTO_DISTANCE_MAX_SPEED);

        SmartDashboard.putNumber("Auto Distance Error (m)", distanceError);
        SmartDashboard.putNumber("Auto Distance Speed", autoSpeed);

        m_robotDrive.driveRobotRelativeNormalized(
            xSpeedLimiter.calculate(autoSpeed),
            0.0,
            0.0);
    }

    private boolean isAutoDistanceFinished() {
        isAutoDistanceFinished =
            Math.abs(getAutoDistanceErrorMeters()) <= Units.inchesToMeters(AUTO_DISTANCE_TOLERANCE_IN);
        return isAutoDistanceFinished;
    }

    private double getAutoDistanceErrorMeters() {
        double targetDistanceInches = SmartDashboard.getNumber(AUTO_AIM_DISTANCE_IN_KEY, AUTO_AIM_DEFAULT_DISTANCE_IN);
        double targetDistanceMeters = Units.inchesToMeters(targetDistanceInches);

        Translation2d robotToHub = m_robotDrive.getHubLocation().minus(m_robotDrive.getPose().getTranslation());
        double distanceToHub = robotToHub.getNorm();

        return distanceToHub - targetDistanceMeters;
    }

    private void holdAutoDistance() {
        SmartDashboard.putNumber("Auto Distance Error (m)", getAutoDistanceErrorMeters());
        m_robotDrive.driveRobotRelativeNormalized(0.0, 0.0, 0.0);
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
    PathConstraints constraints = new PathConstraints(
        2.0,
        2.0,
        Units.degreesToRadians(360),
        Units.degreesToRadians(540)
    );

    try {
        // 1. ALWAYS load the path authored on the Blue side.
        // PathPlanner will automatically flip it for you if you are on Red.
        PathPlannerPath path =
            PathPlannerPath.fromPathFile("blue right trench");

        // 2. Use pathfindThenFollowPath!
        // This automatically handles flipping the pathfinding destination AND the path itself.
        return AutoBuilder.pathfindThenFollowPath(path, constraints);

    } catch (Exception e) {
        DriverStation.reportError(
            "Unable to load path: blue right trench",
            e.getStackTrace());

        return Commands.none();
    }
}

private Command driveToLeftTrench() {
    PathConstraints constraints = new PathConstraints(
        2.0,
        2.0,
        Units.degreesToRadians(360),
        Units.degreesToRadians(540)
    );

    try {
        // 1. ALWAYS load the path authored on the Blue side.
        // PathPlanner will automatically flip it for you if you are on Red.
        PathPlannerPath path =
            PathPlannerPath.fromPathFile("blue left trench");

        // 2. Use pathfindThenFollowPath!
        // This automatically handles flipping the pathfinding destination AND the path itself.
        return AutoBuilder.pathfindThenFollowPath(path, constraints);

    } catch (Exception e) {
        DriverStation.reportError(
            "Unable to load path: blue left trench",
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
        return new PathPlannerAuto(selected);
    }
}