package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoAimAndShoot extends Command {

    private enum Phase {
        AIMING,
        DISTANCE,
        HOLD
    }

    private final DriveSubsystem drive;
    private final Shooter shooter;
    private final Intake intake;
    private final double durationSeconds;

    private final Timer timer = new Timer();
    private final PIDController visionPID = new PIDController(0.03, 0.0, 0.001);
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2.0);

    private Phase phase = Phase.AIMING;

    private static final String AUTO_AIM_DISTANCE_IN_KEY = "Auto Aim Target Distance (in)";
    private static final double AUTO_AIM_DEFAULT_DISTANCE_IN = 100.0;
    private static final double AUTO_DISTANCE_KP = 1.0;
    private static final double AUTO_DISTANCE_MAX_SPEED = 0.35;
    private static final double AUTO_AIM_TOLERANCE_DEG = 2.0;
    private static final double AUTO_DISTANCE_TOLERANCE_IN = 2.0;
    private static final double AUTO_DISTANCE_DIRECTION = -1.0;

    /**
     * Constructs an AutoAimAndShoot command.
     *
     * @param drive Drivetrain subsystem
     * @param shooter Shooter subsystem
     * @param intake Intake subsystem
     * @param durationSeconds How long the command will run (e.g., 20.0 seconds)
     */
    public AutoAimAndShoot(DriveSubsystem drive, Shooter shooter, Intake intake, double durationSeconds) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.durationSeconds = durationSeconds;

        visionPID.enableContinuousInput(-180.0, 180.0);

        addRequirements(drive, shooter);
    }

    /** Default constructor for a 20-second shot cycle. */
    public AutoAimAndShoot(DriveSubsystem drive, Shooter shooter, Intake intake) {
        this(drive, shooter, intake, 20.0);
    }

    @Override
    public void initialize() {
        timer.restart();
        phase = Phase.AIMING;

        RobotContainer.isAutoAimFinished = false;
        RobotContainer.isAutoDistanceFinished = false;

        visionPID.reset();
        xSpeedLimiter.reset(0.0);
        rotLimiter.reset(0.0);

        // Spin up shooter to setpoints
        shooter.setFrontTargetRPM(SmartDashboard.getNumber("Front Shooter RPM Setpoint", shooter.m_fronttargetRPM));
        shooter.setBackTargetRPM(SmartDashboard.getNumber("Back Shooter RPM Setpoint", shooter.m_backtargetRPM));
        shooter.start();
        intake.stopRoller();
    }

    @Override
    public void execute() {
        switch (phase) {
            case AIMING:
                double currentHeading = drive.getPose().getRotation().getDegrees();
                double targetHeading = drive.getAngleToHub();
                double headingError = MathUtil.inputModulus(targetHeading - currentHeading, -180.0, 180.0);

                SmartDashboard.putNumber("Auto Aim Error (deg)", headingError);

                if (Math.abs(headingError) <= AUTO_AIM_TOLERANCE_DEG) {
                    RobotContainer.isAutoAimFinished = true;
                    phase = Phase.DISTANCE;
                    xSpeedLimiter.reset(0.0);
                    drive.driveRobotRelativeNormalized(0.0, 0.0, 0.0);
                } else {
                    double rot = visionPID.calculate(currentHeading, targetHeading);
                    drive.drive(0.0, 0.0, rotLimiter.calculate(rot));
                }
                break;

            case DISTANCE:
                double distanceError = getAutoDistanceErrorMeters();
                SmartDashboard.putNumber("Auto Distance Error (m)", distanceError);

                if (Math.abs(distanceError) <= Units.inchesToMeters(AUTO_DISTANCE_TOLERANCE_IN)) {
                    RobotContainer.isAutoDistanceFinished = true;
                    phase = Phase.HOLD;
                    drive.driveRobotRelativeNormalized(0.0, 0.0, 0.0);
                } else {
                    double autoSpeed = AUTO_DISTANCE_DIRECTION * MathUtil.clamp(
                        distanceError * AUTO_DISTANCE_KP,
                        -AUTO_DISTANCE_MAX_SPEED,
                        AUTO_DISTANCE_MAX_SPEED
                    );
                    SmartDashboard.putNumber("Auto Distance Speed", autoSpeed);
                    // AUTO_DISTANCE_DIRECTION makes negative robot X move the
                    // shooter/back side of the robot toward the hub.
                    drive.driveRobotRelativeNormalized(
                        xSpeedLimiter.calculate(autoSpeed), 0.0, 0.0);
                }
                break;

            case HOLD:
                SmartDashboard.putNumber("Auto Distance Error (m)", getAutoDistanceErrorMeters());
                drive.driveRobotRelativeNormalized(0.0, 0.0, 0.0);
                break;
        }

        // Note: Shooter.periodic() will automatically start the feeder Kraken and
        // intake conveyor once isAutoAimFinished, isAutoDistanceFinished, and
        // target RPM are all reached.
    }

    private double getAutoDistanceErrorMeters() {
        double targetDistanceInches = SmartDashboard.getNumber(AUTO_AIM_DISTANCE_IN_KEY, AUTO_AIM_DEFAULT_DISTANCE_IN);
        double targetDistanceMeters = Units.inchesToMeters(targetDistanceInches);

        Translation2d robotToHub = drive.getHubLocation().minus(drive.getPose().getTranslation());
        return robotToHub.getNorm() - targetDistanceMeters;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        drive.driveRobotRelativeNormalized(0.0, 0.0, 0.0);
        RobotContainer.isAutoAimFinished = false;
        RobotContainer.isAutoDistanceFinished = false;
    }
}