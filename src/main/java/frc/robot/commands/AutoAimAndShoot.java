package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.ShooterTuning;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Auto aim, range, then feed while holding the hub.
 *
 * <p>Three things changed relative to the old version, all of them aimed at
 * shots that actually go in:
 *
 * <ul>
 *   <li>Heading is corrected in <em>every</em> phase. The old command stopped
 *       steering once it moved on to ranging, so a bump during the drive left
 *       the final shot several degrees off.
 *   <li>With the distance table enabled the robot skips the ranging phase
 *       entirely and shoots from where it stands, using the current distance.
 *   <li>Distance is measured from the muzzle, matching what the shooter gates
 *       on, and the firing decision lives in {@link Shooter} so teleop and
 *       autonomous behave identically.
 * </ul>
 */
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
        double currentHeading = drive.getPose().getRotation().getDegrees();
        double targetHeading = drive.getAngleToHub();
        double headingError = MathUtil.inputModulus(targetHeading - currentHeading, -180.0, 180.0);

        SmartDashboard.putNumber("Auto Aim Error (deg)", headingError);
        RobotContainer.isAutoAimFinished = Math.abs(headingError) <= AUTO_AIM_TOLERANCE_DEG;

        // Track the hub in every phase; reaching the right distance with the
        // wrong heading still misses.
        double rot = rotLimiter.calculate(visionPID.calculate(currentHeading, targetHeading));

        if (phase == Phase.AIMING) {
            if (RobotContainer.isAutoAimFinished) {
                phase = ShooterTuning.useDistanceTable() ? Phase.HOLD : Phase.DISTANCE;
                xSpeedLimiter.reset(0.0);
                if (phase == Phase.HOLD) {
                    RobotContainer.isAutoDistanceFinished = true;
                }
            }
            drive.drive(0.0, 0.0, rot);
            return;
        }

        if (phase == Phase.DISTANCE) {
            double distanceError = getAutoDistanceErrorMeters();
            SmartDashboard.putNumber("Auto Distance Error (m)", distanceError);

            if (Math.abs(distanceError) <= Units.inchesToMeters(AUTO_DISTANCE_TOLERANCE_IN)) {
                RobotContainer.isAutoDistanceFinished = true;
                phase = Phase.HOLD;
                xSpeedLimiter.reset(0.0);
                drive.drive(0.0, 0.0, rot);
                return;
            }

            double autoSpeed = AUTO_DISTANCE_DIRECTION * MathUtil.clamp(
                distanceError * AUTO_DISTANCE_KP,
                -AUTO_DISTANCE_MAX_SPEED,
                AUTO_DISTANCE_MAX_SPEED);
            SmartDashboard.putNumber("Auto Distance Speed", autoSpeed);

            // Command the robot-relative translation in the field frame so the
            // same call can keep correcting heading while ranging.
            double headingRadians = drive.getPose().getRotation().getRadians();
            double limitedSpeed = xSpeedLimiter.calculate(autoSpeed);
            drive.drive(
                limitedSpeed * Math.cos(headingRadians),
                limitedSpeed * Math.sin(headingRadians),
                rot);
            return;
        }

        // HOLD: stay put, keep the hub in the sight line, let Shooter decide
        // when the ball may be fed.
        RobotContainer.isAutoDistanceFinished = true;
        SmartDashboard.putNumber("Auto Distance Error (m)", getAutoDistanceErrorMeters());
        drive.drive(0.0, 0.0, rot);
    }

    /** Distance error to the calibrated shooting distance, measured from the muzzle. */
    private double getAutoDistanceErrorMeters() {
        double targetDistanceInches = SmartDashboard.getNumber(
            AUTO_AIM_DISTANCE_IN_KEY, AUTO_AIM_DEFAULT_DISTANCE_IN);
        double targetDistanceMeters = Units.inchesToMeters(targetDistanceInches);
        return drive.getShotDistanceMeters() - targetDistanceMeters;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        drive.drive(0.0, 0.0, 0.0);
        RobotContainer.isAutoAimFinished = false;
        RobotContainer.isAutoDistanceFinished = false;
    }
}
