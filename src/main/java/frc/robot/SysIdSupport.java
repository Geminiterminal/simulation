package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.util.Set;

/**
 * Coordinates WPILib {@link SysIdRoutine} characterization commands across all robot subsystems
 * ({@link DriveSubsystem}, {@link Shooter}, and {@link Intake}), exposing:
 *
 * <ul>
 *   <li>A {@code "SysId Mechanism"} {@link SendableChooser} on SmartDashboard for selecting which
 *       mechanism is active for controller hold-buttons and dashboard test buttons.
 *   <li>Deferred deadman-safe commands bound to the four unused Thrustmaster T.16000M FCS buttons
 *       (11, 13, 15, 16) and SmartDashboard buttons.
 *   <li>Autonomous chooser keys and command builders for both individual SysId tests and complete
 *       4-part (Quasistatic Fwd/Rev + Dynamic Fwd/Rev) characterization sequences.
 * </ul>
 */
public final class SysIdSupport {
  public static final String MECHANISM_CHOOSER_KEY = "SysId Mechanism";
  public static final String AUTO_MAX_DRIVE_DISTANCE_KEY = "SysId/Auto Max Drive Distance (m)";
  public static final String THRUSTMASTER_MAPPING_KEY = "SysId/Thrustmaster Buttons";
  public static final String SYSID_AUTO_PREFIX = "SYSID:";

  private static final double DEFAULT_AUTO_MAX_DRIVE_DISTANCE_METERS = 3.5;
  private static final double MIN_AUTO_MAX_DRIVE_DISTANCE_METERS = 0.5;
  private static final double MAX_AUTO_MAX_DRIVE_DISTANCE_METERS = 12.0;
  private static final double SETTLE_PAUSE_SECONDS = 0.5;

  /** Robot mechanisms available for WPILib SysId characterization. */
  public enum Mechanism {
    DRIVE_LINEAR("Drivetrain Linear (Drive Motors)"),
    DRIVE_ANGULAR("Drivetrain Angular (Rotation / MOI)"),
    DRIVE_STEER("Drivetrain Steer (Turning Motors)"),
    SHOOTER_FLYWHEELS("Shooter Flywheels (Front & Back)"),
    SHOOTER_FEEDER("Shooter Feeder (Kraken X60)"),
    INTAKE_DEPLOY("Intake Deploy Arm (SparkFlex)"),
    INTAKE_ROLLER("Intake Roller (Kraken X60)"),
    INTAKE_CONVEYOR("Intake Conveyor (SparkMax)"),
    DISABLED("Disabled (Ignore SysId Buttons)");

    public final String displayName;

    Mechanism(String displayName) {
      this.displayName = displayName;
    }
  }

  /** Individual test types in a 4-part WPILib SysId routine. */
  public enum TestType {
    QUASISTATIC_FORWARD,
    QUASISTATIC_REVERSE,
    DYNAMIC_FORWARD,
    DYNAMIC_REVERSE
  }

  private final DriveSubsystem m_drive;
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final Set<Subsystem> m_allRequirements;
  private final SendableChooser<Mechanism> m_mechanismChooser = new SendableChooser<>();

  public SysIdSupport(DriveSubsystem drive, Shooter shooter, Intake intake) {
    m_drive = drive;
    m_shooter = shooter;
    m_intake = intake;
    m_allRequirements = Set.of(m_drive, m_shooter, m_intake);

    configureDashboard();
  }

  private void configureDashboard() {
    m_mechanismChooser.setDefaultOption(Mechanism.DRIVE_LINEAR.displayName, Mechanism.DRIVE_LINEAR);
    for (Mechanism mechanism : Mechanism.values()) {
      if (mechanism != Mechanism.DRIVE_LINEAR) {
        m_mechanismChooser.addOption(mechanism.displayName, mechanism);
      }
    }
    SmartDashboard.putData(MECHANISM_CHOOSER_KEY, m_mechanismChooser);

    if (Double.isNaN(SmartDashboard.getNumber(AUTO_MAX_DRIVE_DISTANCE_KEY, Double.NaN))) {
      SmartDashboard.putNumber(AUTO_MAX_DRIVE_DISTANCE_KEY, DEFAULT_AUTO_MAX_DRIVE_DISTANCE_METERS);
    }
    SmartDashboard.putString(
        THRUSTMASTER_MAPPING_KEY,
        "Hold: 11=Quasi Fwd, 13=Quasi Rev, 15=Dyn Fwd, 16=Dyn Rev");

    SmartDashboard.putData(
        "SysId/Quasistatic Forward", createDeferredTestCommand(TestType.QUASISTATIC_FORWARD));
    SmartDashboard.putData(
        "SysId/Quasistatic Reverse", createDeferredTestCommand(TestType.QUASISTATIC_REVERSE));
    SmartDashboard.putData(
        "SysId/Dynamic Forward", createDeferredTestCommand(TestType.DYNAMIC_FORWARD));
    SmartDashboard.putData(
        "SysId/Dynamic Reverse", createDeferredTestCommand(TestType.DYNAMIC_REVERSE));
  }

  /** Registers SysId autonomous options onto the robot's {@code Auto Chooser}. */
  public static void registerAutoChooserOptions(SendableChooser<String> chooser) {
    addSysIdOption(
        chooser, "SysId: Selected Mechanism (All 4 Tests)", autoKeySelectedFull());
    addSysIdOption(
        chooser,
        "SysId: Selected - Quasistatic Forward",
        autoKeySelectedTest(TestType.QUASISTATIC_FORWARD));
    addSysIdOption(
        chooser,
        "SysId: Selected - Quasistatic Reverse",
        autoKeySelectedTest(TestType.QUASISTATIC_REVERSE));
    addSysIdOption(
        chooser,
        "SysId: Selected - Dynamic Forward",
        autoKeySelectedTest(TestType.DYNAMIC_FORWARD));
    addSysIdOption(
        chooser,
        "SysId: Selected - Dynamic Reverse",
        autoKeySelectedTest(TestType.DYNAMIC_REVERSE));

    for (Mechanism mechanism : Mechanism.values()) {
      if (mechanism != Mechanism.DISABLED) {
        addSysIdOption(
            chooser,
            "SysId: " + mechanism.displayName + " [All 4 Tests]",
            autoKeyMechanismFull(mechanism));
      }
    }
  }

  private static void addSysIdOption(
      SendableChooser<String> chooser, String label, String sysIdKey) {
    chooser.addOption(label, sysIdKey);
  }

  public static String autoKeySelectedFull() {
    return SYSID_AUTO_PREFIX + "SELECTED:FULL";
  }

  public static String autoKeySelectedTest(TestType testType) {
    return SYSID_AUTO_PREFIX + "SELECTED:" + testType.name();
  }

  public static String autoKeyMechanismFull(Mechanism mechanism) {
    return SYSID_AUTO_PREFIX + mechanism.name() + ":FULL";
  }

  /** Returns true if an {@code Auto Chooser} selection key denotes a SysId routine. */
  public static boolean isSysIdAutoKey(String selectedKey) {
    return selectedKey != null && selectedKey.startsWith(SYSID_AUTO_PREFIX);
  }

  /** Currently selected mechanism in the {@code SysId Mechanism} dashboard chooser. */
  public Mechanism getSelectedMechanism() {
    Mechanism selected = m_mechanismChooser.getSelected();
    return selected != null ? selected : Mechanism.DRIVE_LINEAR;
  }

  /** Maximum travel distance per leg when running {@code DRIVE_LINEAR} in a full auto sequence. */
  public static double getAutoMaxDriveDistanceMeters() {
    return MathUtil.clamp(
        SmartDashboard.getNumber(
            AUTO_MAX_DRIVE_DISTANCE_KEY, DEFAULT_AUTO_MAX_DRIVE_DISTANCE_METERS),
        MIN_AUTO_MAX_DRIVE_DISTANCE_METERS,
        MAX_AUTO_MAX_DRIVE_DISTANCE_METERS);
  }

  /**
   * Creates a deferred command that resolves the currently selected {@link Mechanism} when
   * scheduled. Suitable for {@code Trigger.whileTrue(...)} and SmartDashboard command buttons.
   */
  public Command createDeferredTestCommand(TestType testType) {
    return Commands.defer(
        () -> createSingleTestCommand(getSelectedMechanism(), testType),
        m_allRequirements);
  }

  /** Builds the autonomous command corresponding to a {@code SYSID:...} auto chooser key. */
  public Command createAutoCommand(String autoKey) {
    if (!isSysIdAutoKey(autoKey)) {
      return Commands.none();
    }
    String payload = autoKey.substring(SYSID_AUTO_PREFIX.length());
    String[] parts = payload.split(":", 2);
    if (parts.length != 2) {
      DriverStation.reportWarning("Unrecognized SysId auto key: " + autoKey, false);
      return Commands.none();
    }

    Mechanism targetMechanism;
    if ("SELECTED".equals(parts[0])) {
      targetMechanism = getSelectedMechanism();
    } else {
      try {
        targetMechanism = Mechanism.valueOf(parts[0]);
      } catch (IllegalArgumentException ex) {
        DriverStation.reportWarning("Unknown SysId mechanism in auto key: " + autoKey, false);
        return Commands.none();
      }
    }

    if (targetMechanism == Mechanism.DISABLED) {
      DriverStation.reportWarning(
          "SysId autonomous routine requested while 'SysId Mechanism' is set to Disabled.",
          false);
      return Commands.none();
    }

    if ("FULL".equals(parts[1])) {
      return createFullFourTestRoutine(targetMechanism);
    }

    try {
      TestType testType = TestType.valueOf(parts[1]);
      return createSingleTestCommand(targetMechanism, testType);
    } catch (IllegalArgumentException ex) {
      DriverStation.reportWarning("Unknown SysId test type in auto key: " + autoKey, false);
      return Commands.none();
    }
  }

  /** Creates a single fresh SysId test command for {@code mechanism} and {@code testType}. */
  public Command createSingleTestCommand(Mechanism mechanism, TestType testType) {
    if (mechanism == null || mechanism == Mechanism.DISABLED) {
      return Commands.runOnce(
          () ->
              DriverStation.reportWarning(
                  "SysId test ignored because 'SysId Mechanism' is set to Disabled.", false));
    }

    SysIdRoutine.Direction direction =
        (testType == TestType.QUASISTATIC_FORWARD || testType == TestType.DYNAMIC_FORWARD)
            ? SysIdRoutine.Direction.kForward
            : SysIdRoutine.Direction.kReverse;
    boolean quasistatic =
        (testType == TestType.QUASISTATIC_FORWARD || testType == TestType.QUASISTATIC_REVERSE);

    Command prepareOtherSubsystems = prepareIdleSubsystemsCommand(mechanism);
    Command routineCommand = buildRawRoutineCommand(mechanism, direction, quasistatic);
    return prepareOtherSubsystems.andThen(routineCommand);
  }

  /**
   * Builds a complete 4-test autonomous characterization sequence (Quasistatic Forward ->
   * Quasistatic Reverse -> Dynamic Forward -> Dynamic Reverse), waiting between legs for the
   * mechanism to come to rest and enforcing per-leg floor-distance bounds on linear drive tests.
   */
  public Command createFullFourTestRoutine(Mechanism mechanism) {
    if (mechanism == null || mechanism == Mechanism.DISABLED) {
      return Commands.none();
    }

    Command prePosition = Commands.none();
    if (mechanism == Mechanism.INTAKE_DEPLOY) {
      // Ensure Leg 1 (Quasistatic Forward) starts near the stowed end of travel so all four legs
      // sweep the full safe arc without immediately tripping the forward limit.
      prePosition =
          Commands.either(
              Commands.runOnce(m_intake::stow, m_intake)
                  .andThen(
                      Commands.waitUntil(() -> m_intake.getDeployPositionRotations() <= 8.0)
                          .withTimeout(2.5))
                  .andThen(Commands.runOnce(() -> m_intake.setDeployMotorSpeed(0.0), m_intake))
                  .andThen(Commands.waitSeconds(SETTLE_PAUSE_SECONDS)),
              Commands.none(),
              () -> m_intake.getDeployPositionRotations() > 10.0);
    }

    return Commands.sequence(
            prepareIdleSubsystemsCommand(mechanism),
            prePosition,
            buildAutoLegCommand(mechanism, TestType.QUASISTATIC_FORWARD),
            buildSettleBetweenTestsCommand(mechanism),
            buildAutoLegCommand(mechanism, TestType.QUASISTATIC_REVERSE),
            buildSettleBetweenTestsCommand(mechanism),
            buildAutoLegCommand(mechanism, TestType.DYNAMIC_FORWARD),
            buildSettleBetweenTestsCommand(mechanism),
            buildAutoLegCommand(mechanism, TestType.DYNAMIC_REVERSE),
            buildFinalStopCommand())
        .withName("SysIdFullRoutine-" + mechanism.name());
  }

  private Command buildAutoLegCommand(Mechanism mechanism, TestType testType) {
    Command leg = createSingleTestCommand(mechanism, testType);
    if (mechanism != Mechanism.DRIVE_LINEAR) {
      return leg;
    }
    // Bound linear translation travel per leg during autonomous sequences so the robot stays
    // safely within available field/practice floor space and returns on the reverse leg.
    double[] legStartMeters = {0.0};
    return Commands.runOnce(() -> legStartMeters[0] = m_drive.getAverageDrivePositionMeters())
        .andThen(
            leg.until(
                () ->
                    Math.abs(m_drive.getAverageDrivePositionMeters() - legStartMeters[0])
                        >= getAutoMaxDriveDistanceMeters()));
  }

  private Command buildSettleBetweenTestsCommand(Mechanism mechanism) {
    switch (mechanism) {
      case DRIVE_LINEAR:
      case DRIVE_ANGULAR:
      case DRIVE_STEER:
        return Commands.runOnce(m_drive::stopAll, m_drive)
            .andThen(Commands.waitUntil(m_drive::isStopped).withTimeout(2.0))
            .andThen(Commands.waitSeconds(SETTLE_PAUSE_SECONDS));
      case SHOOTER_FLYWHEELS:
        return Commands.runOnce(m_shooter::stopAll, m_shooter)
            .andThen(Commands.waitUntil(m_shooter::areFlywheelsStopped).withTimeout(4.0))
            .andThen(Commands.waitSeconds(SETTLE_PAUSE_SECONDS));
      case SHOOTER_FEEDER:
        return Commands.runOnce(m_shooter::stopAll, m_shooter)
            .andThen(Commands.waitUntil(m_shooter::isFeederStopped).withTimeout(2.0))
            .andThen(Commands.waitSeconds(SETTLE_PAUSE_SECONDS));
      case INTAKE_DEPLOY:
        // Keep deploy motor at 0 V without commanding stow so reverse legs sweep back from the
        // deployed end where the preceding forward leg finished.
        return Commands.runOnce(() -> m_intake.setDeployMotorSpeed(0.0), m_intake)
            .andThen(Commands.waitUntil(m_intake::isDeployArmStopped).withTimeout(2.0))
            .andThen(Commands.waitSeconds(SETTLE_PAUSE_SECONDS));
      case INTAKE_ROLLER:
        return Commands.runOnce(m_intake::stopRoller, m_intake)
            .andThen(Commands.waitUntil(m_intake::isRollerStopped).withTimeout(4.0))
            .andThen(Commands.waitSeconds(SETTLE_PAUSE_SECONDS));
      case INTAKE_CONVEYOR:
        return Commands.runOnce(m_intake::stopConveyor, m_intake)
            .andThen(Commands.waitUntil(m_intake::isConveyorStopped).withTimeout(2.0))
            .andThen(Commands.waitSeconds(SETTLE_PAUSE_SECONDS));
      case DISABLED:
      default:
        return Commands.waitSeconds(SETTLE_PAUSE_SECONDS);
    }
  }

  private Command prepareIdleSubsystemsCommand(Mechanism activeMechanism) {
    switch (activeMechanism) {
      case DRIVE_LINEAR:
      case DRIVE_ANGULAR:
      case DRIVE_STEER:
        return Commands.runOnce(
            () -> {
              m_shooter.stopAll();
              m_intake.stopRoller();
              m_intake.stopConveyor();
            },
            m_drive,
            m_shooter,
            m_intake);
      case SHOOTER_FLYWHEELS:
      case SHOOTER_FEEDER:
        return Commands.runOnce(
            () -> {
              m_drive.stopAll();
              m_intake.stopRoller();
              m_intake.stopConveyor();
            },
            m_drive,
            m_shooter,
            m_intake);
      case INTAKE_DEPLOY:
      case INTAKE_ROLLER:
      case INTAKE_CONVEYOR:
        return Commands.runOnce(
            () -> {
              m_drive.stopAll();
              m_shooter.stopAll();
            },
            m_drive,
            m_shooter,
            m_intake);
      case DISABLED:
      default:
        return Commands.none();
    }
  }

  private Command buildFinalStopCommand() {
    return Commands.runOnce(
        () -> {
          m_drive.stopAll();
          m_shooter.stopAll();
          m_intake.setDeployMotorSpeed(0.0);
          m_intake.stopRoller();
          m_intake.stopConveyor();
        },
        m_drive,
        m_shooter,
        m_intake);
  }

  private Command buildRawRoutineCommand(
      Mechanism mechanism, SysIdRoutine.Direction direction, boolean quasistatic) {
    switch (mechanism) {
      case DRIVE_LINEAR:
        return quasistatic
            ? m_drive.sysIdLinearQuasistatic(direction)
            : m_drive.sysIdLinearDynamic(direction);
      case DRIVE_ANGULAR:
        return quasistatic
            ? m_drive.sysIdAngularQuasistatic(direction)
            : m_drive.sysIdAngularDynamic(direction);
      case DRIVE_STEER:
        return quasistatic
            ? m_drive.sysIdSteerQuasistatic(direction)
            : m_drive.sysIdSteerDynamic(direction);
      case SHOOTER_FLYWHEELS:
        return quasistatic
            ? m_shooter.sysIdFlywheelsQuasistatic(direction)
            : m_shooter.sysIdFlywheelsDynamic(direction);
      case SHOOTER_FEEDER:
        return quasistatic
            ? m_shooter.sysIdFeederQuasistatic(direction)
            : m_shooter.sysIdFeederDynamic(direction);
      case INTAKE_DEPLOY:
        return quasistatic
            ? m_intake.sysIdDeployQuasistatic(direction)
            : m_intake.sysIdDeployDynamic(direction);
      case INTAKE_ROLLER:
        return quasistatic
            ? m_intake.sysIdRollerQuasistatic(direction)
            : m_intake.sysIdRollerDynamic(direction);
      case INTAKE_CONVEYOR:
        return quasistatic
            ? m_intake.sysIdConveyorQuasistatic(direction)
            : m_intake.sysIdConveyorDynamic(direction);
      case DISABLED:
      default:
        return Commands.none();
    }
  }
}
