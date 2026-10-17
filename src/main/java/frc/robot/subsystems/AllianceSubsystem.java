package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Owns the alliance the robot is competing on.
 *
 * <p>On the real robot the alliance is reported by the FMS and cannot be
 * changed. In simulation the alliance can be changed at any time, either from
 * the {@code Alliance} chooser on the dashboard or with the alliance-toggle
 * button on the driver controller.
 *
 * <p>Whenever the alliance changes the simulated driver station is updated as
 * well, so {@link DriverStation#getAlliance()} (and therefore PathPlanner's
 * path mirroring) always agrees with the alliance the robot code is using.
 */
public class AllianceSubsystem extends SubsystemBase {
  /** Dashboard key of the alliance chooser (simulation only). */
  public static final String kChooserKey = "Alliance";

  /** Dashboard key of the read-only alliance status. */
  public static final String kStatusKey = "Alliance Status";

  /** Dashboard key of the "move the robot to the other side" toggle. */
  public static final String kMirrorKey = "Mirror Robot On Alliance Change";

  private final boolean m_isSim = RobotBase.isSimulation();
  private final List<Consumer<DriverStation.Alliance>> m_listeners = new ArrayList<>();

  /** The alliance the robot is currently competing on. */
  private DriverStation.Alliance m_alliance;

  /** True once the alliance has been settled on (used to suppress the first event). */
  private boolean m_established = false;

  /** True once the user has taken the alliance over from the dashboard. */
  private boolean m_userOverride = false;

  /** Alliance requested by the dashboard chooser (written from the NT thread). */
  private volatile DriverStation.Alliance m_requestedAlliance = null;

  /** Dashboard chooser, only created in simulation. */
  private SendableChooser<DriverStation.Alliance> m_chooser = null;

  public AllianceSubsystem() {
    Optional<DriverStation.Alliance> dsAlliance = DriverStation.getAlliance();
    m_alliance = dsAlliance.orElse(DriverStation.Alliance.Blue);
    // If the driver station already knows the alliance we are settled; if it
    // does not (it attaches a moment after the program starts) the first value
    // we hear is treated as the starting alliance rather than as a change.
    m_established = dsAlliance.isPresent();

    if (m_isSim) {
      m_chooser = new SendableChooser<>();
      // Default to whatever the simulated driver station reports so the
      // dashboard matches the simulation from the very first frame.
      m_chooser.setDefaultOption(m_alliance.name(), m_alliance);
      m_chooser.addOption(DriverStation.Alliance.Blue.name(), DriverStation.Alliance.Blue);
      m_chooser.addOption(DriverStation.Alliance.Red.name(), DriverStation.Alliance.Red);
      // This listener runs on the NetworkTables thread, so it only records the
      // request; it is applied on the robot thread in periodic().
      m_chooser.onChange(alliance -> m_requestedAlliance = alliance);
      SmartDashboard.putData(kChooserKey, m_chooser);

      SmartDashboard.putBoolean(kMirrorKey, true);
    }
  }

  /**
   * Returns the alliance the robot is currently competing on.
   *
   * @return the current alliance
   */
  public DriverStation.Alliance getAlliance() {
    return m_alliance;
  }

  /**
   * Returns whether the robot is on the red alliance.
   *
   * @return true when on the red alliance
   */
  public boolean isRed() {
    return m_alliance == DriverStation.Alliance.Red;
  }

  /**
   * Registers a listener that is called whenever the alliance changes. The
   * listener is not called for the alliance the robot starts with.
   *
   * @param listener callback receiving the new alliance
   */
  public void addAllianceChangedListener(Consumer<DriverStation.Alliance> listener) {
    m_listeners.add(listener);
  }

  /**
   * Changes the alliance. Only possible in simulation - on the real robot the
   * alliance comes from the FMS.
   *
   * @param alliance the new alliance
   */
  public void setAlliance(DriverStation.Alliance alliance) {
    if (!m_isSim) {
      DriverStation.reportWarning("The alliance can only be changed in simulation", false);
      return;
    }

    if (alliance == m_alliance) {
      return;
    }

    m_userOverride = true;
    m_established = true;
    m_alliance = alliance;

    pushToDriverStation(alliance);
    notifyListeners();
  }

  /**
   * Switches between the red and blue alliances.
   */
  public void toggleAlliance() {
    setAlliance(isRed() ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red);
  }

  /**
   * Returns whether the robot should be moved to the other side of the field
   * when the alliance changes. Only meaningful in simulation.
   *
   * @return true to mirror the robot pose on an alliance change
   */
  public boolean shouldMirrorRobot() {
    return !m_isSim || SmartDashboard.getBoolean(kMirrorKey, true);
  }

  /** Keeps the simulated driver station in sync with the robot's alliance. */
  private void pushToDriverStation(DriverStation.Alliance alliance) {
    DriverStationSim.setAllianceStationId(
        alliance == DriverStation.Alliance.Red
            ? AllianceStationID.Red1
            : AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  private void notifyListeners() {
    for (Consumer<DriverStation.Alliance> listener : m_listeners) {
      listener.accept(m_alliance);
    }
  }

  @Override
  public void periodic() {
    // 1. Apply a request made on the dashboard. The chooser callback runs on
    //    the NetworkTables thread, so the request is only picked up here. The
    //    request is not cleared: once it has been applied it equals the
    //    current alliance and is ignored, so no update can get lost.
    DriverStation.Alliance requested = m_requestedAlliance;
    if (requested != null) {
      if (requested != m_alliance) {
        setAlliance(requested);
      } else {
        // Picking the alliance that is already active still counts as the
        // driver taking over from the driver station.
        m_userOverride = true;
      }
    }

    // 2. Until the driver takes over from the dashboard, follow the driver
    //    station: the FMS on the real robot, the simulated driver station in
    //    simulation.
    if (!m_userOverride) {
      DriverStation.getAlliance().ifPresent(alliance -> {
        if (alliance != m_alliance) {
          m_alliance = alliance;
          if (m_established) {
            notifyListeners();
          }
          m_established = true;
          // Keep the dashboard chooser showing the alliance we are on.
          if (m_chooser != null) {
            m_chooser.setDefaultOption(alliance.name(), alliance);
          }
        }
      });
    }

    SmartDashboard.putString(kStatusKey, m_alliance.name());
  }
}
