/* package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class AutoClimb extends Command {

    private final Climber climber;

    // Climb sequence has two phases:
    // Phase 1 — extend UP to the bar
    // Phase 2 — retract DOWN to lift the robot off the floor
    private enum ClimbPhase { EXTENDING, RETRACTING, DONE }
    private ClimbPhase phase;

    // How close to the soft limit counts as "at the top"
    private static final double TOP_THRESHOLD    = 55.0; // close enough to TOP_LIMIT (57.426)
    private static final double BOTTOM_THRESHOLD =  2.0; // close enough to BOTTOM_LIMIT (0.0)

    public AutoClimb() {
        this.climber = Climber.getInstance();
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        phase = ClimbPhase.EXTENDING;
        climber.extend(); // Start going UP immediately
    }

    @Override
    public void execute() {
        double pos = climber.getPosition();

        if (phase == ClimbPhase.EXTENDING) {
            // Keep extending until we reach the top
            if (pos >= TOP_THRESHOLD) {
                climber.stop();
                phase = ClimbPhase.RETRACTING;
                climber.retract(); // Start pulling DOWN to lift robot
            }
        } else if (phase == ClimbPhase.RETRACTING) {
            // Keep retracting until robot is lifted (near bottom)
            if (pos <= BOTTOM_THRESHOLD) {
                climber.stop();
                phase = ClimbPhase.DONE;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return phase == ClimbPhase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop(); // Always stop motor when command ends
    }
} */