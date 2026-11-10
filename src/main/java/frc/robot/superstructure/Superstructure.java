package frc.robot.superstructure;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

/**
 * Single authority for all subsystem goals during a match.
 *
 * <p>Every 20ms:
 * <ol>
 *   <li>All registered status signals are refreshed in a single CAN bus transaction</li>
 *   <li>{@link GoalResolver} reads stored intents + DriverStation state → {@link RobotGoal}</li>
 *   <li>A {@link SuperstructureContext} is built from the goal + subsystem state snapshots</li>
 *   <li>Each subsystem receives {@code applyGoal(ctx)} — the only place subsystems are told what
 *       to do</li>
 * </ol>
 *
 * <p>OI triggers set intents directly on the {@link #resolver}. Auto routines use
 * {@link #setGoalCommand}. Neither OI nor auto routines ever call subsystems directly.
 */
public class Superstructure extends SubsystemBase {

    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    public final GoalResolver resolver = new GoalResolver();

    // Per-subsystem signal registration. Preserves insertion order for diagnostics.
    private final Map<Subsystem, List<BaseStatusSignal>> mSubsystemSignalsMap = new LinkedHashMap<>();

    // Flattened cache of all signals for the batch refresh call. Rebuilt on registration.
    private BaseStatusSignal[] mAllSignals = new BaseStatusSignal[0];

    public Superstructure() {}

    public void setSubsystems(CommandSwerveDrivetrain drivetrain, Shooter shooter, Climber climber) {
        this.drivetrain = drivetrain;
        this.shooter    = shooter;
    }

    /**
     * Register status signals for a subsystem. Call during subsystem construction.
     * Signals will be batch-refreshed each cycle before goal resolution.
     */
    public void registerSignals(Subsystem subsystem, BaseStatusSignal... signals) {
        List<BaseStatusSignal> list = mSubsystemSignalsMap.computeIfAbsent(subsystem, k -> new ArrayList<>());
        for (BaseStatusSignal signal : signals) {
            list.add(signal);
        }
        rebuildSignalCache();
    }

    private void rebuildSignalCache() {
        mAllSignals = mSubsystemSignalsMap.values().stream()
            .flatMap(List::stream)
            .toArray(BaseStatusSignal[]::new);
    }

    // -------------------------------------------------------------------------
    // Periodic loop
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Batch-refresh all registered CAN signals in a single transaction
        if (mAllSignals.length > 0) {
            BaseStatusSignal.refreshAll(mAllSignals);
        }

        RobotGoal goal = resolver.resolve();
        SuperstructureContext ctx = new SuperstructureContext(goal, shooter.getState(), drivetrain.getStateSnapshot());
        drivetrain.applyGoal(ctx);
        shooter.applyGoal(ctx);
    }

    // -------------------------------------------------------------------------
    // Auto routine commands
    // -------------------------------------------------------------------------

    /**
     * Returns a command that overrides the resolver with the given goal for one cycle (runOnce).
     * The resolver's stored intents are updated to match, so the goal persists until OI or the
     * next setGoalCommand changes them.
     *
     * <p>Usage:
     * <pre>
     * Commands.sequence(
     *     superstructure.setGoalCommand(RobotGoal.autonomous()),
     *     pathCommand("pickupPath"),
     *     superstructure.setGoalCommand(RobotGoal.freeDrive())
     * )
     * </pre>
     */
    public Command setGoalCommand(RobotGoal goal) {
        return Commands.runOnce(() -> resolver.setGoal(goal));
    }
}
