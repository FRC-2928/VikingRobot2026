package frc.robot.superstructure;

import frc.robot.subsystems.CommandSwerveDrivetrain.DrivetrainState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterGoal;
import frc.robot.subsystems.HopperFloor.HopperFloorState;
/**
 * Immutable snapshot passed to every subsystem's {@code applyGoal()} each cycle.
 *
 * <p>Contains the current {@link RobotGoal} plus read-only state snapshots of all subsystems.
 * Subsystems use this to check interlocks against other subsystems without direct references.
 *
 * <p>Phase 3: {@code shooterState} added for Phase 4 climber interlock
 * ({@code ctx.shooterState().atSafeAngle()}).
 */
public record SuperstructureContext(RobotGoal goal, ShooterState shooterState, DrivetrainState drivetrainState, HopperFloorState hopperFloorState) {

    /** Convenience constructor for cycles where no subsystem state is needed yet. */
    public SuperstructureContext(RobotGoal goal) {
        this(goal, new ShooterState(), new DrivetrainState(false, null, null), new HopperFloorState(false, false)); //TODO: Make this correct
    }
}
