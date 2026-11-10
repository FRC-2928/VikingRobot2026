package frc.robot.subsystems.shooter;

/**
 * What the Superstructure is asking the shooter/intake subsystem to do this cycle.
 *
 * <p>Safe default is {@link #HOME}. All other goals represent active behavior.
 * Intake is part of this subsystem — there is no separate IntakeGoal.
 */
public enum ShooterGoal {
    SHOOT,
	HALT,
	AIM_HUB_AUTO,
	SHOOT_OVERRIDE,
	SHOOT_HOME,
	HOME
}
