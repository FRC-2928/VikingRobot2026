package frc.robot.superstructure;

import frc.robot.subsystems.DriveGoal;
import frc.robot.subsystems.shooter.ShooterGoal;

/**
 * Immutable snapshot of what every subsystem should be doing this cycle.
 *
 * <p>Build via {@link #builder()} — all fields default to their subsystem-safe defaults.
 * Only specify what needs to be active:
 *
 * <pre>
 * RobotGoal goal = RobotGoal.builder()
 *     .withDrive(DriveGoal.LOCK)
 *     .build();
 * </pre>
 *
 * <p>Derive from an existing goal via {@link #modify()}:
 *
 * <pre>
 * RobotGoal autoShoot = RobotGoal.shootSpeaker()
 *     .modify()
 *     .withDrive(DriveGoal.AUTONOMOUS)
 *     .build();
 * </pre>
 */
public record RobotGoal(DriveGoal drive, ShooterGoal shooter) {

    // -------------------------------------------------------------------------
    // Factory methods
    // -------------------------------------------------------------------------

    /** Driver has full joystick control. All subsystems at safe idle. */
    public static RobotGoal freeDrive() {;
        return builder().build();
    }

    /** Wheels locked in X-formation. */
    public static RobotGoal lockWheels() {
        return builder().withDrive(DriveGoal.LOCK).build();
    }

    /** Path follower controls the drive. Used during auto routines. */
    public static RobotGoal autonomous() {
        return builder().withDrive(DriveGoal.AUTONOMOUS).build();
    }

	// Look into Viking2024 arc-sandbox branch for uses

    // -------------------------------------------------------------------------
    // Builder
    // -------------------------------------------------------------------------

    /** Returns a new {@link Builder} with all fields set to safe defaults. */
    public static Builder builder() {
        return new Builder();
    }

    /** Returns a {@link Builder} pre-populated from this goal for easy derivation. */
    public Builder modify() {
        return new Builder(this);
    }

    public static class Builder {
        // Safe defaults
        private DriveGoal   drive   = DriveGoal.TELEOP;
        private ShooterGoal shooter = ShooterGoal.HOME;

        private Builder() {}

        private Builder(RobotGoal base) {
            this.drive   = base.drive;
            this.shooter = base.shooter;
        }

        public Builder withDrive(DriveGoal drive) {
            this.drive = drive;
            return this;
        }

        public Builder withShooter(ShooterGoal shooter) {
            this.shooter = shooter;
            return this;
        }

        public RobotGoal build() {
            return new RobotGoal(drive, shooter);
        }
    }
}
