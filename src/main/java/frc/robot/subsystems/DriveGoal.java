package frc.robot.subsystems;

//TODO: make this work with 2026
/**
 * High-level intent for the drive subsystem, set by the Superstructure (future)
 * or directly by commands (current, pre-Superstructure).
 *
 * Safe default: TELEOP (driver joystick has control).
 */
public enum DriveGoal {
    /** Driver joystick controls the robot. Default state during teleop. */
    TELEOP,

    /** Wheels locked in X formation. Used for defense and end-of-match. */
    LOCK,

    /** Path follower (PathPlanner/BLine) controls the robot. Used during auto. */
    AUTONOMOUS,

    /** Joystick translation + limelight-based rotational correction for speaker shots. */
    AIM_SPEAKER,

    /** Joystick translation + limelight-based additive correction toward the note on the ground. Driver retains full control. */
    TRACK_NOTE,

    /** Joystick translation with automatic rotation to face the direction of travel. Used during intake. */
    INTAKE_DRIVE
}
