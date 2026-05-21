# VikingRobot2026

FRC Team 2928's competition robot code. Java, WPILib 2026, Gradle build.

## Build & Deploy

```bash
./gradlew build          # compile + check
./gradlew deploy         # deploy to roboRIO
./gradlew simulateJava   # run in sim
```

## Architecture: Goal-Based Superstructure

This codebase uses a centralized Superstructure that produces a **RobotGoal** each cycle and passes it to every subsystem. The design is inspired by 254/2910-style state machines but uses a composable goal record rather than a monolithic state enum.

### Core Principles

1. **Single authority.** The Superstructure is the only thing that tells subsystems what to do during a match. OI classes set intents. Auto routines set intents. Nothing ever calls `subsystem.setWantedState()` directly except the Superstructure's `applyGoal()` path.

2. **Goals are total.** Every cycle, every subsystem receives an explicit instruction. There is no "hold your current state" sentinel. If a goal doesn't mention a subsystem, that subsystem gets its safe default (see below). This prevents stale behavior from accumulating across state transitions.

3. **Safe defaults per subsystem.** Each subsystem has a default goal that represents "do nothing dangerous":
   - Drive → `TELEOP` (driver has control)
   - Shooter → `HOME` (hood down, flywheels off)
   - Intake → `STOP` (roller off, no motion)
   - Indexer → `STOP`
   - Hopper → `STOP`
   - Climber → `IDLE`

   The `RobotGoal.builder()` initializes all fields to these defaults. You only specify what needs to be *active*.

4. **Subsystems own their own safety.** Each subsystem receives a `SuperstructureContext` (the desired goal + read-only state of all other subsystems) and decides for itself whether it can proceed. If a precondition isn't met (e.g., climber can't deploy while intake is extended), the subsystem holds at a safe state until the condition clears.

5. **Goals encode coordination by design.** If subsystem A needs subsystem B in a certain state, the goal must explicitly command B toward that state. For example, `RobotGoal.climb()` includes `intake=RETRACT` so the intake actively moves out of the way while the climber waits for it.

6. **Subsystems handle the "how."** The Superstructure says *what* (e.g., "shoot at hub"). The subsystem figures out *how* (spin up flywheels → wait for speed → aim hood → fire kicker). Internal sequencing lives inside the subsystem's `handleStateTransition()`.

### Data Flow (Every 20ms)

```
OI → IntentStore / OverrideManager
                ↓
        GoalResolver (reads intents + DriverStation state)
                ↓
            RobotGoal (built via builder, safe defaults)
                ↓
    SuperstructureContext (goal + all subsystem state snapshots)
                ↓
    Each subsystem.applyGoal(context)
        → checks interlocks against context
        → sets internal WantedState
        → handleStateTransition()
        → applyState() → hardware
```

### RobotGoal Builder Pattern

Goals are built with a builder. Defaults are safe. Only specify active subsystems:

```java
// Everything unmentioned defaults to safe idle
public static RobotGoal intakeDrive() {
    return builder()
        .withDrive(DriveGoal.FACE_TRAVEL_DIRECTION)
        .withIntake(IntakeGoal.EXTEND_AND_RUN)
        .build();
}

// Use .modify() to derive from an existing goal
RobotGoal autoShoot = RobotGoal.shootAtHub()
    .modify()
    .withDrive(DriveGoal.AUTONOMOUS)
    .build();
```

### Interlocks

Interlocks are pairwise guard clauses inside each subsystem's `applyGoal()` method. They check the `SuperstructureContext` for physical state of other subsystems and refuse to proceed if unsafe:

```java
// In ClimberSubsystem.applyGoal():
if (desired == ClimberGoal.DEPLOY && ctx.intakeState().isExtended()) {
    wantedState = WantedState.IDLE;  // wait for intake to clear
    return;
}
```

Keep interlocks minimal. If you're adding more than 2-3 per subsystem, the goals themselves should be redesigned to avoid the conflict.

### Subsystem Internal Pattern

Every subsystem follows the same structure:

```java
public class FooSubsystem extends SubsystemBase {
    enum WantedState { ... }   // what the Superstructure asked for
    enum SystemState { ... }   // what we're actually doing (may differ during transitions)

    void applyGoal(SuperstructureContext ctx) { /* interlocks + set wantedState */ }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Foo", inputs);
        systemState = handleStateTransition();
        applyState();
    }

    SystemState handleStateTransition() { /* WantedState + sensor state → SystemState */ }
    void applyState() { /* SystemState → hardware outputs via IO */ }

    FooState getState() { /* returns read-only snapshot for SuperstructureContext */ }
}
```

### IO Abstraction

Each subsystem uses an IO interface + real implementation (AdvantageKit pattern):
- `FooIO.java` - interface with default no-op methods + `@AutoLog` inputs class
- `FooIOReal.java` - hardware implementation (TalonFX, CANcoder, etc.)

### Auto Routines

Auto routines are WPILib command sequences that change the active goal through the Superstructure's intent/override system. They never command subsystems directly:

```java
Commands.sequence(
    superstructure.setGoalCommand(RobotGoal.intakeDrive()),
    pathCommand("pickupPath"),
    superstructure.setGoalCommand(RobotGoal.shootAtHub()),
    waitUntil(shooter::isShotComplete),
    superstructure.setGoalCommand(RobotGoal.freeDrive())
)
```

### OI Rules

- DriverOI and OperatorOI set intents and push/pop overrides on the Superstructure.
- They never reference subsystems directly except for read-only queries (e.g., LED state display).
- Exception: operator nudges (tuning hood angle, flywheel speed) may go to the shooter directly since they modify calibration, not robot state.

## Documentation Maintenance

When implementing architecture components (new subsystems, goal factories, interlocks, etc.), also create or update the corresponding architecture doc in `docs/architecture/`. These serve as student reference material and give future agents fast context.

Expected docs (create as the corresponding code is implemented):
- `docs/architecture/superstructure.md` - Goal-based system overview, data flow
- `docs/architecture/subsystem-template.md` - How subsystems are built (WantedState/SystemState pattern)
- `docs/architecture/interlocks.md` - Pairwise safety constraints and how to add new ones
- `docs/architecture/auto-routines.md` - How auto works with goals
- `docs/architecture/vision.md` - Limelight integration and pose estimation pipeline

Rules:
- Only update docs based on code actually written in the session (not speculative)
- Propose doc changes to the user and wait for approval before writing
- Keep each doc concise - a student should be able to read one in under 5 minutes
- Include mermaid diagrams where they aid understanding
- Reference specific source files/classes so docs stay grounded in the code

## Conventions

- Logging: AdvantageKit (`Logger.processInputs`, `Logger.recordOutput`)
- Vision: Limelight MegaTag2 for pose estimation, fused into drivetrain Kalman filter
- Paths: BLine custom path follower (primary) + Choreo (available)
- Motor control: CTRE Phoenix 6 (TalonFX)
- Swerve: CTRE TunerSwerveDrivetrain base class
