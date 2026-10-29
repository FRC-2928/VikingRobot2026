# Refactor Sketch: Goal-Based Superstructure

## Package Layout

```
frc.robot/
├── Robot.java
├── RobotContainer.java                  # Creates subsystems, wires OI
│
├── oi/
│   ├── BaseOI.java
│   ├── DriverOI.java                    # Sets intents on the IntentStore
│   └── OperatorOI.java                  # Direct nudges stay here
│
├── superstructure/
│   ├── Superstructure.java              # The periodic loop: resolve → apply
│   ├── RobotGoal.java                   # Record: what every subsystem should do
│   ├── IntentStore.java                 # Clean intent flag management
│   ├── OverrideManager.java             # Push/pop override goals
│   └── GoalResolver.java               # Transition logic: intents + sensors → goal
│
├── subsystems/
│   ├── drive/
│   │   ├── DriveSubsystem.java          # WantedState/SystemState machine
│   │   ├── DriveIO.java
│   │   └── DriveIOReal.java
│   ├── shooter/
│   │   ├── ShooterSubsystem.java        # WantedState/SystemState machine
│   │   ├── ShooterIO.java
│   │   └── ShooterIOReal.java
│   ├── intake/
│   │   ├── IntakeSubsystem.java         # WantedState/SystemState machine
│   │   ├── IntakeIO.java
│   │   └── IntakeIOReal.java
│   ├── indexer/
│   │   ├── IndexerSubsystem.java        # WantedState/SystemState machine
│   │   ├── IndexerIO.java
│   │   └── IndexerIOReal.java
│   └── hopper/
│       ├── HopperSubsystem.java
│       ├── HopperIO.java
│       └── HopperIOReal.java
│
├── auto/
│   └── Autonomous.java                  # Composes goal changes + path following
│
└── vision/
    └── Limelight.java
```

---

## System Block Diagram

```mermaid
graph TD
    subgraph "INPUTS"
        JS1[Driver Controller]
        JS2[Operator Controller]
        SENS[Sensors<br>Limelights, Beam Breaks, etc.]
    end

    subgraph "OI LAYER"
        DOI[DriverOI]
        OOI[OperatorOI]
    end

    subgraph "SUPERSTRUCTURE (runs every 20ms)"
        IS[IntentStore]
        OM[OverrideManager]
        GR[GoalResolver]
        GOAL["RobotGoal (current)"]
        APPLY["applyGoal()"]
    end

    subgraph "SUBSYSTEMS (each has WantedState → SystemState → applyState)"
        DRV[DriveSubsystem]
        SHT[ShooterSubsystem]
        INT[IntakeSubsystem]
        IDX[IndexerSubsystem]
        HOP[HopperSubsystem]
    end

    subgraph "HARDWARE"
        HW1[Swerve Modules]
        HW2[Flywheel + Hood + Kicker]
        HW3[Intake Rack + Roller]
        HW4[Indexer Belt]
        HW5[Hopper Floor]
    end

    JS1 --> DOI
    JS2 --> OOI
    SENS --> GR

    DOI -->|"set/clear intents"| IS
    DOI -->|"push/pop overrides"| OM
    OOI -->|"nudges (direct)"| SHT

    IS --> GR
    OM --> GR
    GR -->|"produces"| GOAL
    GOAL --> APPLY

    APPLY -->|"drive.setWantedState()"| DRV
    APPLY -->|"shooter.setWantedState()"| SHT
    APPLY -->|"intake.setWantedState()"| INT
    APPLY -->|"indexer.setWantedState()"| IDX
    APPLY -->|"hopper.setWantedState()"| HOP

    DRV --> HW1
    SHT --> HW2
    INT --> HW3
    IDX --> HW4
    HOP --> HW5
```

---

## Data Flow Per Cycle

```mermaid
sequenceDiagram
    participant OI as OI Layer
    participant IS as IntentStore
    participant OM as OverrideManager
    participant GR as GoalResolver
    participant SS as Superstructure.periodic()
    participant SUB as Subsystems

    Note over OI: Between cycles, buttons fire
    OI->>IS: set(INTAKE_DRIVE)
    OI->>OM: pushOverride(SHOOT_HUB) [if trigger held]

    Note over SS: periodic() fires
    SS->>OM: hasActiveOverride()?
    alt Override active
        OM-->>SS: overrideGoal
    else No override
        SS->>GR: resolve(intentStore, sensorState)
        GR-->>SS: computedGoal
    end
    SS->>SS: currentGoal = result
    SS->>SUB: drive.setWantedState(currentGoal.drive)
    SS->>SUB: shooter.setWantedState(currentGoal.shooter)
    SS->>SUB: intake.setWantedState(currentGoal.intake)
    SS->>SUB: indexer.setWantedState(currentGoal.indexer)
    SS->>SUB: hopper.setWantedState(currentGoal.hopper)

    Note over SUB: Each subsystem's periodic() fires
    SUB->>SUB: handleStateTransition()
    SUB->>SUB: applyState() → hardware
```

---

## RobotGoal Structure

```mermaid
classDiagram
    class RobotGoal {
        +DriveGoal drive
        +ShooterGoal shooter
        +IntakeGoal intake
        +IndexerGoal indexer
        +HopperGoal hopper
        +freeDrive()$ RobotGoal
        +shootAtHub()$ RobotGoal
        +shootAtHome()$ RobotGoal
        +intakeDrive()$ RobotGoal
        +intakeGround()$ RobotGoal
        +intakeRetract()$ RobotGoal
        +disabled()$ RobotGoal
    }

    class DriveGoal {
        <<enum>>
        TELEOP
        TARGET_LOCK_HUB
        TARGET_LOCK_HOME
        FACE_TRAVEL_DIRECTION
        AUTONOMOUS
        BRAKE
        IDLE
    }

    class ShooterGoal {
        <<enum>>
        HOME
        AIM_HUB
        AIM_HOME
        SHOOT_HUB
        SHOOT_HOME
        SHOOT_OVERRIDE
    }

    class IntakeGoal {
        <<enum>>
        STOP
        EXTEND
        EXTEND_AND_RUN
        RETRACT
        REVERSE
    }

    class IndexerGoal {
        <<enum>>
        STOP
        FEED
    }

    class HopperGoal {
        <<enum>>
        STOP
        FEED
        REVERSE
    }

    RobotGoal --> DriveGoal
    RobotGoal --> ShooterGoal
    RobotGoal --> IntakeGoal
    RobotGoal --> IndexerGoal
    RobotGoal --> HopperGoal
```

---

## GoalResolver Logic

```mermaid
flowchart TD
    START["resolve(intents, sensors)"] --> OVR{Override active?}
    OVR -->|Yes| RET_OVR[Return override goal]
    OVR -->|No| DIS{Disabled?}
    DIS -->|Yes| RET_DIS["Return RobotGoal.disabled()"]
    DIS -->|No| AUTO{Autonomous?}
    AUTO -->|Yes| RET_AUTO["Return RobotGoal.autonomous()"]
    AUTO -->|No| CHK_INTENTS["Check intents in priority order"]

    CHK_INTENTS --> I1{SHOOT_HUB?}
    I1 -->|Yes| RET_SHOOT["Return RobotGoal.shootAtHub()"]
    I1 -->|No| I2{TARGET_LOCK?}
    I2 -->|Yes| RET_TL["Return RobotGoal.targetLock()"]
    I2 -->|No| I3{INTAKE_DRIVE?}
    I3 -->|Yes| RET_ID["Return RobotGoal.intakeDrive()"]
    I3 -->|No| I4{INTAKE_MANUAL?}
    I4 -->|Yes| RET_IM["Return RobotGoal.intakeManual()"]
    I4 -->|No| I5{INTAKE_RETRACT?}
    I5 -->|Yes| RET_IR["Return RobotGoal.intakeRetract()"]
    I5 -->|No| RET_FREE["Return RobotGoal.freeDrive()"]
```

---

## Override Manager

```mermaid
stateDiagram-v2
    state "No Override Active" as NONE
    state "Override Active<br>(previous goal saved)" as ACTIVE

    [*] --> NONE

    NONE --> ACTIVE : pushOverride(goal)
    ACTIVE --> NONE : popOverride()
    ACTIVE --> ACTIVE : pushOverride rejected<br>(log + discard)
```

---

## How Auto Works

```mermaid
flowchart TD
    subgraph "Auto Routine (WPILib Command Sequence)"
        A1["setGoalCommand(intakeDrive)"]
        A2["runPath('pickupPath')"]
        A3["waitUntil(hasBall)"]
        A4["setGoalCommand(shootAtHub)"]
        A5["waitUntil(shotComplete)"]
        A6["setGoalCommand(freeDrive)"]
    end

    A1 --> A2 --> A3 --> A4 --> A5 --> A6

    subgraph "What setGoalCommand does"
        SGC["Sets an AUTO intent<br>with a specific goal.<br>GoalResolver returns<br>that goal when<br>in autonomous mode."]
    end
```

Auto routines are just command sequences that change which goal the Superstructure is targeting. The Superstructure still applies it every cycle - auto doesn't bypass the system.

---

## Subsystem Internal Pattern (all subsystems follow this)

```mermaid
flowchart TD
    subgraph "ShooterSubsystem.periodic()"
        A["Read inputs from IO"] --> B["handleStateTransition()"]
        B --> C["applyState()"]
        C --> D["Log state + inputs"]
    end

    subgraph "handleStateTransition()"
        E{WantedState?}
        E -->|HOME| F[SystemState.HOMING]
        E -->|AIM_HUB| G[SystemState.AIMING]
        E -->|SHOOT_HUB| H{flywheels at speed?}
        H -->|No| I[SystemState.SPINNING_UP]
        H -->|Yes| J[SystemState.SHOOTING]
    end

    subgraph "applyState()"
        K{SystemState?}
        K -->|HOMING| L["hood → 0, flywheels → 0, kicker → 0"]
        K -->|AIMING| M["hood → lookup, flywheels → lookup, kicker → 0"]
        K -->|SPINNING_UP| N["hood → lookup, flywheels → lookup, kicker → 0"]
        K -->|SHOOTING| O["hood → lookup, flywheels → lookup, kicker → full"]
    end
```

The subsystem adds intelligence that the Superstructure doesn't need to know about. The Superstructure says "shoot at hub" and the shooter figures out the spin-up sequencing internally.

---

## Key Differences From Current 2928 Code

| Aspect | Current | Refactored |
|--------|---------|------------|
| State representation | `RobotState` enum (13 values) | `RobotGoal` record (composable) |
| State execution | Trigger → Command (scheduler-managed) | `applyGoal()` direct calls every cycle |
| Subsystem control | Mixed (some through SS, some direct) | ALL through Superstructure (single writer) |
| Intent storage | Mutable boolean on enum constants | Separate `IntentStore` class |
| Override handling | EnumSet + prevState field | `OverrideManager` with push/pop |
| Shooter coordination | Command composition (`alongWith`) | Subsystem handles internally via states |
| Cleanup on state exit | `finallyDo()` on commands | Natural: next cycle applies new goal |
| Auto integration | Superstructure exposes command factories | Auto sets goals; SS applies them same as teleop |

---

## Migration Path (Suggested Order)

```mermaid
flowchart LR
    S1["1. Create RobotGoal<br>+ IntentStore<br>+ OverrideManager"] --> S2["2. Give every subsystem<br>WantedState/SystemState"]
    S2 --> S3["3. Rewrite Superstructure<br>periodic() as<br>resolve → apply"]
    S3 --> S4["4. Move OI to set<br>intents only"]
    S4 --> S5["5. Convert auto routines<br>to goal-setting commands"]
    S5 --> S6["6. Remove Trigger-based<br>state execution"]
```

Step 2 is the biggest lift (shooter and indexer/hopper don't have state machines today). Steps 3-4 can be done together. Step 6 is cleanup once everything else works.
