# Requirements Document

## Introduction

This document specifies the requirements for a centralized state machine architecture for an FRC robot. The architecture uses a Superstructure subsystem as the central orchestrator that manages robot states, coordinates subsystem actions, and handles state transitions based on driver inputs and robot conditions. The system translates driver inputs into StateIntents, validates transitions, and ensures safe robot operation across multiple operational modes.

## Glossary

- **Superstructure**: The central state machine subsystem that orchestrates all robot subsystems and manages state transitions
- **StateIntent**: An enumerated request from DriverOI representing a desired state change or action
- **RobotState**: An enumerated value representing the current operational mode of the robot
- **DriverOI**: The driver operator interface that translates joystick inputs into StateIntents
- **State_Transition**: The process of moving from one RobotState to another
- **Transition_Validation**: The process of determining whether a requested state transition is safe and valid
- **State_Command**: A WPILib Command that executes continuously while a specific RobotState is active
- **Subsystem**: A WPILib SubsystemBase representing a physical robot mechanism (Shooter, Intake, Drivetrain, etc.)

## Requirements

### Requirement 1: StateIntent Definition and Translation

**User Story:** As a robot programmer, I want driver inputs to be translated into semantic StateIntents, so that the state machine can make intelligent decisions about state transitions.

#### Acceptance Criteria

1. THE Superstructure SHALL define a StateIntent enum containing all possible driver action requests
2. WHEN a driver presses a button or moves a joystick, THE DriverOI SHALL translate the input into a corresponding StateIntent
3. THE StateIntent enum SHALL include intents for: manual intake, manual shooting, autonomous aiming, drive mode toggling, and idle operations
4. WHEN DriverOI creates a StateIntent, THE StateIntent SHALL be passed to Superstructure for evaluation
5. THE StateIntent SHALL be immutable once created

### Requirement 2: State Machine Architecture

**User Story:** As a robot programmer, I want a centralized state machine in Superstructure, so that robot behavior is predictable and all subsystems are coordinated.

#### Acceptance Criteria

1. THE Superstructure SHALL maintain a currentState field of type RobotState
2. THE Superstructure SHALL define a RobotState enum containing all valid robot operational states
3. WHEN Superstructure is initialized, THE Superstructure SHALL set currentState to a safe default state
4. THE Superstructure SHALL maintain a mapping from each RobotState to its corresponding State_Command
5. WHEN currentState changes, THE Superstructure SHALL schedule the State_Command associated with the new state
6. THE Superstructure SHALL execute state transition logic in its periodic() method
7. THE RobotState enum SHALL include states for: home zone driving, mid zone driving, aiming, manual intake, and idle

### Requirement 3: State Transition Validation

**User Story:** As a robot programmer, I want state transitions to be validated before execution, so that the robot never enters an unsafe or invalid configuration.

#### Acceptance Criteria

1. WHEN a StateIntent is received, THE Superstructure SHALL evaluate whether the transition is valid from the current state
2. IF a transition is invalid, THEN THE Superstructure SHALL reject the StateIntent and log a warning
3. IF a transition is valid, THEN THE Superstructure SHALL update currentState to the new state
4. THE Superstructure SHALL maintain a transition validation function for each RobotState
5. WHEN validating a transition, THE Superstructure SHALL check robot sensor states and subsystem readiness
6. THE Superstructure SHALL prevent transitions that would cause subsystem conflicts

### Requirement 4: Automatic State Transitions

**User Story:** As a robot programmer, I want the robot to automatically transition between states based on sensor data, so that the robot adapts to changing conditions without driver intervention.

#### Acceptance Criteria

1. WHEN the robot's field position changes zones, THE Superstructure SHALL automatically transition to the appropriate zone state
2. WHEN a game piece is detected in the intake, THE Superstructure SHALL automatically transition from intake state to ready state
3. WHEN shot conditions are met during aiming, THE Superstructure SHALL enable shooting
4. THE Superstructure SHALL evaluate automatic transition conditions in each periodic() cycle
5. WHEN an automatic transition occurs, THE Superstructure SHALL log the transition with timestamp and reason

### Requirement 5: State Command Initialization and Execution

**User Story:** As a robot programmer, I want each state to have an associated command that runs while that state is active, so that robot behavior is encapsulated and maintainable.

#### Acceptance Criteria

1. WHEN Superstructure is constructed, THE Superstructure SHALL initialize State_Commands for all RobotStates
2. WHEN a RobotState becomes active, THE Superstructure SHALL schedule its associated State_Command
3. WHEN a RobotState becomes inactive, THE Superstructure SHALL cancel its associated State_Command
4. THE Superstructure SHALL use WPILib Triggers to bind State_Commands to state activation
5. IF a State_Command is not defined for a RobotState, THEN THE Superstructure SHALL log an error and use a safe default command

### Requirement 6: DriverOI Integration

**User Story:** As a robot driver, I want my controller inputs to be properly interpreted and sent to the state machine, so that I can control the robot effectively.

#### Acceptance Criteria

1. THE DriverOI SHALL accept a Superstructure reference in its constructor
2. WHEN a driver input occurs, THE DriverOI SHALL create the appropriate StateIntent
3. THE DriverOI SHALL call a Superstructure method to submit StateIntents
4. THE DriverOI SHALL maintain Trigger objects for all driver inputs
5. THE DriverOI SHALL configure button bindings in a configureControls() method that accepts no parameters
6. WHEN configureControls() is called, THE DriverOI SHALL bind all controller buttons to their corresponding StateIntent submissions

### Requirement 7: State Trigger Initialization

**User Story:** As a robot programmer, I want state triggers to be properly initialized, so that state commands execute correctly without runtime errors.

#### Acceptance Criteria

1. WHEN Superstructure is constructed, THE Superstructure SHALL initialize the stateTriggers list before use
2. THE Superstructure SHALL create one Trigger for each RobotState
3. WHEN a state Trigger is created, THE Superstructure SHALL bind it to execute the state's command while the state is active
4. THE Superstructure SHALL store all state Triggers in the stateTriggers list
5. IF stateTriggers is accessed before initialization, THEN THE Superstructure SHALL throw a clear error message

### Requirement 8: Transition Function Management

**User Story:** As a robot programmer, I want each state to have a dedicated transition function, so that state-specific logic is organized and maintainable.

#### Acceptance Criteria

1. THE Superstructure SHALL maintain a Map from RobotState to transition functions
2. WHEN Superstructure is constructed, THE Superstructure SHALL register a transition function for each RobotState
3. WHEN periodic() executes, THE Superstructure SHALL call the transition function for currentState
4. IF no transition function exists for currentState, THEN THE Superstructure SHALL log a warning with the state name
5. THE transition function SHALL evaluate both StateIntents and sensor conditions to determine state changes

### Requirement 9: StateIntent Handling Method

**User Story:** As a robot programmer, I want a clear method for submitting StateIntents to Superstructure, so that the interface between DriverOI and Superstructure is well-defined.

#### Acceptance Criteria

1. THE Superstructure SHALL provide a public method to accept StateIntent submissions
2. WHEN a StateIntent is submitted, THE Superstructure SHALL validate the transition
3. IF the transition is valid, THEN THE Superstructure SHALL update currentState
4. IF the transition is invalid, THEN THE Superstructure SHALL return a failure indication
5. THE method SHALL return a Command that can be bound to controller buttons
6. THE method SHALL log all StateIntent submissions with timestamp and current state

### Requirement 10: Safe Default State

**User Story:** As a robot programmer, I want the robot to start in a safe default state, so that the robot doesn't perform dangerous actions on startup.

#### Acceptance Criteria

1. WHEN Superstructure is constructed, THE Superstructure SHALL initialize currentState to a safe idle state
2. THE idle state SHALL stop all motors and retract all mechanisms
3. THE idle state SHALL be a valid transition target from any other state
4. WHEN the robot is disabled, THE Superstructure SHALL transition to the idle state
5. THE idle state command SHALL continuously ensure all subsystems are in safe positions

### Requirement 11: State Logging and Diagnostics

**User Story:** As a robot programmer, I want comprehensive logging of state transitions and StateIntents, so that I can debug issues and analyze robot behavior.

#### Acceptance Criteria

1. WHEN a state transition occurs, THE Superstructure SHALL log the previous state, new state, and trigger reason
2. WHEN a StateIntent is received, THE Superstructure SHALL log the intent type and current state
3. WHEN a StateIntent is rejected, THE Superstructure SHALL log the rejection reason
4. THE Superstructure SHALL log the current state to NetworkTables every periodic cycle
5. THE Superstructure SHALL maintain a transition history buffer of the last 50 transitions
6. WHEN periodic() executes, THE Superstructure SHALL log execution time for performance monitoring

### Requirement 12: Subsystem Coordination

**User Story:** As a robot programmer, I want Superstructure to coordinate multiple subsystems safely, so that subsystems don't conflict or interfere with each other.

#### Acceptance Criteria

1. THE Superstructure SHALL maintain references to all robot subsystems
2. WHEN a State_Command executes, THE Superstructure SHALL coordinate commands across multiple subsystems
3. THE Superstructure SHALL prevent simultaneous conflicting subsystem operations
4. WHEN transitioning states, THE Superstructure SHALL ensure subsystems complete critical operations before state change
5. THE Superstructure SHALL provide methods that return coordinated multi-subsystem commands

### Requirement 13: RobotContainer Integration

**User Story:** As a robot programmer, I want RobotContainer to properly initialize the state machine architecture, so that all components are wired together correctly.

#### Acceptance Criteria

1. THE RobotContainer SHALL construct Superstructure after all subsystems are created
2. THE RobotContainer SHALL pass itself as a reference to Superstructure constructor
3. THE RobotContainer SHALL construct DriverOI with references to both the controller and Superstructure
4. THE RobotContainer SHALL call DriverOI.configureControls() during initialization
5. THE RobotContainer SHALL not directly bind commands to subsystems that are managed by Superstructure
