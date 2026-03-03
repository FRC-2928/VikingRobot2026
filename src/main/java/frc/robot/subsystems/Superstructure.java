package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ExtendAndRunIntake;
import frc.robot.commands.Intake.IntakeGround;

public class Superstructure extends SubsystemBase {

    public enum StateIntent {
        ACTION_TOGGLE_TARGET_LOCK_MODE,
        ACTION_SHOOT_OVERRIDE,
        ACTION_INTAKE_MANUAL,
        ACTION_INTAKE_AUTO,
        ACTION_NONE;

        private StateIntent() {
            isIntended = false;
        }

        boolean isIntended;

        public void setIsIntended(boolean isIntended) { this.isIntended = isIntended; }

        public boolean getIsInteded() { return isIntended; }

        public void toggleIntent() { this.isIntended = !isIntended; }
    }

    public enum OverrideIntent {
        OVERRIDE_SHOOT_MODE,
        OVERRIDE_INTAKE_MODE,
        OVERRIDE_NONE
    }

    public enum RobotState {
        DISABLED,
        AUTONOMOUS,
        FREE_DRIVE,
        DRIVE_TARGET_LOCK,
        SHOOTING,
        MANUAL_INTAKE,
        AUTO_INTAKE,
        MID_FIELD,
        GET_READY_CLIMB,
        UNJAM
    }

    // --------------------- Class Members ---------------------
    /// singleton instance of the Superstructure
    private static Superstructure sInstance = null;
    /// Convenience reference to the RobotContainer instance
    private final RobotContainer mRobotContainer;
    /// Map of Subsystems to their desired signals to be refreshed
    private Map<SubsystemBase, List<BaseStatusSignal>> mSubsystemSignalsMap;
    /// Current state of the subsystem
    private RobotState currentState = RobotState.DISABLED;  // disabled by default
    /// Stored (previous) state of the subsystem -- used for handling state overrides
    private RobotState prevState = null;
    /// List of triggers for the states
    private List<Trigger> stateTriggers;
    /// Map of RobotStates to transition validation functions
    private Map<RobotState, Runnable> transitionFunctions;

    // --------------------- Internal State Variables ---------------------
    /// boolean tracking the target lock state
    private boolean mTargetLockRequested = false;

    // Overrides
    /**
     * Enumeration of different overrides -- represented as bitflags
     */
    private enum StateOverrides {
        OVERRIDE_SHOOTING,    // bit 0
        OVERRIDE_INTAKING     // bit 1
        // ... future overrides would be bit 2, 3, etc.
    }

    /// Set tracking which overrides are currently active
    private EnumSet<StateOverrides> mActiveOverrides = EnumSet.noneOf(StateOverrides.class);
    // Metrics
    /// counter tracking the incidence rate of multiple simultaneous override requests
    private int mSimultaneousOverrideRequests = 0;
    /// counter tracking the incidence rate of clearing overrides without any active
    private int mNoActiveOverridesCount = 0;

    /**
     * Singleton creator for the Superstructure -- creates a new instance if one is not available
     *
     * @param robotContainer the @c RobotContainer instance to use
     * @return a fully initialized Superstructure instance
     */
    public static synchronized Superstructure create(RobotContainer robotContainer) {
        if (sInstance != null) {
            return sInstance;
        }

        sInstance = new Superstructure(robotContainer);
        return sInstance;
    }

    /**
     * Singleton accessor -- intended to be invoked after an instance is already available via create()
     *
     * @return the singleton instance of the Superstructure
     */
    public static synchronized Superstructure getInstance() {
        return sInstance;
    }

    /**
     * Constructor for the Superstructure
     *
     * @param robotContainer the @c RobotContainer convenience reference
     */
    private Superstructure(RobotContainer robotContainer) {
        this.mRobotContainer = robotContainer;
        this.stateTriggers = new ArrayList<>();
    }

    // Initializes trigger for when given state is active, and runs given command when trigger is active
    private void initState(RobotState state, Command runWhenCurrentState) {
        stateTriggers.add(new Trigger(() -> this.currentState == state).whileTrue(runWhenCurrentState));
    }

    public void init() {
        // Init each state's command to run
        initState(RobotState.DISABLED, handleDisabled());
        initState(RobotState.AUTONOMOUS, handleAutonomous());
        initState(RobotState.FREE_DRIVE, freeDrive());
        initState(RobotState.DRIVE_TARGET_LOCK, driveTargetLock());
        initState(RobotState.MANUAL_INTAKE, extendAndIntake());
        initState(RobotState.AUTO_INTAKE, autoIntake());
        initState(RobotState.SHOOTING, startShooting());

        transitionFunctions = new HashMap<>();

        // Bind transition function for each state -- these functions determine if the state should be changed
        // this should really be a map... inbounds + triggers set a requested state and then we look up the current state
        // in the map and find the valid transition state if one exists or reject the transition...
        // in the current impl we have to check if we need to transition to, for example, disabled in every state...
        transitionFunctions.put(RobotState.DISABLED, this::checkTransitionFromDisabled);
        transitionFunctions.put(RobotState.AUTONOMOUS, this::checkTransitionFromAutonomous);
        transitionFunctions.put(RobotState.FREE_DRIVE, this::checkTransitionFromFreeDrive);
        transitionFunctions.put(RobotState.DRIVE_TARGET_LOCK, this::checkTransitionFromTargetLock);
        transitionFunctions.put(RobotState.SHOOTING, this::checkTransitionFromShooting);
        transitionFunctions.put(RobotState.MANUAL_INTAKE, this::checkManualIntakeTransition);
        transitionFunctions.put(RobotState.AUTO_INTAKE, this::checkTransitionFromAutoIntake);
    }

    /**
     * Applies the provided @c StateIntent and transitions the internal state machine accordingly
     *
     * @param intent the requested @c StateIntent
     */
    private void toggleIntent(StateIntent intent) {
        switch (intent) {
            case ACTION_TOGGLE_TARGET_LOCK_MODE: {
                mTargetLockRequested = !mTargetLockRequested;
                break;
            }
            default: {
                // no-op
                break;
            }
        }
    }

    public Command setIntent(StateIntent intent, boolean intended) {
        return new InstantCommand(() -> intent.setIsIntended(intended));
    }

    public Command toggleStateIntent(StateIntent intent) {
        return new InstantCommand(() -> toggleIntent(intent));
    }

    /**
     * Handles override requests and transitions the internal state machine accordingly
     * @return the command to run for the shoot override request
     */
    public Command requestShootOverride() {
        return new InstantCommand(() -> requestOverride(OverrideIntent.OVERRIDE_SHOOT_MODE), this);
    }

    /**
     * Clears the override state and returns the robot to the previously-active state
     * @return the command to clear the override state
     */
    public Command clearOverrideCommand() {
        return new InstantCommand(() -> requestOverride(OverrideIntent.OVERRIDE_NONE), this);
    }

    /**
     * Handles override requests and transitions the internal state machine accordingly
     *
     * @param intent the requested @c OverrideIntent
     */
    private synchronized void requestOverride(OverrideIntent intent) {
        if (intent == OverrideIntent.OVERRIDE_NONE) {
            if (mActiveOverrides.isEmpty()) {
                // No overrides to clear
                mNoActiveOverridesCount++;
                Logger.recordOutput("Superstructure/NoActiveOverridesCount", mNoActiveOverridesCount);
                return;
            }

            // Clear all active overrides
            mActiveOverrides.clear();
            Logger.recordOutput("Superstructure/OverrideState", "NONE");
            
            // Restore previous state
            if (prevState != null) {
                currentState = prevState;
                prevState = null;
            }
            return;
        }
        
        // Check if any override is already active
        if (!mActiveOverrides.isEmpty()) {
            // Reject the override request
            mSimultaneousOverrideRequests++;
            Logger.recordOutput("Superstructure/SimultaneousOverrideCount", mSimultaneousOverrideRequests);
            return;
        }

        // Save current state before applying override so we can safely transition back once the override is done
        prevState = currentState;
        
        switch (intent) {
            case OVERRIDE_SHOOT_MODE: {
                // Add OVERRIDE_SHOOTING to the set of active overrides
                mActiveOverrides.add(StateOverrides.OVERRIDE_SHOOTING);  // Internally: 01 (bit 0 is now 1)
                currentState = RobotState.SHOOTING;  // transition directly into shooting mode
                Logger.recordOutput("Superstructure/OverrideState", "SHOOTING");
                break;
            }
            case OVERRIDE_INTAKE_MODE:
                break;
            // ... other overrides here
            case OVERRIDE_NONE:
            default:
                break;
        }
    }

    /**
     * Utility function to get a supplier of the robot state
     *
     * @return a supplier for the current @c RobotState
     */
    public Supplier<RobotState> getRobotStateSupplier() {
        return () -> currentState;
    }

    /**
     * Executes periodic logic for the Superstructure, including state transitions
     */
    @Override
    public void periodic() {
        Logger.recordOutput("Superstructure/SimultaneousOverrideRequests", mSimultaneousOverrideRequests);
        Logger.recordOutput("Superstructure/NoActiveOverridesCount", mNoActiveOverridesCount);

        RobotState lastState;
        do {
            lastState = currentState;  // track the most recent state of the robot in case it changes
            // Get the transition function for the current state and execute it
            transitionFunctions
                    .getOrDefault(currentState, () -> {
                        Logger.recordOutput("Superstructure/ErrorCurrentStateMissingTransitionFunction", true);
                        System.out.println(String.format("[ERROR] Missing transition function for current state: %s", currentState));
                    })
                    .run();
        } while (currentState != lastState);

        Logger.recordOutput("Superstructure/CurrentState", currentState);
    }

    private Command handleDisabled() {
        // TODO: implement this... no-op state, put everything into a ground/idle state...
        return new InstantCommand();
    }

    private Command handleAutonomous() {
        // TODO: implement this... probably set drivetrain to auto mode... just a no-op state internally
        // since auto routines will control drivetrain through direct invocations...
        return new InstantCommand();
    }

    private Command freeDrive() {
        return mRobotContainer.drivetrain.freeDrive();
    }

    private Command handleShooting() {
        // TODO: implement this
        return new InstantCommand();
    }

    private Command driveTargetLock() {
        return new ParallelCommandGroup(
                mRobotContainer.drivetrain.targetLock(),
                prepareShooter())
            .andThen(startShooting());
    }

    private Command extendIntakeCommand()
    {
        return mRobotContainer.intake.extend();
    }

    private void checkTransitionFromDisabled() {
        if (DriverStation.isAutonomousEnabled()) {
            currentState = RobotState.AUTONOMOUS;
        }

        if (DriverStation.isTeleopEnabled()) {
            currentState = RobotState.FREE_DRIVE;
        }
    }

    private void checkTransitionFromAutonomous() {
        // should never happen -- precautionary fallback though
        if (DriverStation.isTeleopEnabled()) {
            currentState = RobotState.FREE_DRIVE;
        }

        // robot state should ALWAYS transition through disabled in between match phases...
        // we'll set the expectation appropriately...
        if (DriverStation.isDisabled()) {
            currentState = RobotState.DISABLED;
        }
    }

    private void checkTransitionFromFreeDrive() {
        if (mTargetLockRequested) {
            currentState = RobotState.DRIVE_TARGET_LOCK;
        } else if () {
            currentState = RobotState.AUTO_INTAKE;
        }
    }

    private void checkTransitionFromTargetLock() {
        // TODO: implement
        if (!mTargetLockRequested) {
            currentState = RobotState.FREE_DRIVE;
            return;
        }

        // if we're here then target lock mode is still active...
        // so we need to check if we're able to transition into other modes...
        // if (mRobotContainer.drivetrain.atTargetLockPose())
    }

    private void checkTransitionFromShooting() {
        // Override has been cleared - state already restored by requestOverride()
        // This function handles any other transitions out of shooting if needed
        // For now, shooting state is only exited via override cancellation
    }

    private void checkManualIntakeTransition() {
        // if (driverOI.intake.getAsBoolean()) {
        //     currentState = RobotState.MANUAL_INTAKE;
        // }
    }

    private void checkTransitionFromAutoIntake() {
        if (!StateIntent.ACTION_INTAKE_AUTO.getIsInteded()) {
            currentState = RobotState.FREE_DRIVE;
        }
    }

    // Runs flywheels and kicker. Command will not end on its own
    public Command startShooting() {
        return new RunCommand(
                () -> {
                    mRobotContainer.shooter.shoot();
                },
                mRobotContainer.shooter)
            .alongWith(mRobotContainer.drivetrain.brake())
            .alongWith(mRobotContainer.hopperFloor.runHopperCommand())
            .alongWith(mRobotContainer.indexer.runIndexerCommand());
    }

    // Spins up flywheels to speed and turns hood to correct angle. Command will not end on its own
    public Command prepareShooter() {
        return new RunCommand(
                () -> {
                    mRobotContainer.shooter.aim();
                },
                mRobotContainer.shooter);
            // .alongWith(mRobotContainer.drivetrain.aimAtHubAndMove(mRobotContainer.driverOI.controller, 0));
    }

    public Command shootAutomated() {
        return new SequentialCommandGroup(prepareShooter(), startShooting());
    }

    // Stops robot from shooting
    public Command idle() {
        return new InstantCommand(
            () -> {
                mRobotContainer.drivetrain.setState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);      
            });
    }

    public Command extendAndIntake() {
        return new ExtendAndRunIntake(mRobotContainer.intake);
    }

    public Command autoIntake() {
        return new IntakeGround(true, mRobotContainer, 1.0);
    }

    public Command pathWhileIntaking(String pathFileName) {
        return new InstantCommand();
        // return new ParallelDeadlineGroup(mRobotContainer.drivetrain.runPath(pathFileName), this.extendAndIntake())
        //         .finallyDo(() -> {
        //             mRobotContainer.intake.;
        //             mRobotContainer.intake.retract();
        //         });
    }

    public void resetSubsystems() {
        // TODO: Implement Climber
        mRobotContainer.drivetrain.setState(CommandSwerveDrivetrain.WantedState.IDLE);
        mRobotContainer.hopperFloor.halt();
        mRobotContainer.indexer.halt();
        mRobotContainer.intake.setWantedState(IntakeIOReal.WantedState.RETRACT);
        mRobotContainer.shooter.home();
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = mRobotContainer.getTeleopMatchTime();
        double secondsLeftTeleop = 140 - matchTime;
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active =
                switch (alliance.get()) {
                    case Red -> !redInactiveFirst;
                    case Blue -> redInactiveFirst;
                };

        if (secondsLeftTeleop > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (secondsLeftTeleop > 105) {
            // Shift 1
            return shift1Active;
        } else if (secondsLeftTeleop > 80) {
            // Shift 2
            return !shift1Active;
        } else if (secondsLeftTeleop > 55) {
            // Shift 3
            return shift1Active;
        } else if (secondsLeftTeleop > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }
}
