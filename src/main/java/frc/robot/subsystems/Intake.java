package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeStates;

public class Intake extends SubsystemBase {
    private IntakeIO intakeIO;
    public IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();
    private WantedState mDesiredState = WantedState.STOP;
    private SystemState mCurrentState = SystemState.STOP;

    // Intake States
    public enum WantedState {
        INTAKE,
        STOP,
        EXTEND,
        EXTEND_AND_RUN,
        RETRACT
    }

    public enum SystemState {
        INTAKE,
        STOP,
        EXTEND,
        EXTEND_AND_RUN,
        RETRACT
    }

    public Intake() {
        this.intakeIO = new IntakeIOReal();
        //initDefaultCommand();
    }

    public void retract() {
        // TODO: use this when the intake is fixed to be able to retract fully
        // intakeIO.retract();
        intakeIO.moveToPosition(Constants.Intake.INTAKE_RETRACTION_LIMIT);
    }

    public void extend() {
        // intakeIO.moveToPosition(Constants.Intake.INTAKE_FORWARD_DISTANCE_LIMIT);
        intakeIO.extendForward();
        // return new InstantCommand(() -> intakeIO.moveToPosition(Constants.Intake.INTAKE_FORWARD_DISTANCE_LIMIT), this);
    }

    public void extendAndRun() {
        extend();
        run();
    }

    public void run() {
        intakeIO.setState(IntakeStates.FORWARD);
    }

    // public Command extendAndRun() {
    //     return new InstantCommand(() -> {
    //         intakeIO.moveToPosition(Constants.Intake.INTAKE_FORWARD_DISTANCE_LIMIT);
    //         intakeIO.setState(IntakeStates.FORWARD);
    //     }, this);
    // }

    public boolean checkExtended() {
        // Rotations value is actually inches because of configured gear ratio
        // boolean isExtended = (intakeInputs.expansionMotorAngle.gte(Constants.Intake.expansionMotorMaxDistance));
        // Logger.recordOutput("Intake/IsExtended", isExtended);
        // return isExtended;
        return false;
    }

    private boolean checkRetracted() {
        // Rotations value is actually inches because of configured gear ratio
        // TODO: Find acutal retracted value
        // boolean isRetracted = intakeInputs.expansionMotorAngle.lte(Units.Inches.of(0));
        // return isRetracted;
        return false;
    }

    public void setWantedState(WantedState state) {
        mDesiredState = state;
    }

    /* 
    public void initDefaultCommand() {
        setDefaultCommand(new RetractAndStop(this));
    }
    */

    private SystemState handleStateTransition() {
        return switch (mDesiredState) {
            case STOP -> SystemState.STOP;
            case INTAKE -> {
                // TODO: add protections in here in future iteration
                yield SystemState.INTAKE;
            }
            case EXTEND -> {
                yield SystemState.EXTEND;
            }
            case EXTEND_AND_RUN -> {
                // TODO: add protections in here in future iteration
                yield SystemState.EXTEND_AND_RUN;
            }
            case RETRACT -> {
                yield SystemState.RETRACT;
            }
            default -> SystemState.STOP;
        };
    }

    private void applyStates() {
        switch (mCurrentState) {
            default:
                break;
            case STOP:
                intakeIO.setState(IntakeStates.OFF);
                intakeIO.stopMotion();
                break;
            case INTAKE:
                intakeIO.setState(IntakeStates.FORWARD);
                break;
            case EXTEND:
                extend();
                break;
            case RETRACT:
                retract();
                break;
            case EXTEND_AND_RUN:
                extendAndRun();
                break;
        }
    }

    @Override
    public void periodic() {
        // this.checkExtended(); // Testing only
        Logger.recordOutput("Intake/State", mCurrentState);
        Logger.recordOutput("Intake/Intakevelocity", this.intakeInputs.intakeAngularVelocity);
        Logger.recordOutput("Intake/position", this.intakeInputs.intakePosition);
        Logger.recordOutput("Intake/RackVelocity", this.intakeInputs.intakeRackSpeed);
        this.intakeIO.updateInputs(this.intakeInputs);
        Logger.processInputs("Intake", this.intakeInputs);

        mCurrentState = handleStateTransition();
        applyStates();
    }

    @Override
    public void simulationPeriodic() {
        intakeIO.simPeriodic();
    }
}
