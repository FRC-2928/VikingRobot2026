package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeStates;
import frc.robot.commands.Intake.RetractAndStop;

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
        EXTEND_INTAKE,
        RETRACT
    }

    public enum SystemState {
        INTAKE,
        STOP,
        EXTEND,
        EXTEND_INTAKE,
        RETRACT
    }

    public Intake() {
        this.intakeIO = new IntakeIOReal();
        initDefaultCommand();
    }

    public void retract() {
        intakeIO.retract();
    }

    public Command extend() {
        return new InstantCommand(() -> intakeIO.extend(), this);
    }

    public Command extendAndRun() {
        return new InstantCommand(() -> {
            intakeIO.extend();
            intakeIO.setState(IntakeStates.FORWARD);
        }, this);
    }

    public boolean checkExtended() {
        // Rotations value is actually inches because of configured gear ratio
        Boolean isExtended = (intakeInputs.expansionMotorAngle.gte(Constants.Intake.expansionMotorMaxDistance));
        Logger.recordOutput("Intake/IsExtended", isExtended);
        return isExtended;
    }

    private boolean checkRetracted() {
        // Rotations value is actually inches because of configured gear ratio
        // TODO: Find acutal retracted value
        Boolean isRetracted = intakeInputs.expansionMotorAngle.lte(Units.Inches.of(0));
        return isRetracted;
    }

    public void setWantedState(WantedState state) {
        mDesiredState = state;
    }

    public void initDefaultCommand() {
        setDefaultCommand(new RetractAndStop(this));
    }

        private SystemState handleStateTransition() {
        return switch (mDesiredState) {
            case STOP -> SystemState.STOP;
            case INTAKE -> {
                if (!checkExtended()) {
                    yield SystemState.EXTEND;
                }
                yield SystemState.INTAKE;
            }
            case EXTEND -> {
                if (!checkExtended()) {
                    yield SystemState.EXTEND;
                }
                yield SystemState.STOP;
            }
            case EXTEND_INTAKE -> {
                if(!checkExtended()){
                    yield SystemState.EXTEND;
                }
                yield SystemState.INTAKE;
            }
            case RETRACT -> {
                if(!checkRetracted()) {
                    yield SystemState.RETRACT;
                }
                yield SystemState.STOP;
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
        }
    }

    @Override
    public void periodic() {
        this.checkExtended(); // Testing only

        this.intakeIO.updateInputs(this.intakeInputs);
        mCurrentState = handleStateTransition();
        applyStates();
        Logger.processInputs("Intake", this.intakeInputs);
    }

    @Override
    public void simulationPeriodic() {
        intakeIO.simPeriodic();
    }
}
