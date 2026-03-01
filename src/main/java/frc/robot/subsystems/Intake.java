package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeStates;
import frc.robot.commands.Intake.RetractAndStop;

public class Intake extends SubsystemBase {
    private IntakeIO intakeIO;
    public IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();

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

    public void setWantedState(IntakeIOReal.WantedState state) {
        intakeIO.setWantedState(state);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new RetractAndStop(this));
    }

    @Override
    public void periodic() {
        this.checkExtended(); // Testing only

        this.intakeIO.updateInputs(this.intakeInputs);
        this.intakeIO.stateMachinePeriodic();
        Logger.processInputs("Intake", this.intakeInputs);
    }

    @Override
    public void simulationPeriodic() {
        intakeIO.simPeriodic();
    }
}
