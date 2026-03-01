package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.Intake.RetractAndStop;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private IntakeIO intakeIO;
    public IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();

    public Intake() {
        this.intakeIO = new IntakeIOReal();
        initDefaultCommand();
    }

    public void setIntakeSpeed(double speed) {
        intakeIO.setSpeed(speed);
        Logger.recordOutput("Intake/Roller Speed", speed);
    }

    public void retract() {
        intakeIO.retract();
    }

    public void extend() {
        intakeIO.extend();
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
