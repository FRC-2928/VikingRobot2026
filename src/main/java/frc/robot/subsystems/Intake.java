package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.Intake.RetractAndStop;
import frc.robot.commands.Intake.StopRoller;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    public IntakeIO intakeIO;
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

    public Command retractStop() {
        return new ParallelCommandGroup(new StopRoller(this), new RetractAndStop(this));
    }

    public void initDefaultCommand() {
        setDefaultCommand(new RetractAndStop(this));
    }

    @Override
    public void periodic() {
        this.checkExtended(); // Testing only 

        this.intakeIO.updateInputs(this.intakeInputs);
        Logger.processInputs("Intake", this.intakeInputs);
    }
}
