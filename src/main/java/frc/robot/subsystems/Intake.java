package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.IntakeIO.IntakeInputs;

public class Intake extends SubsystemBase {
    public IntakeIO intakeIO;
    public IntakeInputs intakeInputs = new IntakeInputs();

    public Intake() {
        this.intakeIO = new IntakeIOReal();
    }

    public void setIntakeSpeed(double speed) {
        intakeIO.setSpeed(speed);
    }

    public void retract() {
        intakeIO.retract();
    }

    @Override
    public void periodic() {
        this.intakeIO.updateInputs(this.intakeInputs);
    }
}
