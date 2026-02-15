package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.Intake.Retract;
import frc.robot.commands.Intake.StopRoller;
import frc.robot.subsystems.IntakeIO.IntakeInputs;

public class Intake extends SubsystemBase {
    public IntakeIO intakeIO;
    public IntakeInputs intakeInputs = new IntakeInputs();

    public Intake() {
        this.intakeIO = new IntakeIOReal();
        initDefaultCommand();
    }

    public void setIntakeSpeed(double speed) {
        intakeIO.setSpeed(speed);
    }

    public void retract() {
        intakeIO.retract();
    }

    public void extend() {
        intakeIO.extend();
    }

    public boolean checkExtended() {
        // Rotations value is actually inches because of configured gear ratio
        return (intakeInputs.expansionMotorAngle.in(Units.Rotations)
                >= Constants.Intake.expansionMotorMaxDistance.in(Units.Inches));
    }

    public Command retractStop() {
        return new ParallelCommandGroup(new StopRoller(this), new Retract(this));
    }

    public void initDefaultCommand() {
        setDefaultCommand(this.retractStop());
    }

    @Override
    public void periodic() {
        this.intakeIO.updateInputs(this.intakeInputs);
    }
}
