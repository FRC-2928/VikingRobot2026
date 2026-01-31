package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
    private TalonSRX motor;
    public IntakeIO intakeIO;
    public IntakeIOInputs intakeIOInputs = new IntakeIOInputs();

    public Intake() {
        motor = new TalonSRX(10); // creates a new TalonSRX with ID 0
        this.intakeIO = new IntakeIOReal();
    }

    public void runIntake(double speed) {
        motor.set(TalonSRXControlMode.PercentOutput, speed); // runs the motor at 50% power
    }

    @Override
    public void periodic() {
        this.intakeIO.updateInputs(this.intakeIOInputs);
    }
}
