package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private TalonSRX motor;

    public Intake() {
        motor = new TalonSRX(10); // creates a new TalonSRX with ID 0
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.peakCurrentLimit = 40; // the peak current, in amps
        config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
        config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
        motor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
        motor.setInverted(true);
    }

    public void runIntake(double speed) {
        motor.set(TalonSRXControlMode.PercentOutput, speed); // runs the motor at 50% power
    }

    @Override
    public void periodic() {}
}
