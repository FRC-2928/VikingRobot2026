package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    public TalonFX intakeMotor;
    public StatusSignal<AngularVelocity> intakeAngularVelocity;

    public IntakeIOReal() {
        this.intakeMotor = new TalonFX(Constants.CAN.CTRE.intake, Constants.CAN.CTRE.bus);
        // TODO: Declare the motor for instake expand

        final TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Units.Amps.of(80))
                .withSupplyCurrentLimit(Units.Amps.of(60))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);

        MotorOutputConfigs outputConfigs =
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
        config.withMotorOutput(outputConfigs).withCurrentLimits(currentLimitsConfigs);
        intakeMotor.getConfigurator().apply(config); // apply the config settings

        this.intakeAngularVelocity = this.intakeMotor.getRotorVelocity();
        BaseStatusSignal.setUpdateFrequencyForAll(Units.Hertz.of(100), intakeAngularVelocity);
    }

    // Add the methods for retract and expand

    @Override
    public void setSpeed(double speed) {
        // Do a feed forward later
        intakeMotor.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void updateInputs(IntakeInputs intakeInputs) {
        BaseStatusSignal.refreshAll(intakeAngularVelocity);
        intakeInputs.angularVelocity = intakeAngularVelocity.getValue();
    }
}
