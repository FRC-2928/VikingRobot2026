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
    private TalonFX intakeRollerMotor;
    private TalonFX intakeExpansionMotor;
    public StatusSignal<AngularVelocity> intakeAngularVelocity;

    public IntakeIOReal() {
        // The Intake Roller motor
        this.intakeRollerMotor = new TalonFX(Constants.CAN.CTRE.intakeRoller, Constants.CAN.CTRE.bus);

        final TalonFXConfiguration intakeRollerConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs intakeRollerCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Units.Amps.of(80))
                .withSupplyCurrentLimit(Units.Amps.of(60))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);

        MotorOutputConfigs intakeRollerOutputConfigs =
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
        intakeRollerConfig
                .withMotorOutput(intakeRollerOutputConfigs)
                .withCurrentLimits(intakeRollerCurrentLimitsConfigs);
        intakeRollerMotor.getConfigurator().apply(intakeRollerConfig); // apply the config settings

        // The Intake Expansion motor
        this.intakeExpansionMotor = new TalonFX(Constants.CAN.CTRE.intakeExpansion, Constants.CAN.CTRE.bus);

        final TalonFXConfiguration intakeExpansionConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs intakeExpansionCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Units.Amps.of(80))
                .withSupplyCurrentLimit(Units.Amps.of(60))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);

        MotorOutputConfigs intakeExpansionOutputConfigs =
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
        intakeExpansionConfig
                .withMotorOutput(intakeExpansionOutputConfigs)
                .withCurrentLimits(intakeExpansionCurrentLimitsConfigs);
        intakeRollerMotor.getConfigurator().apply(intakeExpansionConfig); // apply the config settings

        this.intakeAngularVelocity = this.intakeRollerMotor.getRotorVelocity();
        BaseStatusSignal.setUpdateFrequencyForAll(Units.Hertz.of(100), intakeAngularVelocity);
    }

    @Override
    public void setSpeed(double speed) {
        // Do a feed forward later
        intakeRollerMotor.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void expand() {
        // Expand and stop once fully expanded
        // intakeExpansionMotor.setControl();
    }

    @Override
    public void retract() {
        // Retract using the torque control
        // intakeExpansionMotor.setControl();
    }

    @Override
    public void updateInputs(IntakeInputs intakeInputs) {
        BaseStatusSignal.refreshAll(intakeAngularVelocity);
        intakeInputs.angularVelocity = intakeAngularVelocity.getValue();
    }
}
