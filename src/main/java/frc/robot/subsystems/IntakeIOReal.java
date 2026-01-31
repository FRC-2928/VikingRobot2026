package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeIOReal implements IntakeIO {
    public TalonFX intake;
    public StatusSignal<AngularVelocity> statusSignal;

    public IntakeIOReal() {
        this.intake = new TalonFX(16);
        this.statusSignal = this.intake.getRotorVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(100, statusSignal);
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        config.CurrentLimits = currentLimitsConfigs;

        currentLimitsConfigs.StatorCurrentLimit = 40; // the peak current, in amps
        intake.getConfigurator().apply(config); // apply the config settings; this selects the quadrature encode
        // intake.setInverted(true);
    }

    @Override
    public void setSpeed(AngularVelocity angularVelocity) {
        // Do a feed forward later
        intake.setControl(new DutyCycleOut(angularVelocity.in(Units.RotationsPerSecond) / 30));
    }

    @Override
    public void updateInputs(IntakeIOInputs intakeInputs) {
        BaseStatusSignal.refreshAll(statusSignal);
        intakeInputs.angularVelocity = statusSignal.getValue();
    }
}
