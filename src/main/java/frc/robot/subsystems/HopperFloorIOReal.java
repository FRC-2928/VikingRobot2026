package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Tuning;

public class HopperFloorIOReal implements HopperFloorIO {
    public TalonFX hopper;
    public StatusSignal<AngularVelocity> statusSignal;

    public HopperFloorIOReal() {
        // TODO: change CAN ID
        this.hopper = new TalonFX(Constants.CAN.CTRE.hopper, Constants.CAN.CTRE.bus);
        this.statusSignal = this.hopper.getRotorVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(100, statusSignal);
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        config.CurrentLimits = currentLimitsConfigs;

        currentLimitsConfigs.StatorCurrentLimit = 40; // the peak current, in amps
        hopper.getConfigurator().apply(config); // apply the config settings; this selects the quadrature encode
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimit = 120.0;

        hopper.getConfigurator().apply(config); 
    }

    @Override
    public void setSpeed(AngularVelocity angularVelocity) {
        // Do a feed forward later
        hopper.setControl(new DutyCycleOut(angularVelocity.in(Units.RotationsPerSecond)));
    }

    @Override
    public void runHopper() {
        hopper.setControl(new DutyCycleOut(Tuning.hopperVelocity.get()));
    }

    @Override
    public void halt() {
        hopper.setControl(new DutyCycleOut(0.0));
    }

    @Override
    public void updateInputs(HopperFloorIOInputs hopperInputs) {
        BaseStatusSignal.refreshAll(statusSignal);
        hopperInputs.angularVelocity = statusSignal.getValue();
    }
}
