package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.Constants;
import frc.robot.Tuning;

public class IndexerIOReal implements IndexerIO {
    public TalonFX indexer;
    public StatusSignal<AngularVelocity> indexerVelocity;

    public IndexerIOReal() {
        // TODO: change CAN ID
        this.indexer = new TalonFX(Constants.CAN.CTRE.indexer, Constants.CAN.CTRE.bus);
        this.indexerVelocity = this.indexer.getRotorVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(100, indexerVelocity);
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        config.CurrentLimits = currentLimitsConfigs;

        currentLimitsConfigs.StatorCurrentLimit = 40; // the peak current, in amps
        indexer.getConfigurator().apply(config); // apply the config settings; this selects the quadrature encode

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimit = 120.0;

        indexer.getConfigurator().apply(config);
    }

    @Override
    public void setSpeed(double angularVelocity) {
        // Do a feed forward later
        indexer.setControl(new DutyCycleOut(MathUtil.clamp(angularVelocity, -1, 1)));
    }

    @Override
    public void runIndexer() {
        indexer.setControl(new DutyCycleOut(MathUtil.clamp(Tuning.intakeVelocity.get(), -1, 1)));
    }

    @Override
    public void halt() {
        indexer.setControl(new DutyCycleOut(0.0));
    }

    @Override
    public void updateInputs(IndexerIOInputs indexerInputs) {
        BaseStatusSignal.refreshAll(indexerVelocity);
        indexerInputs.angularVelocity = indexerVelocity.getValue();
    }

    @Override
    public void simPeriodic() {}
}
