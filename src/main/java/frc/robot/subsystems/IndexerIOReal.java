package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO {
    public TalonFX indexer;

    public StatusSignal<AngularVelocity> indexerVelocity;
    private final StatusSignal<AngularVelocity> indexerAngularVelocitySignal;
    private final StatusSignal<Current>         indexerStatorCurrentSignal;
    private final StatusSignal<Current>         indexerSupplyCurrentSignal;

    // Collection of all status signals
    private List<BaseStatusSignal> mStatusSignals;

    public IndexerIOReal() {
        this.indexer = new TalonFX(Constants.CAN.CTRE.indexer, Constants.CAN.CTRE.bus);
        this.indexerVelocity = this.indexer.getRotorVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(100, indexerVelocity);
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        config.CurrentLimits = currentLimitsConfigs;

        currentLimitsConfigs.StatorCurrentLimit = 40; // the peak current, in amps
        indexer.getConfigurator().apply(config); // apply the config settings; this selects the quadrature encode

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimit = 120.0;

        indexer.getConfigurator().apply(config);

        this.indexerAngularVelocitySignal = this.indexer.getVelocity();
        this.indexerStatorCurrentSignal = this.indexer.getStatorCurrent();
        this.indexerSupplyCurrentSignal = this.indexer.getSupplyCurrent();

        this.mStatusSignals = List.of(
            indexerAngularVelocitySignal,
            indexerStatorCurrentSignal,
            indexerSupplyCurrentSignal
        );
    }

    @Override
    public void setSpeed(double dutyCycle) {
        // Do a feed forward later
        indexer.setControl(new DutyCycleOut(MathUtil.clamp(dutyCycle, -1, 1)));
    }
    

    @Override
    public void runIndexer() {
        // indexer.setControl(new DutyCycleOut(MathUtil.clamp(Tuning.intakeVelocity.get(), -1, 1)));
        indexer.setControl(new VoltageOut(8));
    }

    @Override
    public void halt() {
        indexer.setControl(new DutyCycleOut(0.0));
    }

    @Override
    public void updateInputs(IndexerIOInputs indexerInputs) {
        BaseStatusSignal.refreshAll(mStatusSignals);

        indexerInputs.angularVelocity = indexerVelocity.getValue();
        
        indexerInputs.indexerAngularVelocity = indexerAngularVelocitySignal.getValue();
        indexerInputs.indexerStatorCurrent = indexerStatorCurrentSignal.getValue();
        indexerInputs.indexerSupplyCurrent = indexerSupplyCurrentSignal.getValue();
    }

    @Override
    public void simPeriodic() {}
}
