package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

    final VelocityVoltage indexerVelocityVoltage;

    public IndexerIOReal() {
        this.indexer = new TalonFX(Constants.CAN.CTRE.indexer, Constants.CAN.CTRE.bus);
        this.indexerVelocity = this.indexer.getRotorVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(100, indexerVelocity);
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        indexerConfig.CurrentLimits = currentLimitsConfigs;

        currentLimitsConfigs.StatorCurrentLimit = 40; // the peak current, in amps
        indexer.getConfigurator().apply(indexerConfig); // apply the config settings; this selects the quadrature encode

        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        indexerConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        indexerConfig.CurrentLimits.StatorCurrentLimit = 120.0;

        //TODO: actually tune these PID's
        Slot0Configs indexerFloorSlot0Configs = new Slot0Configs();
        indexerFloorSlot0Configs.kS = 0.65; // Add 0.1 V output to overcome static friction
        indexerFloorSlot0Configs.kV = 0.105; // A velocity target of 1 rps results in 0.12 V output
        indexerFloorSlot0Configs.kP = 0.5; // An error of 1 rps results in 0.11 V output
        indexerFloorSlot0Configs.kI = 0; // no output for integrated error
        indexerFloorSlot0Configs.kD = 0; // no output for error derivative

        // PID Values
        indexerConfig.Slot0 = indexerFloorSlot0Configs;

        indexer.getConfigurator().apply(indexerConfig); // apply the config settings; this selects the quadrature encode

        // create a velocity closed-loop request, voltage output, slot 0 configs
        this.indexerVelocityVoltage = new VelocityVoltage(0).withSlot(0);

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
        indexer.setControl(indexerVelocityVoltage.withVelocity(60));
        //indexer.setControl(new VoltageOut(8));
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
