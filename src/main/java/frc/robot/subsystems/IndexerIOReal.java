package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO {
    public TalonFX indexerUp;
    public TalonFX indexerDown;
    public TalonFX starWheels;

    public StatusSignal<AngularVelocity> indexerUpVelocity;
    private final StatusSignal<AngularVelocity> indexerUpAngularVelocitySignal;
    private final StatusSignal<Current>         indexerUpStatorCurrentSignal;
    private final StatusSignal<Current>         indexerUpSupplyCurrentSignal;

    private final StatusSignal<AngularVelocity> starWheelsAngularVelocitySignal;
    private final StatusSignal<Current>         starWheelsStatorCurrentSignal;
    private final StatusSignal<Current>         starWheelsSupplyCurrentSignal;

    public StatusSignal<AngularVelocity> indexerDownVelocity;
    private final StatusSignal<AngularVelocity> indexerDownAngularVelocitySignal;
    private final StatusSignal<Current>         indexerDownStatorCurrentSignal;
    private final StatusSignal<Current>         indexerDownSupplyCurrentSignal;


    // Collection of all status signals
    private List<BaseStatusSignal> mStatusSignals;

    final VelocityVoltage indexerUpVelocityVoltage;
    final VelocityVoltage indexerDownVelocityVoltage;
    final VelocityVoltage starWheelVelocityVoltage;

    public IndexerIOReal() {
        this.indexerUp = new TalonFX(Constants.CAN.CTRE.indexerUp, Constants.CAN.CTRE.bus);
        this.indexerDown = new TalonFX(Constants.CAN.CTRE.indexerDown, Constants.CAN.CTRE.bus);

        this.starWheels = new TalonFX(Constants.CAN.CTRE.starWheels, Constants.CAN.CTRE.bus);
        this.indexerUpVelocity = this.indexerUp.getRotorVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(100, indexerUpVelocity);
        TalonFXConfiguration indexerUpConfig = new TalonFXConfiguration();
        TalonFXConfiguration indexerDownConfig = new TalonFXConfiguration();
        TalonFXConfiguration starWheelConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        indexerUpConfig.CurrentLimits = currentLimitsConfigs;

        currentLimitsConfigs.StatorCurrentLimit = 40; // the peak current, in amps
        indexerUp.getConfigurator().apply(indexerUpConfig); // apply the config settings; this selects the quadrature encode

        indexerUpConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerUpConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerUpConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerUpConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        indexerUpConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        indexerUpConfig.CurrentLimits.StatorCurrentLimit = 45.0;

        indexerDownConfig = indexerUpConfig.clone();
        indexerDownConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


        starWheelConfig = indexerUpConfig.clone();
        starWheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        Slot0Configs starWheelSlt0Configs = new Slot0Configs();
        starWheelSlt0Configs.kS = 0.65; // Add 0.1 V output to overcome static friction
        starWheelSlt0Configs.kV = 0.105; // A velocity target of 1 rps results in 0.12 V output
        starWheelSlt0Configs.kP = 0.5; // An error of 1 rps results in 0.11 V output
        starWheelSlt0Configs.kI = 0; // no output for integrated error
        starWheelSlt0Configs.kD = 0; // no output for error derivative

        //TODO: actually tune these PID's
        Slot0Configs indexerUpSlot0Configs = new Slot0Configs();
        indexerUpSlot0Configs.kS = 0.65; // Add 0.1 V output to overcome static friction
        indexerUpSlot0Configs.kV = 0.105; // A velocity target of 1 rps results in 0.12 V output
        indexerUpSlot0Configs.kP = 0.5; // An error of 1 rps results in 0.11 V output
        indexerUpSlot0Configs.kI = 0; // no output for integrated error
        indexerUpSlot0Configs.kD = 0; // no output for error derivative

        //TODO: actually tune these PID's - these should NOT be the same as the indexerUp PID's, but they are for now
        Slot0Configs indexerDownSlot0Configs = new Slot0Configs();
        indexerDownSlot0Configs.kS = 0.5; // Add 0.1 V output to overcome static friction
        indexerDownSlot0Configs.kV = 0.1538; // A velocity target of 1 rps results in 0.12 V output
        indexerDownSlot0Configs.kP = 0.6; // An error of 1 rps results in 0.11 V output
        indexerDownSlot0Configs.kI = 0; // no output for integrated error
        indexerDownSlot0Configs.kD = 0; // no output for error derivative

        // PID Values
        indexerUpConfig.Slot0 = indexerUpSlot0Configs;
        indexerDownConfig.Slot0 = indexerDownSlot0Configs;
        starWheelConfig.Slot0 = starWheelSlt0Configs;
        starWheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        starWheelConfig.CurrentLimits.SupplyCurrentLimit = 35.0;

        indexerUp.getConfigurator().apply(indexerUpConfig); // apply the config settings; this selects the quadrature encode
        indexerDown.getConfigurator().apply(indexerDownConfig);
        starWheels.getConfigurator().apply(starWheelConfig);

        // create a velocity closed-loop request, voltage output, slot 0 configs
        this.indexerUpVelocityVoltage = new VelocityVoltage(0).withSlot(0);
        this.indexerDownVelocityVoltage = new VelocityVoltage(0).withSlot(0);
        this.starWheelVelocityVoltage = new VelocityVoltage(0).withSlot(0);

        this.indexerUpAngularVelocitySignal = this.indexerUp.getVelocity();
        this.indexerUpStatorCurrentSignal = this.indexerUp.getStatorCurrent();
        this.indexerUpSupplyCurrentSignal = this.indexerUp.getSupplyCurrent();

        this.indexerDownAngularVelocitySignal = this.indexerDown.getVelocity();
        this.indexerDownStatorCurrentSignal = this.indexerDown.getStatorCurrent();
        this.indexerDownSupplyCurrentSignal = this.indexerDown.getSupplyCurrent();

        this.starWheelsAngularVelocitySignal = this.starWheels.getVelocity();
        this.starWheelsStatorCurrentSignal = this.starWheels.getStatorCurrent();
        this.starWheelsSupplyCurrentSignal = this.starWheels.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100, indexerUpAngularVelocitySignal, indexerUpStatorCurrentSignal, indexerUpSupplyCurrentSignal, indexerDownAngularVelocitySignal, indexerDownStatorCurrentSignal, indexerDownSupplyCurrentSignal, starWheelsAngularVelocitySignal, starWheelsStatorCurrentSignal, starWheelsSupplyCurrentSignal);
        

        this.mStatusSignals = List.of(
            indexerUpAngularVelocitySignal,
            indexerUpStatorCurrentSignal,
            indexerUpSupplyCurrentSignal,
            starWheelsAngularVelocitySignal,
            starWheelsStatorCurrentSignal,
            starWheelsSupplyCurrentSignal,
            indexerDownAngularVelocitySignal,
            indexerDownStatorCurrentSignal,
            indexerDownSupplyCurrentSignal
        );
    }

    @Override
    public void setSpeedIndexer(double dutyCycle) {
        // Do a feed forward later
        indexerUp.setControl(new DutyCycleOut(MathUtil.clamp(dutyCycle, -1, 1)));
        indexerDown.setControl(new DutyCycleOut(MathUtil.clamp(dutyCycle, -1, 1)));
    }

    @Override
    public void setSpeedStarWheels(double dutyCycle) {
        starWheels.setControl(new DutyCycleOut(MathUtil.clamp(dutyCycle, -1, 1)));
    }
    

    @Override
    public void runIndexer() {
        // indexer.setControl(new DutyCycleOut(MathUtil.clamp(Tuning.intakeVelocity.get(), -1, 1)));
        indexerUp.setControl(indexerUpVelocityVoltage.withVelocity(80));
        indexerDown.setControl(indexerDownVelocityVoltage.withVelocity(80)); //TODO:DO NOT RUN WITH THIS, IT WIll BREAK
        //indexer.setControl(new VoltageOut(8));
    }

    @Override
    public void runStarWheels(){
        starWheels.setControl(starWheelVelocityVoltage.withVelocity(60));
    }

    @Override
    public void halt() {
        indexerUp.setControl(new DutyCycleOut(0.0));
        indexerDown.setControl(new DutyCycleOut(0.0));
        starWheels.setControl(new DutyCycleOut(0.0));
    }

    @Override
    public void updateInputs(IndexerIOInputs indexerInputs) {
        BaseStatusSignal.refreshAll(mStatusSignals);

        indexerInputs.angularVelocity = indexerUpVelocity.getValue();
        
        indexerInputs.indexerUpAngularVelocity = indexerUpAngularVelocitySignal.getValue();
        indexerInputs.indexerUpStatorCurrent = indexerUpStatorCurrentSignal.getValue();
        indexerInputs.indexerUpSupplyCurrent = indexerUpSupplyCurrentSignal.getValue();
        indexerInputs.indexerDownAngularVelocity = indexerDownAngularVelocitySignal.getValue();
        indexerInputs.indexerDownStatorCurrent = indexerDownStatorCurrentSignal.getValue();
        indexerInputs.indexerDownSupplyCurrent = indexerDownSupplyCurrentSignal.getValue();
        indexerInputs.starWheelsAngularVelocity = starWheelsAngularVelocitySignal.getValue();
        indexerInputs.starWheelsStatorCurrent = starWheelsStatorCurrentSignal.getValue();
        indexerInputs.starWheelsSupplyCurrent = starWheelsSupplyCurrentSignal.getValue();
    }

    @Override
    public void simPeriodic() {}

	public TalonFX getIndexerUp() { return indexerUp; }

	public void setIndexerUp(TalonFX indexerUp) { this.indexerUp = indexerUp; }

	public TalonFX getIndexerDown() { return indexerDown; }

	public void setIndexerDown(TalonFX indexerDown) { this.indexerDown = indexerDown; }

	public TalonFX getStarWheels() { return starWheels; }

	public void setStarWheels(TalonFX starWheels) { this.starWheels = starWheels; }

	public StatusSignal<AngularVelocity> getIndexerUpVelocity() { return indexerUpVelocity; }

	public void setIndexerUpVelocity(StatusSignal<AngularVelocity> indexerUpVelocity) {
		this.indexerUpVelocity = indexerUpVelocity;
	}

	public StatusSignal<AngularVelocity> getIndexerUpAngularVelocitySignal() { return indexerUpAngularVelocitySignal; }

	public StatusSignal<Current> getIndexerUpStatorCurrentSignal() { return indexerUpStatorCurrentSignal; }

	public StatusSignal<Current> getIndexerUpSupplyCurrentSignal() { return indexerUpSupplyCurrentSignal; }

	public StatusSignal<AngularVelocity> getStarWheelsAngularVelocitySignal() { return starWheelsAngularVelocitySignal; }

	public StatusSignal<Current> getStarWheelsStatorCurrentSignal() { return starWheelsStatorCurrentSignal; }

	public StatusSignal<Current> getStarWheelsSupplyCurrentSignal() { return starWheelsSupplyCurrentSignal; }

	public StatusSignal<AngularVelocity> getIndexerDownVelocity() { return indexerDownVelocity; }

	public void setIndexerDownVelocity(StatusSignal<AngularVelocity> indexerDownVelocity) {
		this.indexerDownVelocity = indexerDownVelocity;
	}

	public StatusSignal<AngularVelocity> getIndexerDownAngularVelocitySignal() { return indexerDownAngularVelocitySignal; }

	public StatusSignal<Current> getIndexerDownStatorCurrentSignal() { return indexerDownStatorCurrentSignal; }

	public StatusSignal<Current> getIndexerDownSupplyCurrentSignal() { return indexerDownSupplyCurrentSignal; }

	public List<BaseStatusSignal> getmStatusSignals() { return mStatusSignals; }

	public void setmStatusSignals(List<BaseStatusSignal> mStatusSignals) { this.mStatusSignals = mStatusSignals; }

	public VelocityVoltage getIndexerUpVelocityVoltage() { return indexerUpVelocityVoltage; }

	public VelocityVoltage getIndexerDownVelocityVoltage() { return indexerDownVelocityVoltage; }

	public VelocityVoltage getStarWheelVelocityVoltage() { return starWheelVelocityVoltage; }
}
