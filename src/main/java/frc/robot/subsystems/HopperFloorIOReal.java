package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;
import frc.robot.Tuning;
import frc.robot.subsystems.ShooterIO.ShooterIOInputs;

import java.util.List;

import org.littletonrobotics.junction.Logger;

public class HopperFloorIOReal implements HopperFloorIO {
    private TalonFX hopper;

    private final StatusSignal<AngularVelocity> hopperAngularVelocitySignal;
    private final StatusSignal<Current>         hopperStatorCurrentSignal;
    private final StatusSignal<Current>         hopperSupplyCurrentSignal;

    final VelocityVoltage hopperVelocityVoltage;

    // Collection of all status signals
    private List<BaseStatusSignal> mStatusSignals;

    // --------------------Simulation----------------------
    private DCMotorSim floorDCMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.HopperFloor.indexerGearRatio),
            DCMotor.getKrakenX60(1));

    public HopperFloorIOReal() {
        // TODO: change CAN ID
        this.hopper = new TalonFX(Constants.CAN.CTRE.hopper, Constants.CAN.CTRE.bus);

        TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        hopperConfig.CurrentLimits = currentLimitsConfigs;

        currentLimitsConfigs.StatorCurrentLimit = 40; // the peak current, in amps

        //TODO: actually tune these PID's
        Slot0Configs hopperFloorSlot0Configs = new Slot0Configs();
        hopperFloorSlot0Configs.kS = 0.7; // Add 0.1 V output to overcome static friction
        hopperFloorSlot0Configs.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
        hopperFloorSlot0Configs.kP = 0.4; // An error of 1 rps results in 0.11 V output
        hopperFloorSlot0Configs.kI = 0; // no output for integrated error
        hopperFloorSlot0Configs.kD = 0; // no output for error derivative

        // PID Values
        hopperConfig.Slot0 = hopperFloorSlot0Configs;

        hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        hopperConfig.CurrentLimits.SupplyCurrentLimit = 55.0;
        hopperConfig.CurrentLimits.StatorCurrentLimit = 120.0;

        hopper.getConfigurator().apply(hopperConfig);

        this.hopperAngularVelocitySignal = this.hopper.getVelocity();
        this.hopperStatorCurrentSignal = this.hopper.getStatorCurrent();
        this.hopperSupplyCurrentSignal = this.hopper.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100, hopperAngularVelocitySignal, hopperStatorCurrentSignal, hopperSupplyCurrentSignal);

        this.mStatusSignals = List.of(
            hopperAngularVelocitySignal,
            hopperStatorCurrentSignal,
            hopperSupplyCurrentSignal
        );

        // create a velocity closed-loop request, voltage output, slot 0 configs
        this.hopperVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    }

    @Override
    public void setSpeed(double angularVelocity) {
        // Do a feed forward later
        //hopper.setControl(new DutyCycleOut(MathUtil.clamp(angularVelocity, -1, 1)));
        // set velocity to 8 rps, add 0.5 V to overcome gravity
        hopper.setControl(this.hopperVelocityVoltage.withVelocity(angularVelocity));
        Logger.recordOutput("HopperFloorIOReal/setSpeed", angularVelocity);
    }

    @Override
    public void runHopper() {
        // hopper.setControl(new DutyCycleOut(MathUtil.clamp(Tuning.hopperVelocity.get(), -1, 1)));
        //hopper.setControl(new VoltageOut(Units.Volts.of(5)));
        // set velocity to 8 rps, add 0.5 V to overcome gravity
        hopper.setControl(hopperVelocityVoltage.withVelocity(45));
        Logger.recordOutput("HopperFloorIOReal/runHopper", Tuning.hopperVelocity.get());
    }

    @Override
    public void runHopperReverse() {
        hopper.setControl(new VoltageOut(Units.Volts.of(-5)));
    }

    @Override
    public void halt() {
        hopper.setControl(new DutyCycleOut(0.0));
    }

    @Override
    public void updateInputs(HopperFloorIOInputs hopperInputs) {
        BaseStatusSignal.refreshAll(mStatusSignals);

        hopperInputs.hopperAngularVelocity = hopperAngularVelocitySignal.getValue();
        hopperInputs.hopperStatorCurrent = hopperStatorCurrentSignal.getValue();
        hopperInputs.hopperSupplyCurrent = hopperSupplyCurrentSignal.getValue();
    }

    @Override
    public void simPeriodic() {
        // Fetch TalonFXSimState for each motor controller
        TalonFXSimState hopperFloorSimState = hopper.getSimState();
        // Set supply voltage to motor controllers
        hopperFloorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        // Get voltage output from motor controllers and set input voltage to the motor sims
        Voltage hopperFloorVoltage = hopperFloorSimState.getMotorVoltageMeasure();
        // Not sure how to make this work
        floorDCMotorSim.setInputVoltage(addFriction(hopperFloorVoltage.in(Units.Volts), 0.2));
        // Update motor sim with time step.
        floorDCMotorSim.update(0.02);
        // Feed DC motor sim position and velocity back to motor controller sim state
        hopperFloorSimState.setRawRotorPosition(
                floorDCMotorSim.getAngularPosition().times(Constants.HopperFloor.indexerGearRatio));
        hopperFloorSimState.setRotorVelocity(
                floorDCMotorSim.getAngularVelocity().times(Constants.HopperFloor.indexerGearRatio));
        Logger.recordOutput("HopperFloorIOReal/Sim running", true);
        Logger.recordOutput("HopperFloorVoltage", hopperFloorVoltage);
    }
    /**
     * Applies the effects of friction to dampen the motor voltage.
     *
     * @param motorVoltage Voltage output by the motor
     * @param frictionVoltage Voltage required to overcome friction
     * @return Friction-dampened motor voltage
     */
    protected double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }
}
