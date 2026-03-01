package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;
import frc.robot.Tuning;

public class HopperFloorIOReal implements HopperFloorIO {
    private TalonFX hopper;
    // --------------------Simulation----------------------
    private DCMotorSim floorDCMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.HopperFloor.indexerGearRatio),
            DCMotor.getKrakenX60(1));

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
    public void setSpeed(double angularVelocity) {
        // Do a feed forward later
        hopper.setControl(new DutyCycleOut(MathUtil.clamp(angularVelocity, -1, 1)));
        Logger.recordOutput("HopperFloorIOReal/setSpeed",angularVelocity);
    }

    @Override
    public void runHopper() {
        hopper.setControl(new DutyCycleOut(MathUtil.clamp(Tuning.hopperVelocity.get(), -1, 1)));
        Logger.recordOutput("HopperFloorIOReal/runHopper", Tuning.hopperVelocity.get());
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
