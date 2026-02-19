package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
    private TalonFX intakeRollerMotor;
    private TalonFX intakeExpansionMotor;
    public StatusSignal<AngularVelocity> intakeAngularVelocity;
    public StatusSignal<Angle> expansionMotorAngle;
    private PositionTorqueCurrentFOC retractPositionTorqueCurrentFOC;
    private PositionVoltage expansionPositionVoltage;
    private final Angle closedAngle = Units.Rotations.of(0);
    private final Angle openAngle = Units.Rotations.of(10);

    // For Simualtion
    private DCMotorSim expansionDCMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1), 0.001, Constants.Intake.expensionMotorGearRatio),
            DCMotor.getKrakenX60(1));

    // Data goten at 02/11/26 Wednesday
    // Hopper extemsion: 11 iches and 3 quarters
    // Gear ratio: 3 to 1

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
        // todo: config this mototr so that it turn off when limit switches.
        CurrentLimitsConfigs intakeExpansionCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Units.Amps.of(80))
                .withSupplyCurrentLimit(Units.Amps.of(60))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        // TODO determine correct currect limit switch values
        HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs()
                /* .withForwardLimitEnable(true)
                .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin)
                .withForwardLimitRemoteSensorID(Constants.CAN.CTRE.intakeSensor)
                .withReverseLimitEnable(true)
                .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin)
                .withReverseLimitRemoteSensorID(Constants.CAN.CTRE.intakeSensor) */

                // TODO: Check if these configs are correct
                .withForwardLimitRemoteCANdiS1(Constants.CAN.INTAKE_CANDI.getInstance())
                .withForwardLimitEnable(true)
                .withForwardLimitAutosetPositionEnable(true)
                .withForwardLimitAutosetPositionValue(Units.Degrees.of(0))
                .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
                .withReverseLimitRemoteCANdiS1(Constants.CAN.INTAKE_CANDI.getInstance())
                .withReverseLimitEnable(true)
                .withReverseLimitAutosetPositionEnable(true)
                .withReverseLimitAutosetPositionValue(Units.Degrees.of(0))
                .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen);

        intakeExpansionConfig
                .SoftwareLimitSwitch
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Units.Rotations.of(100)) // Chnage this software limit to fit later
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Units.Rotations.of(-100)); // Chnage this software limit to fit later

        intakeExpansionConfig.Feedback.withSensorToMechanismRatio(3.0); // May change later reduction gear ratio

        MotorOutputConfigs intakeExpansionOutputConfigs =
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
        intakeExpansionConfig
                .withMotorOutput(intakeExpansionOutputConfigs)
                .withCurrentLimits(intakeExpansionCurrentLimitsConfigs)
                .withHardwareLimitSwitch(hardwareLimitSwitchConfigs);

        intakeExpansionMotor.getConfigurator().apply(intakeExpansionConfig); // apply the config settings

        this.intakeAngularVelocity = this.intakeRollerMotor.getRotorVelocity();
        this.expansionMotorAngle = this.intakeExpansionMotor.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(Units.Hertz.of(100), intakeAngularVelocity, expansionMotorAngle);

        retractPositionTorqueCurrentFOC = new PositionTorqueCurrentFOC(Units.Rotation.of(0))
                .withSlot(0)
                .withLimitReverseMotion(true)
                .withLimitForwardMotion(true);

        expansionPositionVoltage = new PositionVoltage(openAngle)
                .withSlot(1)
                .withLimitReverseMotion(true)
                .withLimitForwardMotion(true);

        if (Constants.mode == Constants.Mode.SIM) {
            TalonFXSimState simState = intakeExpansionMotor.getSimState();
            simState.Orientation = ChassisReference.CounterClockwise_Positive;
            simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    @Override
    public void setSpeed(double speed) {
        // Do a feed forward later
        intakeRollerMotor.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void extend() {
        // Expand and stop once fully expanded
        // intakeExpansionMotor.setControl(expansionPositionVoltage.withPosition(openAngle));
        Logger.recordOutput("Intake/Is extend called", true);
        intakeExpansionMotor.setControl(new DutyCycleOut(8));
    }

    @Override
    public void retract() {
        // Retract using the torque control
        intakeExpansionMotor.setControl(retractPositionTorqueCurrentFOC.withPosition(closedAngle));
    }

    @Override
    public void updateInputs(IntakeInputs intakeInputs) {
        BaseStatusSignal.refreshAll(intakeAngularVelocity, expansionMotorAngle);
        intakeInputs.angularVelocity = intakeAngularVelocity.getValue();
        intakeInputs.expansionMotorAngle =
                Units.Inches.of(expansionMotorAngle.getValue().in(Units.Rotations));
    }

    @Override
    public void simPeriodic() {
        Logger.recordOutput("Inake/simPeriodic called", true);
        TalonFXSimState simState = intakeExpansionMotor.getSimState();

        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        Logger.recordOutput("Intake/battery voltage", RobotController.getBatteryVoltage());

        Voltage motorVoltage = simState.getMotorVoltageMeasure();
        Logger.recordOutput("Intake/voltagePassed", motorVoltage);

        expansionDCMotorSim.setInputVoltage(motorVoltage.in(Units.Volts));
        expansionDCMotorSim.update(0.02);

        simState.setRawRotorPosition(
                expansionDCMotorSim.getAngularPosition().times(Constants.Intake.expensionMotorGearRatio));
        simState.setRotorVelocity(
                expansionDCMotorSim.getAngularVelocity().times(Constants.Intake.expensionMotorGearRatio));
        Logger.recordOutput("Intake/Sim Motor Velocity", expansionDCMotorSim.getAngularVelocity());
    }
}
