package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

public class IntakeIOReal implements IntakeIO {
    private TalonFX intakeRollerMotor;
    private TalonFX intakeExpansionMotor;
    public StatusSignal<AngularVelocity> intakeAngularVelocity;
    public StatusSignal<AngularVelocity> expansionAngularVelocity;
    public StatusSignal<Angle> expansionMotorAngle;
    private PositionVoltage retractPositionControl;
    private final MotionMagicVoltage motionMagicVoltage;
    private PositionVoltage expansionPositionVoltage;
    private final Angle closedAngle = Units.Rotations.of(0);
    private final Angle openAngle = Units.Rotations.of(11.75);

    // For Simualtion
    private DCMotorSim expansionDCMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1), 0.001, Constants.Intake.expensionMotorGearRatio),
            DCMotor.getKrakenX60(1));

    private DCMotorSim rollerDCMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Intake.rollerMotorGearRatio),
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

        final Slot0Configs retractSlot0Configs = new Slot0Configs();
        retractSlot0Configs.kS = 0.2; // Add 0.2 V output to overcome static friction
        retractSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        retractSlot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        retractSlot0Configs.kI = 0; // no output for integrated error
        retractSlot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        final Slot1Configs extendSlot1Configs =
                new Slot1Configs().withKP(60).withKI(0).withKD(3);

        final TalonFXConfiguration intakeExpansionConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs intakeExpansionCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Units.Amps.of(80))
                .withSupplyCurrentLimit(Units.Amps.of(60))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        // TODO determine correct currect limit switch values
        HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs()
                // Forward Limits
                .withForwardLimitRemoteCANdiS1(Constants.CAN.INTAKE_CANDI.getInstance())
                .withForwardLimitEnable(true)
                .withForwardLimitAutosetPositionEnable(true)
                .withForwardLimitAutosetPositionValue(Units.Degrees.of(0))
                .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
                // Reverse Limits
                .withReverseLimitRemoteCANdiS1(Constants.CAN.INTAKE_CANDI.getInstance())
                .withReverseLimitEnable(true)
                .withReverseLimitAutosetPositionEnable(true)
                .withReverseLimitAutosetPositionValue(Units.Rotations.of(0))
                .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen);

        intakeExpansionConfig
                .SoftwareLimitSwitch
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(openAngle) // Chnage this software limit to fit later
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(closedAngle); // Chnage this software limit to fit later

        intakeExpansionConfig.Feedback.withSensorToMechanismRatio(
                Constants.Intake.expensionMotorGearRatio); // May change later reduction gear ratio

        // Motion Magic Configs
        var motionMagicConfigs = intakeExpansionConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 1; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        MotorOutputConfigs intakeExpansionOutputConfigs =
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
        intakeExpansionConfig
                .withMotorOutput(intakeExpansionOutputConfigs)
                .withCurrentLimits(intakeExpansionCurrentLimitsConfigs)
                .withHardwareLimitSwitch(hardwareLimitSwitchConfigs)
                .withSlot0(retractSlot0Configs)
                .withSlot1(extendSlot1Configs);

        intakeExpansionMotor.getConfigurator().apply(intakeExpansionConfig); // apply the config settings

        // Status Signals
        this.intakeAngularVelocity = this.intakeRollerMotor.getRotorVelocity();
        this.expansionMotorAngle = this.intakeExpansionMotor.getPosition();
        this.expansionAngularVelocity = this.intakeExpansionMotor.getRotorVelocity();
        BaseStatusSignal.setUpdateFrequencyForAll(
                Units.Hertz.of(100), intakeAngularVelocity, expansionMotorAngle, expansionAngularVelocity);

        // Retract Position voltage control
        this.retractPositionControl = new PositionVoltage(Units.Rotations.zero()).withSlot(0);

        // Create a Motion Magic request, voltage output
        this.motionMagicVoltage = new MotionMagicVoltage(0);

        // Expansion Position voltage control
        this.expansionPositionVoltage = new PositionVoltage(openAngle).withSlot(1);

        if (Constants.mode == Constants.Mode.SIM) {
            TalonFXSimState simState = intakeExpansionMotor.getSimState();
            simState.Orientation = ChassisReference.CounterClockwise_Positive;
            simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    @Override
    public void setState(Constants.Intake.IntakeStates state) {
        // Do a feed forward later
        intakeRollerMotor.setControl(new DutyCycleOut(state.getSpeed()));
    }

    @Override
    public void extend() {
        intakeExpansionMotor.setControl(expansionPositionVoltage.withPosition(openAngle));
    }

    @Override
    public void retract() {
        intakeExpansionMotor.setControl(motionMagicVoltage.withPosition(closedAngle));
        /*
        intakeExpansionMotor.setControl(
                retractPositionTorqueCurrentFOC.withPosition(closedAngle).withFeedForward(0)); */
    }

    @Override
    public void updateInputs(IntakeInputs intakeInputs) {
        BaseStatusSignal.refreshAll(intakeAngularVelocity, expansionMotorAngle, expansionAngularVelocity);
        intakeInputs.angularVelocity = intakeAngularVelocity.getValue();
        intakeInputs.expansionMotorAngle =
                Units.Inches.of(expansionMotorAngle.getValue().in(Units.Rotations));
        intakeInputs.expansionAngularVelocity =
                Units.InchesPerSecond.of(expansionAngularVelocity.getValue().in(Units.RotationsPerSecond));
    }

    @Override
    public void simPeriodic() {
        // 1) Fetch the TalonFXSimState for each motor controller. Sim state will
        //      provide output voltage to the motor and we will update it with simulated motor pos/vel
        TalonFXSimState expansionMotorSimState = intakeExpansionMotor.getSimState();
        TalonFXSimState rollerMotorSimState = intakeRollerMotor.getSimState();

        // 2) Set supply voltage to the motor controllers (this can simulate battery load/brownout)
        expansionMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        rollerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // 3) Get voltage output from motor controllers, and set input voltage to the motor sims
        Voltage expansionMotorVoltage = expansionMotorSimState.getMotorVoltageMeasure();
        expansionDCMotorSim.setInputVoltage(addFriction(expansionMotorVoltage.in(Units.Volts), 0.2));

        Voltage rollerMotorVoltage = rollerMotorSimState.getMotorVoltageMeasure();
        rollerDCMotorSim.setInputVoltage(addFriction(rollerMotorVoltage.in(Units.Volts), 0.2));

        // 4) Update motor sim with a time step. This will have the motor sim move based on input voltage and give us
        // back the new motor position
        expansionDCMotorSim.update(0.02);
        rollerDCMotorSim.update(0.02);

        // 5) Feed DC motor sim position and velocity back to the motor controller sim state to complete the sim loop
        expansionMotorSimState.setRawRotorPosition(
                expansionDCMotorSim.getAngularPosition().times(Constants.Intake.expensionMotorGearRatio));
        expansionMotorSimState.setRotorVelocity(
                expansionDCMotorSim.getAngularVelocity().times(Constants.Intake.expensionMotorGearRatio));

        rollerMotorSimState.setRawRotorPosition(
                rollerDCMotorSim.getAngularPosition().times(Constants.Intake.rollerMotorGearRatio));
        rollerMotorSimState.setRotorVelocity(
                rollerDCMotorSim.getAngularVelocity().times(Constants.Intake.rollerMotorGearRatio));

        // 5) Set the Limit Swiches
        expansionMotorSimState.setReverseLimit(
                expansionDCMotorSim.getAngularPosition().lte(Units.Rotations.zero()));
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
