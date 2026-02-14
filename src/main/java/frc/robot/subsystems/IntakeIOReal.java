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

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    private TalonFX intakeRollerMotor;
    private TalonFX intakeExpansionMotor;
    public StatusSignal<AngularVelocity> intakeAngularVelocity;
    private PositionTorqueCurrentFOC retractPositionTorqueCurrentFOC;
    private PositionVoltage expansionPositionVoltage;
    private final Angle closedAngle = Units.Rotations.of(0);
    private final Angle openAngle = Units.Rotations.of(10);

    // Data goten at 02/11/26 Wednesday
    // Hopper extemsion: 11 iches and 3 quarters
    // Gear ratio: 1 to 3

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
                .withReverseSoftLimitThreshold(Units.Rotations.of(100)); // Chnage this software limit to fit later

        MotorOutputConfigs intakeExpansionOutputConfigs =
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
        intakeExpansionConfig
                .withMotorOutput(intakeExpansionOutputConfigs)
                .withCurrentLimits(intakeExpansionCurrentLimitsConfigs)
                .withHardwareLimitSwitch(hardwareLimitSwitchConfigs);
        intakeExpansionMotor.getConfigurator().apply(intakeExpansionConfig); // apply the config settings

        this.intakeAngularVelocity = this.intakeRollerMotor.getRotorVelocity();
        BaseStatusSignal.setUpdateFrequencyForAll(Units.Hertz.of(100), intakeAngularVelocity);

        retractPositionTorqueCurrentFOC = new PositionTorqueCurrentFOC(Units.Rotation.of(0))
                .withSlot(0)
                .withLimitReverseMotion(true)
                .withLimitForwardMotion(true);

        expansionPositionVoltage = new PositionVoltage(openAngle)
                .withSlot(1)
                .withLimitReverseMotion(true)
                .withLimitForwardMotion(true);
    }

    @Override
    public void setSpeed(double speed) {
        // Do a feed forward later
        intakeRollerMotor.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void expand() {
        // Expand and stop once fully expanded
        intakeExpansionMotor.setControl(expansionPositionVoltage.withPosition(openAngle));
    }

    @Override
    public void retract() {
        // Retract using the torque control
        intakeExpansionMotor.setControl(retractPositionTorqueCurrentFOC.withPosition(closedAngle));
    }

    @Override
    public void updateInputs(IntakeInputs intakeInputs) {
        BaseStatusSignal.refreshAll(intakeAngularVelocity);
        intakeInputs.angularVelocity = intakeAngularVelocity.getValue();
    }
}
