package frc.robot.subsystems;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

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

public class ShooterIOReal implements ShooterIO {

    // --------------------- Hardware Interfaces ---------------------
    private final TalonFX flywheelA; // Kraken x60
    private final TalonFX flywheelB; // Kraken x60
    private final TalonFX kicker; // Kraken x44
    private final TalonFX hood; // Kraken x44
    private final CANcoder hoodEncoder;  // Remote Encoder for the hood

    // --------------------- Simulation Interfaces ---------------------
    private DCMotorSim flywheelADCMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Shooter.flywheelGearRatio),
            DCMotor.getKrakenX60(1));
    private DCMotorSim flywheelBDCMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Shooter.flywheelGearRatio),
            DCMotor.getKrakenX60(1));
    private DCMotorSim hoodDCMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.001, Constants.Shooter.hoodGearRatio),
            DCMotor.getKrakenX44(1));
    private DCMotorSim kickerDCMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.001, Constants.Shooter.kickerGearRatio),
            DCMotor.getKrakenX44(1));

    /// Status Signals -- values from the motor(s) to monitor and log
    // Hood signals
    private final StatusSignal<Angle>           hoodAngleSignal;
    private final StatusSignal<AngularVelocity> hoodAngularVelocitySignal;
    private final StatusSignal<Current>         hoodStatorCurrentSignal;
    private final StatusSignal<Current>         hoodSupplyCurrentSignal;

    // Flywheel A signals
    private final StatusSignal<AngularVelocity> flywheelAVelocitySignal;
    private final StatusSignal<Current>         flywheelAStatorCurrentSignal;
    private final StatusSignal<Current>         flywheelASupplyCurrentSignal;

    // Flywheel B signals
    private final StatusSignal<AngularVelocity> flywheelBVelocitySignal;
    private final StatusSignal<Current>         flywheelBStatorCurrentSignal;
    private final StatusSignal<Current>         flywheelBSupplyCurrentSignal;

    // Kicker signals
    private final StatusSignal<AngularVelocity> kickerVelocitySignal;
    private final StatusSignal<Current> kickerStatorCurrentSignal;
    private final StatusSignal<Current> kickerSupplyCurrentSignal;

    // Collection of all status signals
    private List<BaseStatusSignal> mStatusSignals;

    private Angle targetHoodAngle = Units.Degrees.zero();
    private AngularVelocity targetFlywheeVelocity = Units.RotationsPerSecond.zero();

    public ShooterIOReal(final Shooter shooter) {
        this.flywheelA = new TalonFX(Constants.CAN.CTRE.shooterFlywheelA, Constants.CAN.CTRE.bus);
        this.flywheelB = new TalonFX(Constants.CAN.CTRE.shooterFlywheelB, Constants.CAN.CTRE.bus);
        this.kicker = new TalonFX(Constants.CAN.CTRE.kicker, Constants.CAN.CTRE.bus);
        this.hood = new TalonFX(Constants.CAN.CTRE.hood, Constants.CAN.CTRE.bus);

        final Slot0Configs flywheelsSlot0Config =
                new Slot0Configs()
                    .withKP(0.5)
                    .withKS(0.25)
                    .withKV(0.122);
        final Slot0Configs hoodSlot0Config =
                new Slot0Configs()
                    .withKP(25)
                    .withKI(3.5);
        final Slot0Configs kickerSlot0Config =
                new Slot0Configs().withKP(0.1);

        //
        // Flywheels
        //
        final TalonFXConfiguration flywheelsConfig = new TalonFXConfiguration();

        flywheelsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flywheelsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Peak Output Amps
        flywheelsConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        flywheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelsConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        flywheelsConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        // Supply Current Limits
        flywheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelsConfig.CurrentLimits.SupplyCurrentLimit = 60; // max current draw allowed
        flywheelsConfig.CurrentLimits.SupplyCurrentLowerLimit = 35; // current allowed *after* the supply current limit is reached
        flywheelsConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1; // max time allowed to draw SupplyCurrentLimit

        // PID Values
        flywheelsConfig.Slot0 = flywheelsSlot0Config;

        flywheelA.getConfigurator().apply(flywheelsConfig);
        flywheelB.getConfigurator().apply(flywheelsConfig);
        flywheelB.setControl(new Follower(flywheelA.getDeviceID(), MotorAlignmentValue.Aligned));

        //
        // Hood Encoder
        //
        hoodEncoder = new CANcoder(0, Constants.CAN.CTRE.bus);
        CANcoderConfiguration hoodEncoderConfig = new CANcoderConfiguration();
        hoodEncoderConfig.MagnetSensor = new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(0.5)
            .withMagnetOffset(Units.Rotations.of(0.03));
        hoodEncoder.getConfigurator().apply(hoodEncoderConfig);

        //
        // Hood
        //
        final TalonFXConfiguration hoodConfig = new TalonFXConfiguration(); // TODO: Check everything about this

        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Peak Output Amps
        hoodConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        // Supply Current Limits
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 60; // max current draw allowed
        hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = 35; // current allowed *after* the supply current limit is reached
        hoodConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1; // max time allowed to draw SupplyCurrentLimit

        hoodConfig.withFeedback(new FeedbackConfigs()
            .withFeedbackRemoteSensorID(0)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withRotorToSensorRatio(Constants.Shooter.hoodGearRatio/3.6)
            .withSensorToMechanismRatio(3.6));  // TODO: move these to constants please

        // SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
        //     .withForwardSoftLimitEnable(true)
        //     .withForwardSoftLimitThreshold(Units.Degrees.of(50))  /* TODO: 0.4 encoder shaft rotations */
        //     .withReverseSoftLimitEnable(true)
        //     .withReverseSoftLimitThreshold(Units.Degrees.of(0));
        // hoodConfig.withSoftwareLimitSwitch(softLimits);
        // PID Values
        hoodConfig.Slot0 = hoodSlot0Config;

        hood.getConfigurator().apply(hoodConfig);

        //
        // Kicker
        //
        final TalonFXConfiguration kickerConfig = new TalonFXConfiguration(); // TODO: Check everything about this

        kickerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Peak Output Amps
        kickerConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kickerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        kickerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        // Supply Current Limits
        kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kickerConfig.CurrentLimits.SupplyCurrentLimit = 60; // max current draw allowed
        kickerConfig.CurrentLimits.SupplyCurrentLowerLimit = 35; // current allowed *after* the supply current limit is reached
        kickerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1; // max time allowed to draw SupplyCurrentLimit

        // PID Values
        kickerConfig.Slot0 = kickerSlot0Config;

        // Apply all the configs
        hood.getConfigurator().apply(hoodConfig);
        kicker.getConfigurator().apply(kickerConfig);

        // TODO: ideally we'd like to iterate over these instead of having to write this each time...
        this.hoodAngleSignal = this.hood.getPosition();
        this.hoodAngularVelocitySignal = this.hood.getVelocity();
        this.hoodStatorCurrentSignal = this.hood.getStatorCurrent();
        this.hoodSupplyCurrentSignal = this.hood.getSupplyCurrent();
        this.flywheelAVelocitySignal = this.flywheelA.getVelocity();
        this.flywheelAStatorCurrentSignal = this.flywheelA.getStatorCurrent();
        this.flywheelASupplyCurrentSignal = this.flywheelA.getSupplyCurrent();
        this.flywheelBVelocitySignal = this.flywheelB.getVelocity();
        this.flywheelBStatorCurrentSignal = this.flywheelB.getStatorCurrent();
        this.flywheelBSupplyCurrentSignal = this.flywheelB.getSupplyCurrent();
        this.kickerVelocitySignal = this.kicker.getVelocity();
        this.kickerStatorCurrentSignal = this.kicker.getStatorCurrent();
        this.kickerSupplyCurrentSignal = this.kicker.getSupplyCurrent();


        this.mStatusSignals = List.of(
            hoodAngleSignal,
            hoodAngularVelocitySignal,
            hoodStatorCurrentSignal,
            hoodSupplyCurrentSignal,
            flywheelAVelocitySignal,
            flywheelAStatorCurrentSignal,
            flywheelASupplyCurrentSignal,
            flywheelBVelocitySignal,
            flywheelBStatorCurrentSignal,
            flywheelBSupplyCurrentSignal,
            kickerVelocitySignal,
            kickerStatorCurrentSignal,
            kickerSupplyCurrentSignal
        );
    }

    // 5-6 motors max
    //	Flywheel: 2-4 max Kraken x60
    //	Hood: 1 Kraken x44 or Minion
    //	kicker (Moves ball into shooter): Kraken x44

    // Rotates the hood to change angle of fuel shooting
    @Override
    public void rotateHood(Angle hoodAngle) {
        targetHoodAngle = hoodAngle.plus(Units.Degrees.of(angleNudgeDegrees));
        Logger.recordOutput("Shooter/targetHoodAngel", targetHoodAngle) ;
        hood.setControl(new PositionVoltage(targetHoodAngle));
    }

    // Runs the flywheel in the shooter. 2 motors. Based on voltage
    @Override
    public void runFlywheels() {
        this.flywheelA.setControl(new VoltageOut(9));
    }

    @Override
    public void stopFlyWheels() {
        this.flywheelA.setControl(new VoltageOut(0));
    }

    // Runs the flywheel in the shooter. 2 motors. Based on velocity
    @Override
    public void runFlywheelsVelocity(AngularVelocity speed) {
        this.targetFlywheeVelocity = speed.plus(Units.RotationsPerSecond.of(speedNudgeRPS));
        Logger.recordOutput("Shooter/flywheelVelocityNudge", Units.RotationsPerSecond.of(speedNudgeRPS));
        Logger.recordOutput("Shooter/targetFlywheelVelocity", targetFlywheeVelocity);
        this.flywheelA.setControl(new VelocityVoltage(targetFlywheeVelocity));
    }

    // Runs the kicker. Shoots ball into flywheels.
    @Override
    public void runKicker(Voltage kickerVoltage) {
        this.kicker.setControl(new VoltageOut(kickerVoltage));
    }

    @Override
    public void simPeriodic() {
        System.out.println("Shooter Being Simulated");

        TalonFXSimState flywheelASimState = flywheelA.getSimState();
        TalonFXSimState flywheelBSimState = flywheelB.getSimState();
        TalonFXSimState kickerSimState = kicker.getSimState();
        TalonFXSimState hoodSimState = hood.getSimState();

        flywheelASimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        flywheelBSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        kickerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        hoodSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        Voltage flywheelAVoltage = flywheelASimState.getMotorVoltageMeasure();
        flywheelADCMotorSim.setInputVoltage(addFriction(flywheelAVoltage.in(Units.Volts), 0.2));
        Logger.recordOutput("Shooter/FlywheelAVoltage", flywheelAVoltage.in(Units.Volts));

        Voltage flywheelBVoltage = flywheelBSimState.getMotorVoltageMeasure();
        flywheelBDCMotorSim.setInputVoltage(addFriction(flywheelBVoltage.in(Units.Volts), 0.2));
        Logger.recordOutput("Shooter/FlywheelBVoltage", flywheelBVoltage.in(Units.Volts));

        Voltage hoodVoltage = hoodSimState.getMotorVoltageMeasure();
        hoodDCMotorSim.setInputVoltage(addFriction(hoodVoltage.in(Units.Volts), 0.2));
        Logger.recordOutput("Shooter/HoodVoltage", hoodVoltage.in(Units.Volts));

        Voltage kickerVoltage = kickerSimState.getMotorVoltageMeasure();
        kickerDCMotorSim.setInputVoltage(addFriction(kickerVoltage.in(Units.Volts), 0.2));
        Logger.recordOutput("Shooter/KickerVoltage", kickerVoltage.in(Units.Volts));

        flywheelADCMotorSim.update(0.02);
        flywheelBDCMotorSim.update(0.02);
        hoodDCMotorSim.update(0.02);
        kickerDCMotorSim.update(0.02);

        flywheelASimState.setRawRotorPosition(
                flywheelADCMotorSim.getAngularPosition().times(Constants.Shooter.flywheelGearRatio));
        flywheelASimState.setRotorVelocity(
                flywheelADCMotorSim.getAngularVelocity().times(Constants.Shooter.flywheelGearRatio));

        flywheelBSimState.setRawRotorPosition(
                flywheelBDCMotorSim.getAngularPosition().times(Constants.Shooter.flywheelGearRatio));
        flywheelBSimState.setRotorVelocity(
                flywheelBDCMotorSim.getAngularVelocity().times(Constants.Shooter.flywheelGearRatio));

        hoodSimState.setRawRotorPosition(hoodDCMotorSim.getAngularPosition().times(Constants.Shooter.hoodGearRatio));
        hoodSimState.setRotorVelocity(hoodDCMotorSim.getAngularVelocity().times(Constants.Shooter.hoodGearRatio));

        kickerSimState.setRawRotorPosition(
                kickerDCMotorSim.getAngularPosition().times(Constants.Shooter.kickerGearRatio));
        kickerSimState.setRotorVelocity(kickerDCMotorSim.getAngularVelocity().times(Constants.Shooter.kickerGearRatio));
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

    // --------------------- Nudge State ---------------------
    private double angleNudgeDegrees = 0.0;
    private double speedNudgeRPS = 0.0;

    @Override
    public void nudgeAngleUp() {
        this.angleNudgeDegrees += 2.0; // Degrees
    }

    @Override
    public void nudgeAngleDown() {
        this.angleNudgeDegrees -= 2.0; // Degrees
    }

    @Override
    public void nudgeSpeedUp() {
        this.speedNudgeRPS += 2.0;
    }

    @Override
    public void nudgeSpeedDown() {
        this.speedNudgeRPS -= 2.0;
    }

    @Override
    public void resetNudges() {
        this.angleNudgeDegrees = 0.0;
        this.speedNudgeRPS = 0.0;
    }

    @Override
    public void updateInputs(final ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(mStatusSignals);

        // all signals should have been refreshed via the list -- update the inputs now
        // hood signals
        inputs.hoodAngle = this.hoodAngleSignal.getValue();
        var isHoodAngleInTolerance = inputs.hoodAngle.isNear(this.targetHoodAngle, Constants.Shooter.hoodAngleTolerance);
        inputs.hoodAngleInTolerance = isHoodAngleInTolerance;
        inputs.targetHoodAngle = targetHoodAngle;
        inputs.hoodAngularVelocity = hoodAngularVelocitySignal.getValue();
        inputs.hoodStatorCurrent = hoodStatorCurrentSignal.getValue();
        inputs.hoodSupplyCurrent = hoodSupplyCurrentSignal.getValue();

        // Flywheel A Signals
        inputs.shooterAVelocity = this.flywheelAVelocitySignal.getValue();
        inputs.shooterAStatorCurrent = this.flywheelAStatorCurrentSignal.getValue();
        inputs.shooterASupplyCurrent = this.flywheelASupplyCurrentSignal.getValue();
        inputs.targetFlywheelVelocity = targetFlywheeVelocity;
        var isFlywheelSpeedInTolerance = inputs.shooterAVelocity.isNear(this.targetFlywheeVelocity, Constants.Shooter.shooterVelocityTolerance);
        inputs.flywheelsInTolerance = isFlywheelSpeedInTolerance;

        // Flywheel B signals
        inputs.shooterBVelocity = this.flywheelBVelocitySignal.getValue();
        inputs.shooterBStatorCurrent = this.flywheelBStatorCurrentSignal.getValue();
        inputs.shooterBSupplyCurrent = this.flywheelBSupplyCurrentSignal.getValue();

        // Kicker Signals
        inputs.kickerVelocity = this.kickerVelocitySignal.getValue();
        inputs.kickerStatorCurrent = this.kickerStatorCurrentSignal.getValue();
        inputs.kickerSupplyCurrent = this.kickerSupplyCurrentSignal.getValue();
    }
}
