package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {

    // --------------------- Hardware Interfaces ---------------------
    private final TalonFX flywheelA; // Kraken x60
    private final TalonFX flywheelB; // Kraken x60
    private final TalonFX uptake; // Kraken x44
    private final TalonFX hood; // Kraken x44

    private StatusSignal<Angle> hoodAngle;
    private StatusSignal<AngularVelocity> velocityA;
    private StatusSignal<AngularVelocity> velocityB;

    public ShooterIOReal(final Shooter shooter) {
        this.flywheelA = new TalonFX(Constants.CAN.CTRE.shooterFlywheelA, Constants.CAN.CTRE.bus);
        this.flywheelB = new TalonFX(Constants.CAN.CTRE.shooterFlywheelB, Constants.CAN.CTRE.bus);
        this.uptake = new TalonFX(Constants.CAN.CTRE.uptake, Constants.CAN.CTRE.bus);
        this.hood = new TalonFX(Constants.CAN.CTRE.hood, Constants.CAN.CTRE.bus);

        //
        // Flywheels
        //
        final TalonFXConfiguration flywheelsConfig = new TalonFXConfiguration();

        flywheelsConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flywheelsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Peak Output Amps
        flywheelsConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        flywheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelsConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        flywheelsConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        // Supply Current Limits
        flywheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelsConfig.CurrentLimits.SupplyCurrentLimit = 60; // max current draw allowed
        flywheelsConfig.CurrentLimits.SupplyCurrentLowerLimit =
                35; // current allowed *after* the supply current limit is reached
        flywheelsConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1; // max time allowed to draw SupplyCurrentLimit

        // PID Values
        flywheelsConfig.Slot0 = Constants.Shooter.flywheelGainsSlot0;

        flywheelA.getConfigurator().apply(flywheelsConfig);
        flywheelB.getConfigurator().apply(flywheelsConfig);
        flywheelB.setControl(new Follower(flywheelA.getDeviceID(), MotorAlignmentValue.Opposed));

        //
        // Hood
        //
        final TalonFXConfiguration hoodConfig = new TalonFXConfiguration(); // TODO: Check everything about this

        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Peak Output Amps
        hoodConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        // Supply Current Limits
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 60; // max current draw allowed
        hoodConfig.CurrentLimits.SupplyCurrentLowerLimit =
                35; // current allowed *after* the supply current limit is reached
        hoodConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1; // max time allowed to draw SupplyCurrentLimit

        // PID Values
        hoodConfig.Slot0 = Constants.Shooter.hoodGainsSlot0;


        //
        // Kicker
        //
        final TalonFXConfiguration kickerConfig = new TalonFXConfiguration(); // TODO: Check everything about this

        kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Peak Output Amps
        kickerConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kickerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        kickerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        // Supply Current Limits
        kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kickerConfig.CurrentLimits.SupplyCurrentLimit = 60; // max current draw allowed
        kickerConfig.CurrentLimits.SupplyCurrentLowerLimit =
                35; // current allowed *after* the supply current limit is reached
        kickerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1; // max time allowed to draw SupplyCurrentLimit

        // PID Values
        kickerConfig.Slot0 = Constants.Shooter.kickerGainsSlot0;


        this.velocityA = this.flywheelA.getRotorVelocity();
        this.velocityB = this.flywheelB.getRotorVelocity();
        this.hoodAngle = this.hood.getPosition();
    }

    // 5-6 motors max
    //	Flywheel: 2-4 max Kraken x60
    //	Hood: 1 Kraken x44 or Minion
    //	Uptake (Moves ball into shooter): Kraken x44

    // Rotates the hood to change angle of fuel shooting
    public void rotateHood(Angle hoodAngle) {
        //this.hood.setControl(new PositionVoltage(hoodAngle));
    }

    // Runs the flywheel in the shooter. 2 motors. Based on voltage
    public void runFlywheels() {
        this.flywheelA.setControl(new VoltageOut(9));
    }

    // Runs the flywheel in the shooter. 2 motors. Based on velocity
    public void runFlywheelsVelocity(AngularVelocity speed) {
        this.flywheelA.setControl(new VelocityVoltage(speed));
    }

    // Runs the kicker/uptake. Shoots ball into flywheels.
    public void runUptake() {
        this.uptake.setControl(new VoltageOut(9));
    }

    @Override
    public void updateInputs(final ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(this.hoodAngle, this.velocityA, this.velocityB);
        inputs.flywheelSpeedA = Units.RotationsPerSecond.of(this.velocityA.getValueAsDouble());
        inputs.flywheelSpeedB = Units.RotationsPerSecond.of(this.velocityB.getValueAsDouble());
        inputs.hoodAngle = Units.Rotation.of(this.hoodAngle.getValueAsDouble());
    }
}
