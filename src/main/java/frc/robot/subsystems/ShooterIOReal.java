package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
	public ShooterIOReal(final Shooter shooter) {
		this.velocityA = this.flywheelA.getRotorVelocity();
		this.velocityB = this.flywheelB.getRotorVelocity();
		this.hoodAngle = this.hood.getPosition();
	}

	// 5-6 motors max
	//	Flywheel: 2-4 max Kraken x60
	//	Hood: 1 Kraken x44 or Minion
	//	Uptake (Moves ball into shooter): Kraken x44
	
	private final TalonFX flywheelA = new TalonFX(Constants.CAN.CTRE.shooterFlywheelA, Constants.CAN.CTRE.bus); // Kraken x60
	private final TalonFX flywheelB = new TalonFX(Constants.CAN.CTRE.shooterFlywheelB, Constants.CAN.CTRE.bus); // Kraken x60
	private final TalonFX uptake = new TalonFX(Constants.CAN.CTRE.uptake, Constants.CAN.CTRE.bus); // Kraken x44 // TODO: Determine if this is a TalonFX or TalonSRX
	private final TalonFX hood = new TalonFX(Constants.CAN.CTRE.hood, Constants.CAN.CTRE.bus); // Kraken x44

	private StatusSignal<Angle> hoodAngle;
	private StatusSignal<AngularVelocity> velocityA;
	private StatusSignal<AngularVelocity> velocityB;

	// Rotates the hood to change angle of fuel shooting
	public void rotateHood() {
		this.hood.setControl(new VoltageOut(9));
	}
	
	// Runs the flywheel in the shooter. 2 motors. Based on voltage
	public void runFlywheels() {
		this.flywheelA.setControl(new VoltageOut(9));
		this.flywheelB.setControl(new VoltageOut(9));
	}

	// Runs the flywheel in the shooter. 2 motors. Based on velocity
	public void runFlywheelsVelocity() {
		this.flywheelA.setControl(new VelocityDutyCycle(4));
		this.flywheelB.setControl(new VelocityDutyCycle(4));
	}

	// Runs the kicker/uptake. Shoots ball into flywheels.
	public void runUptake() {
		this.uptake.setControl(new VoltageOut(9));
	}

	@Override
	public void updateInputs(final ShooterIOInputs inputs) {
		inputs.flywheelSpeedA = Units.RotationsPerSecond.of(this.velocityA.getValueAsDouble());
		inputs.flywheelSpeedB = Units.RotationsPerSecond.of(this.velocityB.getValueAsDouble());
		inputs.hoodAngle = Units.Rotation.of(this.hoodAngle.getValueAsDouble());
	}
}
