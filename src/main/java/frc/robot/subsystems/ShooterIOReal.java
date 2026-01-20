package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
	public ShooterIOReal(final Shooter shooter) {
		this.velocityA = this.flywheelA.getRotorVelocity();
		this.velocityB = this.flywheelB.getRotorVelocity();

		SmartDashboard.putData("SysId", );
	}

	// 5-6 motors max
	//	Flywheel: 2-4 max Kraken x60
	//	Hood: 1 Kraken x44 or Minion
	//	Uptake (Moves ball into shooter): Kraken x44
	//private Talon

	
	private final TalonFX flywheelA = new TalonFX(Constants.CAN.CTRE.shooterFlywheelA, Constants.CAN.CTRE.bus);
	private final TalonFX flywheelB = new TalonFX(Constants.CAN.CTRE.shooterFlywheelB, Constants.CAN.CTRE.bus);
	private final TalonFX uptake = new TalonFX(Constants.CAN.CTRE.uptake, Constants.CAN.CTRE.bus); // TODO: Determine if this is a TalonFX or TalonSRX
	private final TalonFX hood = new TalonFX(Constants.CAN.CTRE.hood, Constants.CAN.CTRE.bus);

	public final StatusSignal<Angle> hoodAngle;
	public final StatusSignal<AngularVelocity> velocityA;
	public final StatusSignal<AngularVelocity> velocityB;




	@Override
	public void updateInputs(final ShooterIOInputs inputs) {
		inputs.flywheelSpeedA = Units.RotationsPerSecond.of(this.velocityA.getValueAsDouble());
		inputs.flywheelSpeedB = Units.RotationsPerSecond.of(this.velocityB.getValueAsDouble());
	}
}
