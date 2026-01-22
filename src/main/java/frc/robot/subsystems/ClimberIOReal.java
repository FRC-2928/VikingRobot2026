package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants;


public class ClimberIOReal implements ClimberIO {
	public ClimberIOReal() {
		
	}
	
	//climber moter, kraken x60
	private final TalonFX climber = new TalonFX(Constants.CAN.CTRE.climber, Constants.CAN.CTRE.bus);


	public final StatusSignal<Angle> position;

	@Override
	public void updateInputs(final ClimberIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.position /*this.home*/);
		this.position = this.position.getValueAsDouble();
	}
}