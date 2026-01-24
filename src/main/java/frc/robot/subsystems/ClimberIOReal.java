package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;

import frc.robot.Constants;

//Franklin needs to finish once the climber design is done.
public class ClimberIOReal implements ClimberIO {
	public ClimberIOReal() {
		climber = new TalonFX(Constants.CAN.CTRE.climber, Constants.CAN.CTRE.bus); //sets the climb motor to the CAN Bus id



		this.position = this.climber.getPosition();
		//this.home = 

		this.lock(true);
	}
	
	//climber moter, kraken x60
	private final TalonFX climber; //intializes the climber motor variable


	private StatusSignal<Angle> position;

	
	
	private double height;
	private double demandPosition;

	
	@Override
	public void set(final double position) { this.demandPosition = position; } //set

	@Override
	public void periodic() {
		//Logger.recordOutput("Climber/ClimbUp" /*this.demandPosition == */); //records the posito
		//Logger.recordOutput("Climber/Disengaging" /*this.disengaging*/); //records if the Climber is disengaging from the climb 
		//Logger.recordOutput("Climber/Locked" /*this.locked */); //records if the Climber is locked


	}

	//create a climb function
	
	private void lock(final boolean engaged) {
		//create a function to set the angle of the motor
		//Logger.recordOutput("Climber/LockEngaged", engaged);
	}

	@Override
	public void updateInputs(final ClimberIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.position /*this.home*/); //updates the position of the climber.
		inputs.position = position.getValueAsDouble(); 
	}
}