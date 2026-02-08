package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

import frc.robot.Constants;

//Franklin needs to finish once the climber design is done.
public class ClimberIOReal implements ClimberIO {
	public ClimberIOReal() {
		climber = new TalonFX(Constants.CAN.CTRE.climber, Constants.CAN.CTRE.bus); //sets the climb motor to the CAN Bus id
		hook = new TalonFX(Constants.CAN.CTRE.climberHook, Constants.CAN.CTRE.bus); //sets the hook motor to the CAN Bus id

		//motors configs
		final TalonFXConfiguration climberConfig = new TalonFXConfiguration(); //creates a new configuration for the climber motor
		climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //inverts the climber motor
		climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; //sets the climber motor to brake mode
		climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAXheight;
		//enable rotation limits so the motor never pulls the climber into itself
		climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MINheight;
		climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

		//applying the motor configs
		climber.getConfigurator().apply(climberConfig);
		//climberConfig.HardwareLimitSwitch.

		final TalonFXConfiguration hookConfig = new TalonFXConfiguration(); //creates a new configuration for the hook motor
		hookConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //inverts the hook motor
		hookConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; //sets the hook motor to brake mode

		//applying the motor configs
		climber.getConfigurator().apply(climberConfig);

		this.position = this.climber.getPosition();
		this.home = this.climber.getReverseLimit();
		

		//this.lock(true); configed it so the motor locks when it stops (hopefully)
	}
	
	//climber moter, kraken x60
	private final TalonFX climber; //intializes the climber motor variable
	private final TalonFX hook; //intializes the hook motor variable

	private StatusSignal<Angle> position; //status signal for the postion of the climber based on the angle
	private StatusSignal<ReverseLimitValue> home;
	
	
	private double MAXheight; //set the amount of rotations needed to get to max height
	private double MINheight = 0;
	private double demandPosition;
	private boolean disengaging; //boolean for if the climber is disengaging from the climb
	private double disengagingStartPos; //the position the climber starts disengaging at

	

	@Override
	public void periodic() {
		Logger.recordOutput("Climber/ClimbUp", this.demandPosition == this.MAXheight); //records if the Climber is climbing up
		Logger.recordOutput("Climber/Disengaging", this.disengaging); //records if the Climber is disengaging from the climb

		if (Math.abs(this.demandPosition - this.position.getValueAsDouble()) < 0.1) {
			//lock the motor if the position is within 0.1 units of the demand position
			climber.setControl(new PositionDutyCycle(this.position.getValueAsDouble()));
		} else if (this.demandPosition -0.1 <= this.position.getValueAsDouble()) {
			climber.setControl(new PositionDutyCycle(this.demandPosition)); //sets the control of the climber motor to the demand position
			Logger.recordOutput("Climber/State", "Engaged");

			disengaging = false;
		} else {
			if (this.disengaging) {
				/*if () {

				}else {

				}*/
			} else {
				this.disengagingStartPos = this.position.getValueAsDouble(); //sets the position the climber starts disengaging at to the current position
				this.disengaging = true; //sets the climber to disengaging

				Logger.recordOutput("Climber/State", "Disengaging");
			}
		}


		//Logger.recordOutput("Climber/Disengaging" /*this.disengaging*/); //records if the Climber is disengaging from the climb 
		//Logger.recordOutput("Climber/Locked" /*this.locked */); //records if the Climber is locked


	}

	//create a climb function
	
	private void lock(final boolean engaged) {
		//climber.
		
	}

	@Override
	public void updateInputs(final ClimberIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.position, home); //updates the position of the climber.
		inputs.position = climber.getPosition().getValueAsDouble(); //gives the positiong of the climber to the inputs

		
		
		/*if (input.positon == MINheight) {
			this.home = true;
		} else {
			this.home = false;
		}*/

	}
}

/*
    /\_/\
  =(• . •)=
   /     \    
   pls keep the silly cat for vibes (trust)     
*/ 