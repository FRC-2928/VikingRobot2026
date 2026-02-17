package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.commands.climber.climberCommand;

//Franklin needs to finish once the climber design is done.
public class ClimberIOReal implements ClimberIO {
	public ClimberIOReal() {
		climber = new TalonFX(Constants.CAN.CTRE.climber, Constants.CAN.CTRE.bus); //sets the climb motor to the CAN Bus id
		

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

		this.position = this.climber.getPosition();
		this.home = this.climber.getReverseLimit();
		

		this.lock(true, this.position.getValueAsDouble()); //locks the motor at the current position
	}
	
	//climber moter, kraken x60
	private final TalonFX climber; //intializes the climber motor variable

	private StatusSignal<Angle> position; //status signal for the postion of the climber based on the angle
	private StatusSignal<ReverseLimitValue> home;
	
	//positon values
	private double MAXheight = 30; //inches of height increase 
	private double MINheight = 0; 
	private double demandPosition;
	private boolean disengaging; //boolean for if the climber is disengaging from the climb
	private double disengagingStartPos; //the position the climber starts disengaging at
	private boolean invertedDirection = false;
	
	private climberCommand.ClimberHeight targetHeight = climberCommand.ClimberHeight.HOMEPOS;

	@Override
	public void periodic() {
		
		Logger.recordOutput("Climber/ClimbUp", this.demandPosition == targetHeight.height); //records if the Climber is climbing up
		Logger.recordOutput("Climber/Disengaging", this.disengaging);
		if (targetHeight == climberCommand.ClimberHeight.HOMEPOS) {
			if (getInInches(this.demandPosition) < 30 && this.demandPosition > 0) {
				//reset the climber to home
				climber.setControl(new PositionDutyCycle(MINheight));
				disengaging = true;

			}
		} else if (targetHeight == climberCommand.ClimberHeight.L1){
			if (Math.abs(this.demandPosition - getInInches(this.position.getValueAsDouble())) < 0.1) {
			climber.setControl(new PositionDutyCycle(getInRotations(this.demandPosition)));
			}
		} else if (targetHeight == climberCommand.ClimberHeight.L2) {
			if (Math.abs(this.demandPosition - getInInches(this.position.getValueAsDouble())) < 0.1 && getInInches(this.position.getValueAsDouble()) < 30) {
				//checks to see if the position is less than or equal to L1
				climber.setControl(new PositionDutyCycle(null));
			} else if (Math.abs(this.demandPosition - getInInches(this.position.getValueAsDouble())) < 0.1 && getInInches(this.position.getValueAsDouble()) > 30) {
				//when the climber has passsed the first level
				if (!invertedDirection) {
					invertedDirection = true;
					//tells the code the motor needs to reverse direction
				}
				//might use this:
				//double change = getInRotations(this.demandPosition - 30);
				//climber.setControl(new PositionDutyCycle(change - getInRotations(demandPosition)));
				climber.setControl(new PositionDutyCycle(MINheight)); //sets it to the home position

			} else if (this.demandPosition == getInInches(this.position.getValueAsDouble())) {
				lock(false, getInRotations(this.demandPosition));
			}
		} else if (targetHeight == climberCommand.ClimberHeight.L3) {
			if (Math.abs(this.demandPosition - getInInches(this.position.getValueAsDouble())) < 0.1 && getInInches(this.position.getValueAsDouble()) < 30) {
				//checks to see if the position is less than or equal to L1
				climber.setControl(new PositionDutyCycle(null));
			} else if (Math.abs(this.demandPosition - getInInches(this.position.getValueAsDouble())) < 0.1 && getInInches(this.position.getValueAsDouble()) > 30) {
				//when the climber has passsed the first level
				if (!invertedDirection) {
					invertedDirection = true;
					//tells the code the motor needs to reverse direction
				} else {
					//might use this:
					//double change = getInRotations(this.demandPosition - 30);
					//climber.setControl(new PositionDutyCycle(change - getInRotations(demandPosition)));
					climber.setControl(new PositionDutyCycle(MINheight)); //sets it to the home position
				}

			} else if (Math.abs(this.demandPosition - getInInches(this.position.getValueAsDouble())) < 0.1 && getInInches(this.position.getValueAsDouble()) > 48) {
				if (invertedDirection) {
					invertedDirection = false;
				} else {
					climber.setControl(new PositionDutyCycle(getInRotations(demandPosition - )));
				}
			} else if (this.demandPosition == getInInches(this.position.getValueAsDouble())) {
				lock(false, getInRotations(this.demandPosition));
			}
		}
	}

	public void setClimberHeight(climberCommand.ClimberHeight newHeight) {
		this.targetHeight = newHeight;
	}

	public climberCommand.ClimberHeight getClimberHeight() {
		return targetHeight;
	}

	/*public void periodic() {
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
				if (this.position.getValueAsDouble() < this.disengagingStartPos - Constants.Climber.disengageDistance || this.home.getValue() == ReverseLimitValue.ClosedToGround) {
					//checks to see if the climber is already at the home positon (or close to it)
					climber.setControl(new PositionDutyCycle(this.demandPosition));

					this.disengaging = false; //stops the disengaging

					Logger.recordOutput("Climber/State", "Disengaging Finished"); //logs the finished disengage
				}else {
					//if the climber is not near the home position it continues to disengage
					climber.setControl(new PositionDutyCycle(this.disengagingStartPos - Constants.Climber.disengageDistance * 2));

					Logger.recordOutput("Climber/State", "Disengaging Moving");
				}
			} else {
				this.disengagingStartPos = this.position.getValueAsDouble(); //sets the position the climber starts disengaging at to the current position
				this.disengaging = true; //sets the climber to disengaging

				Logger.recordOutput("Climber/State", "Disengaging");
			}
		}
		//Logger.recordOutput("Climber/Locked" this.locked ); //records if the Climber is locked


	}
	*/
	@Override
	public void override(final double dutycycle) {
		climber.setControl(new PositionDutyCycle(dutycycle));
		this.demandPosition = this.position.getValueAsDouble();
	}

	//create a climb function
	
	private void lock(final boolean engaged, final double target) {
		if (engaged) {
			if (Math.abs(target - this.position.getValueAsDouble()) < 0.1) {
			climber.setControl(new DutyCycleOut(this.position.getValueAsDouble()));
		} else {
			climber.setControl(new DutyCycleOut(target));
		}
		}
	}

	
	@Override
	public void updateInputs(final ClimberIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.position, home); //updates the position of the climber.
		inputs.position = this.getInRotations(climber.getPosition().getValueAsDouble());
	}

	//get the distance in inches
	private double getInInches(double rotations) {
		final double gearRatio = 5; //the amount of rotation of motor to rotation of gear

		final double circumference = 1.5 * Math.PI;

		return (rotations/gearRatio)*circumference;
	}
	private double getInRotations(double inches) {
		final double gearRatio = 5;

		final double circumference = 1.5 * Math.PI;

		return (inches/circumference) * gearRatio;
	}
}

/*
    /\_/\
  =(• . •)=
   /     \    
   pls keep the silly cat for vibes (trust)     
*/ 