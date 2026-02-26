package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.climber.ClimberCommand;

// Franklin needs to finish once the climber design is done.
public class ClimberIOReal implements ClimberIO {
    public ClimberIOReal() {
        climber = new TalonFX(
                Constants.CAN.CTRE.climber, Constants.CAN.CTRE.bus); // sets the climb motor to the CAN Bus id

        // motors configs
        final TalonFXConfiguration climberConfig =
                new TalonFXConfiguration(); // creates a new configuration for the climber motor
        climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // inverts the climber motor
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // sets the climber motor to brake mode
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAXheight;
        // enable rotation limits so the motor never pulls the climber into itself
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MINheight;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        climberConfig.MotorOutput.

        // applying the motor configs
        climber.getConfigurator().apply(climberConfig);

        this.position = this.climber.getPosition();
        this.home = this.climber.getReverseLimit();

    }

    // climber moter, kraken x60
    private final TalonFX climber; // intializes the climber motor variable

    private StatusSignal<Angle> position; // status signal for the postion of the climber based on the angle
    private StatusSignal<ReverseLimitValue> home;

    // positon values
    private double MAXheight = 30; // inches of height increase
    private double MINheight = 0;
    private boolean engaged = false;

    // tracks if L1 is completed
    private boolean L1complete = false;
    private boolean L2complete = false;

    private ClimberCommand.ClimberHeight targetHeight = Robot.cont.climber.inputs.targetheight;
    private ClimberCommand.ClimberState currentState = Robot.cont.climber.inputs.state;

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/State", this.currentState);
        Logger.recordOutput("Climber/ClimbUp", targetHeight.height); // records the target height of the climber

        // checks to see if the climber is in idle state
        if (currentState != ClimberCommand.ClimberState.IDLE && currentState != ClimberCommand.ClimberState.FAILED) {
            if (targetHeight == ClimberCommand.ClimberHeight.HOMEPOS
                    && currentState == ClimberCommand.ClimberState.DESCENDING) {
                if (this.position.getValueAsDouble() == 0) {
                    // reset the climber to home
                    climber.setControl(new PositionDutyCycle(30));
                } else if (this.position.getValueAsDouble() == 30) {
                    Robot.cont.climber.inputs.state = ClimberCommand.ClimberState.IDLE;
                }
            } else if (targetHeight == ClimberCommand.ClimberHeight.L1
                    && currentState == ClimberCommand.ClimberState.ASCENDING) {
                goToL1();

            } else if (targetHeight == ClimberCommand.ClimberHeight.L2) {
                goToL1();
                if (L1complete) {
                    // go to L2
                    goToL2();
                }
            } else if (targetHeight == ClimberCommand.ClimberHeight.L3) {
                goToL1();
                if (L1complete) {
                    // go to L2
                    goToL2();
                    if (L2complete) {
                        goToL3();
                    }
                }
            }
        }
    }

    // function that handles going to L1 :)
    private void goToL1() {
        if (!L1complete) {
            if (this.position.getValueAsDouble() <= 30 && this.engaged == false) {
                climber.setControl(new PositionDutyCycle(targetHeight.height));
            } else if (this.position.getValueAsDouble() == 30) 
                this.engaged = true;
                climber.setControl(new PositionDutyCycle(MINheight));
            }
            if (this.position.getValueAsDouble() <= 30 && this.engaged == true) {
                if (this.position.getValueAsDouble() != 0) {
                    climber.setControl(new PositionDutyCycle(MINheight));
                } else {
                    Robot.cont.climber.inputs.state = ClimberCommand.ClimberState.IDLE;
                    L1complete = true;
                    this.engaged = false;
                }
            }
        }
    }

    // función que maneja ir a L2
    private void goToL2() {
        if (!L2complete) {
            if (this.position.getValueAsDouble() <= 18 && this.engaged == false) {
                climber.setControl(new PositionDutyCycle(targetHeight.height));
            } else if (this.position.getValueAsDouble() == 18) {
                this.engaged = true;
                climber.setControl(new PositionDutyCycle(MINheight));
            }
            if (this.position.getValueAsDouble() <= 18 && this.engaged == true) {
                if (this.position.getValueAsDouble() != 0) {
                    climber.setControl(new PositionDutyCycle(MINheight));
                } else {
                    Robot.cont.climber.inputs.state = ClimberCommand.ClimberState.IDLE;
                    L2complete = true;
                    this.engaged = false;
                }
            }
        }
    }

    // Funktion, die den Wechsel zu L3 steuert
    private void goToL3() {
        if (this.position.getValueAsDouble() <= 18 && this.engaged == false) {
            climber.setControl(new PositionDutyCycle(targetHeight.height));
        } else if (this.position.getValueAsDouble() == 18) {
            this.engaged = true;
            climber.setControl(new PositionDutyCycle(MINheight));
        }
        if (this.position.getValueAsDouble() <= 18 && this.engaged == true) {
            if (this.position.getValueAsDouble() != 0) {
                climber.setControl(new PositionDutyCycle(MINheight));
            } else {
                Robot.cont.climber.inputs.state = ClimberCommand.ClimberState.IDLE;
                this.engaged = false;
            }
        }
    }

    // public void setClimberHeight(ClimberCommand.ClimberHeight newHeight) {this.targetHeight = newHeight;}

    // public ClimberCommand.ClimberHeight getClimberHeight() {return targetHeight;}

    @Override
    public void override(final double dutycycle) {
        climber.setControl(new PositionDutyCycle(dutycycle));
    }

    @Override
    public void updateInputs(final ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(this.position, this.home); // updates the position of the climber.
        inputs.position = this.climber.getPosition().getValueAsDouble();
    }

}

/*
    /\_/\
  =(• . •)=
   /     \
   pls keep the silly cat for vibes (trust)
*/
