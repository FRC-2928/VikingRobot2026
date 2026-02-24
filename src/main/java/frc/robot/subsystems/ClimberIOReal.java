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
import frc.robot.Robot;
import frc.robot.commands.climber.climberCommand;

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

        // applying the motor configs
        climber.getConfigurator().apply(climberConfig);

        this.position = this.climber.getPosition();
        this.home = this.climber.getReverseLimit();

        this.lock(true, this.position.getValueAsDouble()); // locks the motor at the current position
    }

    // climber moter, kraken x60
    private final TalonFX climber; // intializes the climber motor variable

    private StatusSignal<Angle> position; // status signal for the postion of the climber based on the angle
    private StatusSignal<ReverseLimitValue> home;

    // positon values
    private double MAXheight = getInRotations(30); // inches of height increase
    private double MINheight = 0;
    private boolean invertedDirection = false;
    private boolean engaged = false;

    // tracks if L1 is completed
    private boolean L1complete = false;
    private boolean L2complete = false;

    private climberCommand.ClimberHeight targetHeight = Robot.cont.climber.inputs.targetheight;
    private climberCommand.ClimberState currentState = Robot.cont.climber.inputs.state;

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/State", this.currentState);
        Logger.recordOutput("Climber/ClimbUp", targetHeight.height); // records the target height of the climber

        // checks to see if the climber is in idle state
        if (currentState != climberCommand.ClimberState.IDLE && currentState != climberCommand.ClimberState.FAILED) {
            if (targetHeight == climberCommand.ClimberHeight.HOMEPOS
                    && currentState == climberCommand.ClimberState.DESCENDING) {
                if (getInInches(this.position.getValueAsDouble()) == 0) {
                    // reset the climber to home
                    climber.setControl(new PositionDutyCycle(getInRotations(30)));
                } else if (getInInches(this.position.getValueAsDouble()) == 30) {
                    Robot.cont.climber.inputs.state = climberCommand.ClimberState.IDLE;
                }
            } else if (targetHeight == climberCommand.ClimberHeight.L1
                    && currentState == climberCommand.ClimberState.ASCENDING) {
                goToL1();

            } else if (targetHeight == climberCommand.ClimberHeight.L2) {
                goToL1();
                if (L1complete) {
                    // go to L2
                    goToL2();
                }
            } else if (targetHeight == climberCommand.ClimberHeight.L3) {
                goToL1();
                if (L1complete) {
                    // go to L2
                    goToL2();
                    if (L2complete) {
                        goToL3();
                    }
                }
            }
        } else if (currentState == climberCommand.ClimberState.IDLE) {
            this.lock(true, getInRotations(0)); // lock the motor if the state is idle
        }
    }

    // function that handles going to L1 :)
    private void goToL1() {
        if (!L1complete) {
            if (getInInches(this.position.getValueAsDouble()) <= 30 && this.engaged == false) {
                climber.setControl(new PositionDutyCycle(getInRotations(targetHeight.height)));
            } else if (getInInches(this.position.getValueAsDouble()) == 30) {
                this.engaged = true;
                climber.setControl(new PositionDutyCycle(getInRotations(MINheight)));
            }
            if (getInInches(this.position.getValueAsDouble()) <= 30 && this.engaged == true) {
                if (getInInches(this.position.getValueAsDouble()) != 0) {
                    climber.setControl(new PositionDutyCycle(getInRotations(MINheight)));
                } else {
                    Robot.cont.climber.inputs.state = climberCommand.ClimberState.IDLE;
                    L1complete = true;
                    this.engaged = false;
                }
            }
        }
    }

    // función que maneja ir a L2
    private void goToL2() {
        if (!L2complete) {
            if (getInInches(this.position.getValueAsDouble()) <= 18 && this.engaged == false) {
                climber.setControl(new PositionDutyCycle(getInRotations(targetHeight.height)));
            } else if (getInInches(this.position.getValueAsDouble()) == 18) {
                this.engaged = true;
                climber.setControl(new PositionDutyCycle(getInRotations(MINheight)));
            }
            if (getInInches(this.position.getValueAsDouble()) <= 18 && this.engaged == true) {
                if (getInInches(this.position.getValueAsDouble()) != 0) {
                    climber.setControl(new PositionDutyCycle(getInRotations(MINheight)));
                } else {
                    Robot.cont.climber.inputs.state = climberCommand.ClimberState.IDLE;
                    L2complete = true;
                    this.engaged = false;
                }
            }
        }
    }

    // Funktion, die den Wechsel zu L3 steuert
    private void goToL3() {
        if (getInInches(this.position.getValueAsDouble()) <= 18 && this.engaged == false) {
            climber.setControl(new PositionDutyCycle(getInRotations(targetHeight.height)));
        } else if (getInInches(this.position.getValueAsDouble()) == 18) {
            this.engaged = true;
            climber.setControl(new PositionDutyCycle(getInRotations(MINheight)));
        }
        if (getInInches(this.position.getValueAsDouble()) <= 18 && this.engaged == true) {
            if (getInInches(this.position.getValueAsDouble()) != 0) {
                climber.setControl(new PositionDutyCycle(getInRotations(MINheight)));
            } else {
                Robot.cont.climber.inputs.state = climberCommand.ClimberState.IDLE;
                this.engaged = false;
            }
        }
    }

    // public void setClimberHeight(climberCommand.ClimberHeight newHeight) {this.targetHeight = newHeight;}

    // public climberCommand.ClimberHeight getClimberHeight() {return targetHeight;}

    @Override
    public void override(final double dutycycle) {
        climber.setControl(new PositionDutyCycle(dutycycle));
    }

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
        BaseStatusSignal.refreshAll(this.position, this.home); // updates the position of the climber.
        inputs.position = this.getInRotations(climber.getPosition().getValueAsDouble());
    }

    // get the distance in inches
    private double getInInches(double rotations) {
        final double gearRatio = 25; // the amount of rotation of motor to rotation of gear

        final double circumference = 1.5 * Math.PI;

        return (rotations / gearRatio) * circumference;
    }

    private double getInRotations(double inches) {
        final double gearRatio = 25;

        final double circumference = 1.5 * Math.PI;

        return (inches / circumference) * gearRatio;
    }
}

/*
    /\_/\
  =(• . •)=
   /     \
   pls keep the silly cat for vibes (trust)
*/
