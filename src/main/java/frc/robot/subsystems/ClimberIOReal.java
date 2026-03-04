package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

// Franklin needs to finish once the climber design is done.
public class ClimberIOReal implements ClimberIO {
    public ClimberIOReal() {
        climber = new TalonFX(
                Constants.CAN.CTRE.climber, Constants.CAN.CTRE.bus); // sets the climb motor to the CAN Bus id
        forwardLimit = new DigitalInput(0);
        reverseLimit = new DigitalInput(1);

        // motors configs
        final TalonFXConfiguration climberConfig =
                new TalonFXConfiguration(); // creates a new configuration for the climber motor
        climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // inverts the climber motor
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // sets the climber motor to brake mode
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAXheight;
        // enable rotation limits so the motor never pulls the climber into itself
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MINheight;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;


        // set the gear ratio for the climber motor
        climberConfig.Feedback.SensorToMechanismRatio = 25;

        // applying the motor configs
        climber.getConfigurator().apply(climberConfig);
    }

    // climber moter, kraken x60
    private final TalonFX climber; // intializes the climber motor variable

    // positon values
    private double MAXheight = 30; // inches of height increase
    private double MINheight = 0;
    final DigitalInput forwardLimit;
    final DigitalInput reverseLimit;

    // call in climber.java periodic
    @Override
    public void goUp() {
        climber.setControl(new VoltageOut(Units.Volts.of(5))
            .withLimitForwardMotion(forwardLimit.get())
            .withLimitReverseMotion(reverseLimit.get())
        );
    }

    @Override
    public void goToPosition(Distance position) {
        climber.setControl(new PositionVoltage(position.in(Units.Inches))
            .withLimitForwardMotion(forwardLimit.get())
            .withLimitReverseMotion(reverseLimit.get())
        );
    }
    
    @Override
    public void goDown() {
        climber.setControl(new VoltageOut(Units.Volts.of(-5))
            .withLimitForwardMotion(forwardLimit.get())
            .withLimitReverseMotion(reverseLimit.get())
        );
    }


    @Override
    public void halt() {
        climber.setControl(new VoltageOut(Units.Volts.of(0))
            .withLimitForwardMotion(forwardLimit.get())
            .withLimitReverseMotion(reverseLimit.get())
        );
    }

    LinearFilter spikeFilter = LinearFilter.backwardFiniteDifference(1, 1, 0.2);

    public boolean isEngaged() {
        double current = climber.getStatorCurrent().getValueAsDouble();

        double currentDerivative = spikeFilter.calculate(current);

        if (currentDerivative >= 150) { // Threshold of 150 Amps/sec spike
            return true;
        } else {
            return true; //change to false at week 2
        }
    }

    @Override
    public void override() {
        goDown();
    }

    @Override
    public void updateInputs(final ClimberIOInputs inputs) {
        // BaseStatusSignal.refreshAll(ClimberIO.height, climber.home); // updates the position of the climber.
        inputs.height = this.climber.getPosition().getValueAsDouble();
        inputs.forwardLimitSwitch = this.forwardLimit.get();
        inputs.reverseLimitSwitch = this.reverseLimit.get();
    }
}

/*
    /\_/\
  =(• . •)=
   /     \
   pls keep the silly cat for vibes (trust)
*/

