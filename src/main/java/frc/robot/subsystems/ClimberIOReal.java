package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
        climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // inverts the climber motor
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // sets the climber motor to brake mode
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAXheight;
        // enable rotation limits so the motor never pulls the climber into itself
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MINheight;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // set the gear ratio for the climber motor
        climberConfig.Feedback.SensorToMechanismRatio = 25;

        //configs for the PDIF loop
        final Slot0Configs slot0configs = new Slot0Configs()
            .withKP(0.1) //output per unit of error in velocity
            .withKI(0) //output per unit of integrated error in velocity
            .withKD(0.1) //output per unit of error derivative in velocity
            .withKV(1); //target of 1rps per 1 V output


        //add the slot0 configs to the climber configs
        climberConfig.Slot0 = slot0configs;
        // applying the motor configs
        climber.getConfigurator().apply(climberConfig);

    }

    // climber moter, kraken x60
    private final TalonFX climber; // intializes the climber motor variable

    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    // positon values
    private double MAXheight = 30; // inches of height increase
    private double MINheight = 0;
    final DigitalInput forwardLimit;
    final DigitalInput reverseLimit;

    // call in climber.java periodic

    @Override
    public void goHome() {
        climber.setControl(new VoltageOut(Units.Volts.of(-5))
            .withLimitReverseMotion(reverseLimit.get())
        );
    }

    @Override
    public void extend() {
        climber.setControl(new VoltageOut(Units.Volts.of(5))
            .withLimitForwardMotion(forwardLimit.get())
        );
    }
    @Override
    public void climb(Distance distance) {
        climber.setControl(request.withPosition(distance.in(Units.Inches))
            .withLimitForwardMotion(forwardLimit.get())
            .withLimitReverseMotion(reverseLimit.get())
        );
    }

    
    @Override
    public void override() {
        goHome();
    }

    @Override
    public void updateInputs(final ClimberIOInputs inputs) {
        // BaseStatusSignal.refreshAll(ClimberIO.height, climber.home); // updates the position of the climber.
        inputs.height = this.climber.getPosition().getValueAsDouble();
    }
}

/*
    /\_/\
  =(• . •)=
   /     \
   pls keep the silly cat for vibes (trust)
*/

