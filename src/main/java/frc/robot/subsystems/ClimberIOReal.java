package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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

        positionSignal = climber.getPosition();
        statorCurrent = climber.getStatorCurrent();
        supplyCurrent = climber.getSupplyCurrent();

    }

    // climber moter, kraken x60
    private final TalonFX climber; // intializes the climber motor variable
    private final StatusSignal<Angle> positionSignal; //status signal for the position of the climber
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> supplyCurrent;

    //simultation interface
    private DCMotorSim climberDCMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 25),
            DCMotor.getKrakenX60(1));

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
            .withLimitReverseMotion(!reverseLimit.get())
        );
    }

    @Override
    public void extend() {
        climber.setControl(new VoltageOut(Units.Volts.of(5))
            .withLimitForwardMotion(!forwardLimit.get())
        );
    }
    @Override
    public void climb(Distance distance) {
        climber.setControl(request.withPosition(distance.in(Units.Inches))
            .withLimitForwardMotion(!forwardLimit.get())
            .withLimitReverseMotion(!reverseLimit.get())
        );
    }

    
    @Override
    public void override() {
        goHome();
    }

    @Override
    public void simPeriodic() {
        TalonFXSimState climberSim = climber.getSimState();

        climberSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        Voltage climberVoltage = climberSim.getMotorVoltageMeasure();
        climberDCMotorSim.setInputVoltage(addFriction(climberVoltage.in(Units.Volts), 0.2));
        Logger.recordOutput("Climber/climberVoltage", climberVoltage.in(Units.Volts));

        climberDCMotorSim.update(0.02);

        climberSim.setRawRotorPosition(climberDCMotorSim.getAngularPositionRad() / (2.0 * Math.PI));
        climberSim.setRotorVelocity(climberDCMotorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI));

    }


    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        positionSignal.refresh(); // make sure it’s up to date
        inputs.height = positionSignal.getValueAsDouble(); // copy into inputs
        inputs.statorCurrent = this.statorCurrent.getValue();
        inputs.supplyCurrent = this.supplyCurrent.getValue();
    }


    /**
     * Applies the effects of friction to dampen the motor voltage.
     *
     * @param motorVoltage Voltage output by the motor
     * @param frictionVoltage Voltage required to overcome friction
     * @return Friction-dampened motor voltage
     */
    protected double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }
}

/*
    /\_/\
  =(• . •)=
   /     \
   pls keep the silly cat for vibes (trust)
*/


