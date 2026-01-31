package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.HopperFloorIO.HopperFloorIOInputs;
import frc.robot.subsystems.IntakeIO.IntakeIOInputs;

public class HopperFloor extends SubsystemBase {
    public HopperFloorIO io;
    public HopperFloorIOInputs hopperIOInputs = new HopperFloorIOInputs();

    public HopperFloor() {
            this.io = switch (Constants.mode) {
            case REAL -> new HopperFloorIOReal();
            default -> throw new Error();};
    }


    @Override
    public void periodic() {
        this.io.updateInputs(this.hopperIOInputs);
    }
}
