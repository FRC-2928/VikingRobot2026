package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.HopperFloorIO.HopperFloorIOInputs;

public class HopperFloor extends SubsystemBase {
    public HopperFloorIO io;
    public HopperFloorIOInputs hopperIOInputs = new HopperFloorIOInputs();

    public HopperFloor() {
            this.io = switch (Constants.mode) {
            case REAL -> new HopperFloorIOReal();
            default -> throw new Error();};
    }

    public Command runHopperCommand() {
        return new RunCommand(() -> io.runHopper(), this).finallyDo(() -> io.halt());
    }


    @Override
    public void periodic() {
        this.io.updateInputs(this.hopperIOInputs);
    }
}
