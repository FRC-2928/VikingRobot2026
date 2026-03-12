package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class HopperFloor extends SubsystemBase {
    private HopperFloorIO io;
    public HopperFloorIOInputsAutoLogged hopperIOInputsAutoLogged = new HopperFloorIOInputsAutoLogged();

    public HopperFloor() {
        this.io = switch (Constants.mode) {
            case REAL -> new HopperFloorIOReal();
            default -> new HopperFloorIOReal();};
    }

    public Command runHopperCommand() {
        return new RunCommand(() -> io.runHopper(), this).finallyDo(() -> io.halt());
    }

    public Command runReverseHopperCommand() {
        return new RunCommand(() -> io.runHopperReverse(), this).finallyDo(() -> io.halt());
    }

    public void halt() {
        io.halt();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.hopperIOInputsAutoLogged);
        Logger.processInputs("HopperFloor", this.hopperIOInputsAutoLogged);
    }

    @Override
    public void simulationPeriodic() {
        io.simPeriodic();
    }
}
