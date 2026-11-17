package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.subsystems.hopperfloor.HopperFloorGoal;
import frc.robot.superstructure.SuperstructureContext;
import org.littletonrobotics.junction.Logger;

public class HopperFloor extends SubsystemBase {
    private static final double RUN_SPEED_RPS = 45.0;
    private static final double REVERSE_VOLTAGE = -5.0;

    enum WantedState { IDLE, RUNNING, REVERSE }
    enum SystemState { IDLE, RUNNING, REVERSE }

    private final HopperFloorIO io;
    private final HopperFloorIOInputsAutoLogged inputs =
            new HopperFloorIOInputsAutoLogged();
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public HopperFloor() {
        this(switch (Constants.mode) {
            case REAL -> new HopperFloorIOReal();
            default -> new HopperFloorIO() {};
        });
    }

    public HopperFloor(HopperFloorIO io) {
        this.io = java.util.Objects.requireNonNull(io);
    }

    public void applyGoal(final SuperstructureContext ctx) {
        HopperFloorGoal desired = ctx.goal().hopperFloor();
        wantedState = switch (desired) {
            case IDLE -> WantedState.IDLE;
            case RUN -> WantedState.RUNNING;
            case REVERSE -> WantedState.REVERSE;
        };
    }
    //Only temporary
    @Deprecated
    public Command runHopperCommand() {
        return new RunCommand(() -> wantedState = WantedState.RUNNING, this)
                .finallyDo(this::halt);
    }

    //Only temporary
    @Deprecated
    public Command runReverseHopperCommand() {
        return new RunCommand(() -> wantedState = WantedState.REVERSE, this)
                .finallyDo(this::halt);
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLE;
            case RUNNING -> SystemState.RUNNING;
            case REVERSE -> SystemState.REVERSE;
        };
    }

    private void applyState() {
        switch (systemState) {
            case IDLE -> io.halt();
            case RUNNING -> io.setSpeed(RUN_SPEED_RPS);
            case REVERSE -> io.setVoltage(REVERSE_VOLTAGE);
        }
    }

    public record HopperFloorState(boolean isRunning, boolean isReversing) {}

    public HopperFloorState getState() {
        return new HopperFloorState(
                systemState == SystemState.RUNNING,
                systemState == SystemState.REVERSE);
    }
    public BaseStatusSignal[] getStatusSignals() {
        return io.getStatusSignals();
    }

    public void halt() {
        wantedState = WantedState.IDLE;
        systemState = SystemState.IDLE;
        io.halt();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("HopperFloor", inputs);
        systemState = handleStateTransition();
        applyState();
        Logger.recordOutput("HopperFloor/WantedState", wantedState.name());
        Logger.recordOutput("HopperFloor/SystemState", systemState.name());
    }

    @Override
    public void simulationPeriodic() {
        io.simPeriodic();
    }
}
