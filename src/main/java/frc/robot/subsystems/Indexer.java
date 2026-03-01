package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase {
    private IndexerIO io;
    private IndexerIOInputs indexerIOInputs = new IndexerIOInputs();

    public Indexer() {
        this.io = switch (Constants.mode) {
            case REAL -> new IndexerIOReal();
            default -> new IndexerIOReal();
        };
    }

    public Command runIndexerCommand() {
        return new RunCommand(() -> io.runIndexer(), this).finallyDo(() -> io.halt());
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.indexerIOInputs);
    }

    public Command runForwardCommand() {
        return new InstantCommand(() -> this.io.runIndexer());
    }

    public Command stopCommand() {
        return new InstantCommand(() -> this.io.setSpeed(0));
    }

    @Override
    public void simulationPeriodic() {}
}
