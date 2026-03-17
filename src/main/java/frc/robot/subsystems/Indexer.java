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
        return new RunCommand(() -> {
            io.runIndexer();
            io.runStarWheels();
        }, this)
            .finallyDo(() -> io.halt()
        );
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.indexerIOInputs);
    }

    public Command runForwardCommand() {
        return new InstantCommand(() -> {
            this.io.runIndexer();
            this.io.runStarWheels();
        });
    }

    public Command runSlowerCommand() {
        return new InstantCommand(() -> {
            this.io.setSpeedIndexer(0.5);
            this.io.setSpeedStarWheels(0.5);
        }, this).andThen(stopCommand());
    }

    public Command stopCommand() {
        return new InstantCommand(() -> this.io.setSpeedIndexer(0));
    }

    public void halt() {
        this.io.halt();
    }

    @Override
    public void simulationPeriodic() {}
}
