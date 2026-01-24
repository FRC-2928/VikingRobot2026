package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ShooterLookupTableBuilder;

/**
 * Command to record a shooter data point during testing
 */
public class RecordShooterDataCommand extends Command {
    private final ShooterLookupTableBuilder dataBuilder;
    private final double distanceMeters;
    private final double velocityRPS;
    private final double hoodAngleDegrees;
    private final boolean successful;
    private final String notes;
    private boolean finished = false;

    public RecordShooterDataCommand(
            ShooterLookupTableBuilder dataBuilder,
            double distanceMeters,
            double velocityRPS,
            double hoodAngleDegrees,
            boolean successful,
            String notes) {
        this.dataBuilder = dataBuilder;
        this.distanceMeters = distanceMeters;
        this.velocityRPS = velocityRPS;
        this.hoodAngleDegrees = hoodAngleDegrees;
        this.successful = successful;
        this.notes = notes;
    }

    @Override
    public void initialize() {
        finished = dataBuilder.recordDataPoint(
            distanceMeters, velocityRPS, hoodAngleDegrees, successful, notes
        );
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
