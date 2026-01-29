package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.StringEntry;

public class ShooterDataCollectorIOReal implements ShooterDataCollectorIO {
    private final NetworkTable table;
    
    // Use entries so we can both read and write (publish default values)
    private final DoubleEntry distanceInput;
    private final DoubleEntry velocityInput;
    private final DoubleEntry hoodAngleInput;
    private final BooleanEntry successfulInput;
    private final StringEntry notesInput;
    private final BooleanEntry recordTrigger;
    private final BooleanPublisher recordingStatus;

    public ShooterDataCollectorIOReal() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("ShooterData");
        
        // Create entries and publish default values so they appear in Elastic
        distanceInput = table.getDoubleTopic("distance_m").getEntry(0.0);
        velocityInput = table.getDoubleTopic("velocity_rps").getEntry(0.0);
        hoodAngleInput = table.getDoubleTopic("hood_angle_deg").getEntry(0.0);
        successfulInput = table.getBooleanTopic("successful").getEntry(false);
        notesInput = table.getStringTopic("notes").getEntry("");
        recordTrigger = table.getBooleanTopic("record_trigger").getEntry(false);
        recordingStatus = table.getBooleanTopic("recording_status").publish();
        
        // Publish initial values
        distanceInput.set(0.0);
        velocityInput.set(0.0);
        hoodAngleInput.set(0.0);
        successfulInput.set(false);
        notesInput.set("");
        recordTrigger.set(false);
        recordingStatus.set(false);
        
        System.out.println("ShooterData NetworkTables topics published and ready for Elastic");
    }

    @Override
    public void updateInputs(ShooterDataCollectorIOInputs inputs) {
        inputs.distanceMeters = distanceInput.get();
        inputs.velocityRPS = velocityInput.get();
        inputs.hoodAngleDegrees = hoodAngleInput.get();
        inputs.successful = successfulInput.get();
        inputs.notes = notesInput.get();
        inputs.recordTrigger = recordTrigger.get();
    }

    @Override
    public void setRecordingStatus(boolean status) {
        recordingStatus.set(status);
    }
}
