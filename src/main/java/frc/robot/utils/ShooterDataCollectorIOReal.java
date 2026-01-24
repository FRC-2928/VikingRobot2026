package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.BooleanPublisher;

public class ShooterDataCollectorIOReal implements ShooterDataCollectorIO {
    private final NetworkTable table;
    
    private final DoubleSubscriber distanceInput;
    private final DoubleSubscriber velocityInput;
    private final DoubleSubscriber hoodAngleInput;
    private final BooleanSubscriber successfulInput;
    private final StringSubscriber notesInput;
    private final BooleanSubscriber recordTrigger;
    private final BooleanPublisher recordingStatus;

    public ShooterDataCollectorIOReal() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("ShooterData");
        
        distanceInput = table.getDoubleTopic("distance_m").subscribe(0.0);
        velocityInput = table.getDoubleTopic("velocity_rps").subscribe(0.0);
        hoodAngleInput = table.getDoubleTopic("hood_angle_deg").subscribe(0.0);
        successfulInput = table.getBooleanTopic("successful").subscribe(false);
        notesInput = table.getStringTopic("notes").subscribe("");
        recordTrigger = table.getBooleanTopic("record_trigger").subscribe(false);
        recordingStatus = table.getBooleanTopic("recording_status").publish();
        
        recordingStatus.set(false);
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
