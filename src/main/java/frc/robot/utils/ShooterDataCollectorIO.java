package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterDataCollectorIO {
    @AutoLog
    public static class ShooterDataCollectorIOInputs {
        public double distanceMeters = 0.0;
        public double velocityRPS = 0.0;
        public double hoodAngleDegrees = 0.0;
        public boolean successful = false;
        public String notes = "";
        public boolean recordTrigger = false;
        public boolean recordingStatus = false;
        public int totalRecordedPoints = 0;
    }

    /**
     * Update inputs from NetworkTables
     */
    public default void updateInputs(ShooterDataCollectorIOInputs inputs) {}

    /**
     * Set the recording status output
     */
    public default void setRecordingStatus(boolean status) {}
}
