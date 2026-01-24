package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

/**
 * NetworkTables interface for shooter data collection via Elastic dashboard
 * Integrated with AdvantageKit for logging and replay
 */
public class ShooterDataCollector {
    private final ShooterLookupTableBuilder dataBuilder;
    private final ShooterDataCollectorIO io;
    private final ShooterDataCollectorIOInputsAutoLogged inputs = new ShooterDataCollectorIOInputsAutoLogged();
    
    private boolean lastTriggerState = false;
    private int totalRecordedPoints = 0;

    public ShooterDataCollector(ShooterLookupTableBuilder dataBuilder, ShooterDataCollectorIO io) {
        this.dataBuilder = dataBuilder;
        this.io = io;
        
        System.out.println("ShooterDataCollector initialized with AdvantageKit logging");
    }

    /**
     * Call this periodically (e.g., in Robot.robotPeriodic())
     * Checks for trigger and records data when activated
     */
    public void periodic() {
        // Update inputs from NetworkTables
        io.updateInputs(inputs);
        
        // Log all inputs to AdvantageKit
        Logger.processInputs("ShooterDataCollector", inputs);
        
        // Update total count in inputs for logging
        inputs.totalRecordedPoints = totalRecordedPoints;
        
        boolean currentTrigger = inputs.recordTrigger;
        
        // Detect rising edge (false -> true transition)
        if (currentTrigger && !lastTriggerState) {
            recordDataPoint();
        }
        
        lastTriggerState = currentTrigger;
    }

    /**
     * Record the current values from NetworkTables
     */
    private void recordDataPoint() {
        double distance = inputs.distanceMeters;
        double velocity = inputs.velocityRPS;
        double hoodAngle = inputs.hoodAngleDegrees;
        boolean successful = inputs.successful;
        String notes = inputs.notes;
        
        // Flash recording status
        inputs.recordingStatus = true;
        io.setRecordingStatus(true);
        
        boolean success = dataBuilder.recordDataPoint(
            distance, velocity, hoodAngle, successful, notes
        );
        
        if (success) {
            totalRecordedPoints++;
            
            // Log the recorded data point to AdvantageKit
            Logger.recordOutput("ShooterDataCollector/LastRecorded/Distance", distance);
            Logger.recordOutput("ShooterDataCollector/LastRecorded/Velocity", velocity);
            Logger.recordOutput("ShooterDataCollector/LastRecorded/HoodAngle", hoodAngle);
            Logger.recordOutput("ShooterDataCollector/LastRecorded/Successful", successful);
            Logger.recordOutput("ShooterDataCollector/LastRecorded/Notes", notes);
            Logger.recordOutput("ShooterDataCollector/TotalPoints", totalRecordedPoints);
            
            System.out.println(String.format(
                "Recorded from dashboard: dist=%.2fm, vel=%.1frps, angle=%.1fdeg, success=%b",
                distance, velocity, hoodAngle, successful
            ));
        }
        
        // Reset status after brief delay
        new Thread(() -> {
            try {
                Thread.sleep(200);
                inputs.recordingStatus = false;
                io.setRecordingStatus(false);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    /**
     * Manually trigger recording with current dashboard values
     */
    public void recordNow() {
        recordDataPoint();
    }

    /**
     * Get total number of recorded data points this session
     */
    public int getTotalRecordedPoints() {
        return totalRecordedPoints;
    }
}
