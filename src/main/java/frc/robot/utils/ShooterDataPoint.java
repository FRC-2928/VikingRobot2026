package frc.robot.utils;

/**
 * Represents a single shooter trial data point
 */
public class ShooterDataPoint {
    private final double distanceMeters;
    private final double velocityRPS;
    private final double hoodAngleDegrees;
    private final long timestamp;
    private final boolean successful;
    private final String notes;

    public ShooterDataPoint(double distanceMeters, double velocityRPS, double hoodAngleDegrees, 
                           boolean successful, String notes) {
        this.distanceMeters = distanceMeters;
        this.velocityRPS = velocityRPS;
        this.hoodAngleDegrees = hoodAngleDegrees;
        this.timestamp = System.currentTimeMillis();
        this.successful = successful;
        this.notes = notes;
    }

    public double getDistanceMeters() { return distanceMeters; }
    public double getVelocityRPS() { return velocityRPS; }
    public double getHoodAngleDegrees() { return hoodAngleDegrees; }
    public long getTimestamp() { return timestamp; }
    public boolean isSuccessful() { return successful; }
    public String getNotes() { return notes; }

    /**
     * Convert to CSV format
     */
    public String toCSV() {
        return String.format("%.3f,%.3f,%.3f,%d,%b,%s",
            distanceMeters, velocityRPS, hoodAngleDegrees, timestamp, successful, 
            notes.replace(",", ";"));
    }

    /**
     * CSV header
     */
    public static String getCSVHeader() {
        return "distance_m,velocity_rps,hood_angle_deg,timestamp,successful,notes";
    }
}
