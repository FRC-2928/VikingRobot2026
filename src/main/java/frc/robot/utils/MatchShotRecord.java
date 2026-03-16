package frc.robot.utils;

/**
 * Represents a single shot record captured during a match.
 * Used by MatchRecorder to log telemetry and write CSV output.
 */
public class MatchShotRecord {
    private final double matchTimestamp;
    private final double distanceMeters;
    private final double rotationToTargetDeg;
    private final double computedShotAngleDeg;
    private final double actualShotAngleDeg;
    private final double targetHoodAngleDeg;
    private final double currentHoodAngleDeg;
    private final double targetFlywheelRPS;
    private final double currentFlywheelRPS;

    public MatchShotRecord(
            double matchTimestamp,
            double distanceMeters,
            double rotationToTargetDeg,
            double computedShotAngleDeg,
            double actualShotAngleDeg,
            double targetHoodAngleDeg,
            double currentHoodAngleDeg,
            double targetFlywheelRPS,
            double currentFlywheelRPS) {
        this.matchTimestamp = matchTimestamp;
        this.distanceMeters = distanceMeters;
        this.rotationToTargetDeg = rotationToTargetDeg;
        this.computedShotAngleDeg = computedShotAngleDeg;
        this.actualShotAngleDeg = actualShotAngleDeg;
        this.targetHoodAngleDeg = targetHoodAngleDeg;
        this.currentHoodAngleDeg = currentHoodAngleDeg;
        this.targetFlywheelRPS = targetFlywheelRPS;
        this.currentFlywheelRPS = currentFlywheelRPS;
    }

    public double getMatchTimestamp() { return matchTimestamp; }
    public double getDistanceMeters() { return distanceMeters; }
    public double getRotationToTargetDeg() { return rotationToTargetDeg; }
    public double getComputedShotAngleDeg() { return computedShotAngleDeg; }
    public double getActualShotAngleDeg() { return actualShotAngleDeg; }
    public double getTargetHoodAngleDeg() { return targetHoodAngleDeg; }
    public double getCurrentHoodAngleDeg() { return currentHoodAngleDeg; }
    public double getTargetFlywheelRPS() { return targetFlywheelRPS; }
    public double getCurrentFlywheelRPS() { return currentFlywheelRPS; }

    /**
     * Returns a comma-separated string of all fields in header order.
     */
    public String toCSV() {
        return String.format(
                "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                matchTimestamp,
                distanceMeters,
                rotationToTargetDeg,
                computedShotAngleDeg,
                actualShotAngleDeg,
                targetHoodAngleDeg,
                currentHoodAngleDeg,
                targetFlywheelRPS,
                currentFlywheelRPS);
    }

    /**
     * Returns the CSV header row matching field order.
     */
    public static String getCsvHeader() {
        return "matchTimestamp,distanceMeters,rotationToTargetDeg,computedShotAngleDeg,"
                + "actualShotAngleDeg,targetHoodAngleDeg,currentHoodAngleDeg,targetFlywheelRPS,currentFlywheelRPS";
    }
}
