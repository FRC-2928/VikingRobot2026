package frc.robot.utils;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

/**
 * Self-contained utility class that handles:
 * 1. Loading the shooter lookup table from a CSV file on robot boot (background thread).
 * 2. Recording shot telemetry to AdvantageKit and a per-match CSV file during teleop.
 *
 * Wire-up: instantiate in Robot.robotInit(), call teleopInit() / teleopExit() from the
 * corresponding Robot lifecycle methods, and bind recordShot() to an operator button.
 */
public class MatchRecorder {

    // -----------------------------------------------------------------------
    // Constants
    // -----------------------------------------------------------------------
    private static final String LOOKUP_TABLE_PATH = "/home/lvuser/deploy/shooter_lookup_table.csv";
    private static final String LOOKUP_TABLE_SHOOT_HOME_PATH = "/home/lvuser/deploy/shooter_lookup_table_home.csv";
    private static final String OUTPUT_DIR = "/home/lvuser/match_shots/";

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------
    /** Writer shared between the robot-loop thread (open/close) and worker threads (append). */
    private BufferedWriter writer = null;

    // -----------------------------------------------------------------------
    // Constructor – spawns background lookup-table loader (sub-task 3.2)
    // -----------------------------------------------------------------------
    public MatchRecorder() {
        Thread loaderThread = new Thread(this::loadAllLookupTables, "MatchRecorder-LookupTableLoader");
        loaderThread.setDaemon(true);
        loaderThread.start();
    }

    /**
     * Reads shooter_lookup_table.csv from the deploy directory and populates
     * {@link Constants.Shooter#lookUpTable}.  Falls back to compile-time values on any error.
     */
    private void loadLookupTable(InterpolatingTreeMap<Double, Constants.Shooter.AimValues> lookUpTable, String filePath) {
        try (BufferedReader reader = new BufferedReader(new FileReader(filePath))) {
            String line;
            boolean firstLine = true;
            int rowsLoaded = 0;

            while ((line = reader.readLine()) != null) {
                // Skip header row
                if (firstLine) {
                    firstLine = false;
                    continue;
                }

                line = line.trim();
                if (line.isEmpty()) continue;

                String[] parts = line.split(",");
                if (parts.length < 3) {
                    System.err.println("[MatchRecorder] Skipping malformed row: " + line);
                    continue;
                }

                double distanceMeters = Double.parseDouble(parts[0].trim());
                double hoodAngleDeg   = Double.parseDouble(parts[1].trim());
                double flywheelRPS    = Double.parseDouble(parts[2].trim());

                lookUpTable.put(
                        distanceMeters,
                        new Constants.Shooter.AimValues(
                                Units.Degrees.of(hoodAngleDeg),
                                Units.RotationsPerSecond.of(flywheelRPS)));
                rowsLoaded++;
            }

            System.out.println("[MatchRecorder] Loaded " + rowsLoaded
                    + " lookup-table entries from " + filePath);

        } catch (Exception e) {
            System.err.println("[MatchRecorder] Failed to load lookup table from "
                    + filePath + ": " + e.getMessage());
            System.err.println("[MatchRecorder] Retaining compile-time lookup table values.");
        }
    }

    private void loadAllLookupTables() {
        loadLookupTable(Constants.Shooter.lookUpTable, LOOKUP_TABLE_PATH);
        loadLookupTable(Constants.Shooter.lookUpTableShootHome, LOOKUP_TABLE_SHOOT_HOME_PATH);
    }

    // -----------------------------------------------------------------------
    // teleopInit – open output file (sub-task 3.3)
    // -----------------------------------------------------------------------
    /**
     * Call from {@code Robot.teleopInit()}.  Builds the output filename from DriverStation
     * metadata, creates the output directory if needed, opens the writer, and writes the
     * CSV header row.
     */
    public void teleopInit() {
        // Close any previously open writer (e.g. re-enable after e-stop)
        if (writer != null) {
            teleopExit();
        }

        String eventCode = DriverStation.getEventName();
        int matchNum     = DriverStation.getMatchNumber();
        boolean hasFmsInfo = (eventCode != null && !eventCode.isEmpty()) || matchNum != 0;

        String fileName;
        if (hasFmsInfo) {
            if (eventCode == null || eventCode.isEmpty()) eventCode = "unknown";
            String matchStr    = String.valueOf(matchNum);
            Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();
            String allianceStr = allianceOpt.isPresent()
                    ? (allianceOpt.get() == DriverStation.Alliance.Red ? "Red" : "Blue")
                    : "Unknown";
            int location = DriverStation.getLocation().orElse(0);
            fileName = "shots_" + eventCode + "_match" + matchStr + "_" + allianceStr + location + ".csv";
        } else {
            // No FMS connection — use local date/time so each session gets a unique file
            String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss"));
            fileName = "shots_" + timestamp + ".csv";
        }

        File outputDir = new File(OUTPUT_DIR);
        if (!outputDir.exists()) {
            if (!outputDir.mkdirs()) {
                Logger.recordOutput("MatchRecorder/Error", "Failed to create output directory: " + OUTPUT_DIR);
                return;
            }
        }

        File outputFile = new File(outputDir, fileName);
        try {
            writer = new BufferedWriter(new FileWriter(outputFile, true));
            synchronized (writer) {
                writer.write(MatchShotRecord.getCsvHeader());
                writer.newLine();
                writer.flush();
            }
            Logger.recordOutput("MatchRecorder/OutputFile", outputFile.getAbsolutePath());
        } catch (IOException e) {
            Logger.recordOutput("MatchRecorder/Error", "teleopInit open failed: " + e.getMessage());
            writer = null;
        }
    }

    // -----------------------------------------------------------------------
    // recordShot – capture telemetry and dispatch write thread (sub-tasks 3.4 & 3.6)
    // -----------------------------------------------------------------------
    /**
     * Captures a shot record, logs it to AdvantageKit, and dispatches a short-lived
     * worker thread to append the CSV row.  No-op when teleop is not active.
     *
     * @param drivetrain the swerve drivetrain (for distance / rotation to hub)
     * @param shooter    the shooter subsystem (for hood angle and flywheel velocity)
     */
    public void recordShot(CommandSwerveDrivetrain drivetrain, Shooter shooter) {
        // Guard: only record during teleop (sub-task 3.6)
        if (!DriverStation.isTeleop()) {
            return;
        }

        // --- Gather all fields ---
        double matchTimestamp = RobotContainer.getInstance().getTeleopMatchTime();

        double distanceMeters = drivetrain.getDistanceFromHub().in(Units.Meters);

        double rotationToTargetDeg = drivetrain.getRotationToHub().getDegrees();

        // Computed target angle from lookup table (no nudge)
        Constants.Shooter.AimValues aimValues =
                Constants.Shooter.lookUpTable.get(distanceMeters);
        double computedShotAngleDeg = (aimValues != null)
                ? aimValues.hoodAngle.in(Units.Degrees)
                : 0.0;

        // Actual hood setpoint (what was commanded, including nudge) = current hood angle reading
        double currentHoodAngleDeg = shooter.getHoodAngle().in(Units.Degrees);

        // For actual shot angle we use the same as current hood angle (the real commanded value)
        double actualShotAngleDeg = currentHoodAngleDeg;

        // Target hood angle from lookup table (same as computed, in hood-motor space)
        double targetHoodAngleDeg = computedShotAngleDeg;

        // Flywheel
        double targetFlywheelRPS = (aimValues != null)
                ? aimValues.shooterVelocity.in(Units.RotationsPerSecond)
                : 0.0;
        double currentFlywheelRPS = shooter.getFlywheelVelocity().in(Units.RotationsPerSecond);

        MatchShotRecord record = new MatchShotRecord(
                matchTimestamp,
                distanceMeters,
                rotationToTargetDeg,
                computedShotAngleDeg,
                actualShotAngleDeg,
                targetHoodAngleDeg,
                currentHoodAngleDeg,
                targetFlywheelRPS,
                currentFlywheelRPS);

        // --- Log to AdvantageKit ---
        Logger.recordOutput("MatchRecorder/Shot/matchTimestamp",    matchTimestamp);
        Logger.recordOutput("MatchRecorder/Shot/distanceMeters",    distanceMeters);
        Logger.recordOutput("MatchRecorder/Shot/rotationToTargetDeg", rotationToTargetDeg);
        Logger.recordOutput("MatchRecorder/Shot/computedShotAngleDeg", computedShotAngleDeg);
        Logger.recordOutput("MatchRecorder/Shot/actualShotAngleDeg",  actualShotAngleDeg);
        Logger.recordOutput("MatchRecorder/Shot/targetHoodAngleDeg",  targetHoodAngleDeg);
        Logger.recordOutput("MatchRecorder/Shot/currentHoodAngleDeg", currentHoodAngleDeg);
        Logger.recordOutput("MatchRecorder/Shot/targetFlywheelRPS",   targetFlywheelRPS);
        Logger.recordOutput("MatchRecorder/Shot/currentFlywheelRPS",  currentFlywheelRPS);

        // --- Dispatch short-lived write thread ---
        final BufferedWriter capturedWriter = writer;
        if (capturedWriter == null) {
            Logger.recordOutput("MatchRecorder/Error", "recordShot: writer is null, skipping CSV write");
            return;
        }

        final String csvRow = record.toCSV();
        Thread writeThread = new Thread(() -> {
            synchronized (capturedWriter) {
                try {
                    capturedWriter.write(csvRow);
                    capturedWriter.newLine();
                    capturedWriter.flush();
                } catch (IOException e) {
                    Logger.recordOutput("MatchRecorder/Error", "CSV write failed: " + e.getMessage());
                }
            }
            // Thread terminates itself by returning
        }, "MatchRecorder-WriteWorker");
        writeThread.setDaemon(true);
        writeThread.start();
    }

    // -----------------------------------------------------------------------
    // teleopExit – flush and close (sub-task 3.5)
    // -----------------------------------------------------------------------
    /**
     * Call from {@code Robot.teleopExit()}.  Flushes and closes the output file so it
     * is not corrupted if the RIO loses power shortly after the match ends.
     */
    public void teleopExit() {
        if (writer == null) return;

        synchronized (writer) {
            try {
                writer.flush();
                writer.close();
            } catch (IOException e) {
                Logger.recordOutput("MatchRecorder/Error", "teleopExit close failed: " + e.getMessage());
            } finally {
                writer = null;
            }
        }
    }
}
