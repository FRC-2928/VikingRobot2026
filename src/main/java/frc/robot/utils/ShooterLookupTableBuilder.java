package frc.robot.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Manages collection and storage of shooter tuning data
 */
public class ShooterLookupTableBuilder {
    private static final String DATA_DIRECTORY = "/home/lvuser/shooter_data";
    private static final String CURRENT_SESSION_FILE = "current_session.csv";
    private static final String MASTER_DATA_FILE = "shooter_lookup_data.csv";

    private final String sessionFilePath;
    private final String masterFilePath;
    private boolean initialized = false;

    public ShooterLookupTableBuilder() {
        // Use robot's persistent storage directory
        String baseDir = Filesystem.getOperatingDirectory().getAbsolutePath() + "/shooter_data";
        sessionFilePath = baseDir + "/" + CURRENT_SESSION_FILE;
        masterFilePath = baseDir + "/" + MASTER_DATA_FILE;
    }

    /**
     * Initialize the data collection system
     */
    public void initialize() {
        try {
            // Create directory if it doesn't exist
            File dir = new File(Filesystem.getOperatingDirectory().getAbsolutePath() + "/shooter_data");
            if (!dir.exists()) {
                dir.mkdirs();
            }

            // Create session file with header if it doesn't exist
            File sessionFile = new File(sessionFilePath);
            if (!sessionFile.exists()) {
                writeHeader(sessionFilePath);
            }

            // Create master file with header if it doesn't exist
            File masterFile = new File(masterFilePath);
            if (!masterFile.exists()) {
                writeHeader(masterFilePath);
            }

            initialized = true;
        } catch (IOException e) {
        }
    }

    /**
     * Record a new data point
     */
    public boolean recordDataPoint(
            double distanceMeters, double velocityRPS, double hoodAngleDegrees, boolean successful, String notes) {
        if (!initialized) {
            return false;
        }

        ShooterDataPoint dataPoint =
                new ShooterDataPoint(distanceMeters, velocityRPS, hoodAngleDegrees, successful, notes);

        try {
            // Append to both session and master files
            appendDataPoint(sessionFilePath, dataPoint);
            appendDataPoint(masterFilePath, dataPoint);

            return true;
        } catch (IOException e) {
            return false;
        }
    }

    /**
     * Start a new session (archives current session file)
     */
    public void startNewSession() {
        try {
            File sessionFile = new File(sessionFilePath);
            if (sessionFile.exists() && sessionFile.length() > 0) {
                // Archive current session with timestamp
                String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
                String archivePath = sessionFilePath.replace(".csv", "_" + timestamp + ".csv");
                Files.move(Paths.get(sessionFilePath), Paths.get(archivePath));
            }

            // Create new session file
            writeHeader(sessionFilePath);
        } catch (IOException e) {
        }
    }

    private void writeHeader(String filePath) throws IOException {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filePath))) {
            writer.write(ShooterDataPoint.getCSVHeader());
            writer.newLine();
        }
    }

    private void appendDataPoint(String filePath, ShooterDataPoint dataPoint) throws IOException {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filePath, true))) {
            writer.write(dataPoint.toCSV());
            writer.newLine();
        }
    }

    /**
     * Get the current session file path
     */
    public String getSessionFilePath() {
        return sessionFilePath;
    }

    /**
     * Get the master data file path
     */
    public String getMasterFilePath() {
        return masterFilePath;
    }
}
