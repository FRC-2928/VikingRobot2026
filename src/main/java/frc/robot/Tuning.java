package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class Tuning {
    private Tuning() {
        throw new IllegalCallerException("Cannot instantiate `Tuning`");
    }

    public static final LoggedNetworkNumber flywheelVelocity = new LoggedNetworkNumber("Tuning/FlywheelSpeed", 0);

    public static final LoggedNetworkNumber hoodAngle = new LoggedNetworkNumber("Tuning/HoodAngleDegrees", 0.0);
    public static final LoggedNetworkNumber flywheelSpeed = new LoggedNetworkNumber("Tuning/FlywheelSpeedRPS", 0.0);
    public static final LoggedNetworkNumber kickerSpeed = new LoggedNetworkNumber("Tuning/KickerSpeed", 3.0);

    public static final LoggedNetworkNumber intakeSpeed = new LoggedNetworkNumber("Tuning/IntakeSpeed", 0.8);

    public static final LoggedNetworkNumber drivetrainP = new LoggedNetworkNumber("Tuning/Drivetrain P", 0.15);
    public static final LoggedNetworkNumber shootSpeakerPivotThreshold =
            new LoggedNetworkNumber("Tuning/ShootSpeakerPivotThreshold", 1.25);
    public static final LoggedNetworkNumber shootSpeakerExponent =
            new LoggedNetworkNumber("Tuning/ShootSpeakerExponent", 1);

    public static final LoggedNetworkNumber releaseVelocity = new LoggedNetworkNumber(
            "Tuning/releaseVelocity", Constants.Shooter.releaseVelocity.in(Units.InchesPerSecond));

    public static final LoggedNetworkBoolean publishData = new LoggedNetworkBoolean("Tuning/PublishData", false);

    public static final LoggedNetworkNumber hopperVelocity = new LoggedNetworkNumber(
            "Tuning/hopperVelocity", Constants.HopperFloor.hopperVelocity.in(Units.RotationsPerSecond));

    public static final LoggedNetworkNumber intakeVelocity = new LoggedNetworkNumber(
            "Tuning/IntakeVelocity", Constants.Intake.intakeVelocity.in(Units.DegreesPerSecond));

    public static void writeToCSV(double hAngle, double rVelocity, Distance distanceFromTarget)
            throws FileNotFoundException {
        File csvOutputFile = new File(Filesystem.getOperatingDirectory().getAbsolutePath() + "/ShootingTuner.csv");
        Boolean fileExists = csvOutputFile.exists();
        try (PrintWriter pw = new PrintWriter(new FileOutputStream(csvOutputFile, true))) {
            if (!fileExists) {
                pw.println("hoodAngle, releaseVelocity, distanceFromTarget");
            }
            pw.append(String.format(" %f, %f, %f \n", hAngle, rVelocity, distanceFromTarget.in(Units.Inches)));
        }
    }
}
