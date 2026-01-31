package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;

public final class Tuning {
    private Tuning() {
        throw new IllegalCallerException("Cannot instantiate `Tuning`");
    }

    public static final LoggedNetworkNumber flywheelVelocity = new LoggedNetworkNumber("Tuning/FlywheelSpeed", 0);

    public static final LoggedNetworkNumber hoodAngle =
            new LoggedNetworkNumber("Tuning/HoodAngle", Constants.Shooter.hoodAngle.in(Units.Degrees));

    public static final LoggedNetworkNumber releaseVelocity = new LoggedNetworkNumber(
            "Tuning/releaseVelocity", Constants.Shooter.releaseVelocity.in(Units.InchesPerSecond));

    public static final LoggedNetworkBoolean publishData = new LoggedNetworkBoolean("Tuning/PublishData", false);

    public static final LoggedNetworkNumber hopperVelocity = new LoggedNetworkNumber("Tuning/IntakeVelocity", Constants.HopperFloor.intakeVelocity.in(Units.RotationsPerSecond));

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
