package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.Units;

public final class Tuning {
    private Tuning() {
        throw new IllegalCallerException("Cannot instantiate `Tuning`");
    }

    public static final LoggedNetworkNumber intakeSpeed = new LoggedNetworkNumber("Tuning/SpeedIntakePercent", .8);

    public static final LoggedNetworkNumber hoodAngle = new LoggedNetworkNumber("Tuning/HoodAngleDegrees", 0.0);
    public static final LoggedNetworkNumber flywheelSpeed = new LoggedNetworkNumber("Tuning/FlywheelSpeedRPS", 0.0);
    public static final LoggedNetworkNumber kickerSpeed = new LoggedNetworkNumber("Tuning/KickerSpeed", 0.0);

    public static final LoggedNetworkNumber drivetrainP = new LoggedNetworkNumber("Tuning/Drivetrain P", 0.15);
    public static final LoggedNetworkNumber shootSpeakerPivotThreshold =
            new LoggedNetworkNumber("Tuning/ShootSpeakerPivotThreshold", 1.25);
    public static final LoggedNetworkNumber shootSpeakerExponent =
            new LoggedNetworkNumber("Tuning/ShootSpeakerExponent", 1);

    public static final LoggedNetworkNumber ferryAngle =
            new LoggedNetworkNumber("Tuning/FerryAngle", Constants.Shooter.shootFerry.in(Units.Degrees));
    public static final LoggedNetworkNumber subAngle =
            new LoggedNetworkNumber("Tuning/SubAngle", Constants.Shooter.shootSub.in(Units.Degrees));
}
