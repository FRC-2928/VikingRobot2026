package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.Units;

public final class Tuning {
    private Tuning() {
        throw new IllegalCallerException("Cannot instantiate `Tuning`");
    }

    public static final LoggedNetworkNumber intakeSpeed = new LoggedNetworkNumber("Tuning/SpeedIntakePercent", .8);

    public static final LoggedNetworkNumber flywheelVelocity = new LoggedNetworkNumber(
            "Tuning/FlywheelSpeed", Constants.Shooter.flywheels.speakerVelocity.in(Units.RotationsPerSecond));
    public static final LoggedNetworkNumber flywheelVelocityThreshold = new LoggedNetworkNumber(
            "Tuning/FlywheelSpeedThreshold",
            Constants.Shooter.flywheels.speakerVelocityThreshold.in(Units.RotationsPerSecond));

    public static final LoggedNetworkNumber ampAngle =
            new LoggedNetworkNumber("Tuning/AmpAngle", Constants.Shooter.shootAmp.in(Units.Degrees));
    public static final LoggedNetworkNumber ampPower =
            new LoggedNetworkNumber("Tuning/AmpPower", Constants.Shooter.flywheels.ampPower);
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
