package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.pathplanner.lib.config.PIDConstants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    private static Mode currentMode() {
        if (Robot.isReal()) return Mode.REAL;
        if (!Logger.hasReplaySource()) return Mode.SIM;
        else return Mode.REPLAY;
    }

    private Constants() {
        throw new IllegalCallerException("Cannot instantiate `Constants`");
    }

    public static final Mode mode = Constants.currentMode();
    public static final boolean real = Constants.mode == Constants.Mode.REAL;

    public static final AudioConfigs talonFXAudio = new AudioConfigs()
            .withAllowMusicDurDisable(true)
            .withBeepOnBoot(true)
            .withBeepOnConfig(true);

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double mod(final double lhs, final double rhs) {
        return (lhs % rhs + rhs) % rhs;
    }

    public static final double angleNorm(final double angle) {
        return Constants.mod(angle + 180, 360) - 180;
    }

    public static final double angleDistance(final double a, final double b) {
        return Math.abs(
                Constants.angleNorm(180 - Math.abs(Math.abs(Constants.angleNorm(a) - Constants.angleNorm(b)) - 180)));
    }

    public static record PIDValues(double p, double i, double d, double f) {
        public final PIDController createController() {
            return new PIDController(this.p, this.i, this.d);
        }

        public final ProfiledPIDController createProfiledController(final TrapezoidProfile.Constraints profile) {
            return new ProfiledPIDController(this.p, this.i, this.d, profile);
        }
    }

    public static PIDConstants fromPIDValues(final PIDValues pid) {
        return new PIDConstants(pid.p, pid.d, pid.d);
    }

    // public static final Distance fieldWidth = Units.Meters.of(16.541); // Correlates to Field oriented x coordinate
    // public static final Distance fieldDepth = Units.Meters.of(8.211); // Correlates to Field oriented y coordinate

    public static final AprilTagFieldLayout FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final class LimelightFX {
        private LimelightFX() {
            throw new IllegalCallerException("Cannot instantiate `Constants.LimelightFX`");
        }

        public static final class Colors {
            private Colors() {
                throw new IllegalCallerException("Cannot instantiate `Constants.LimelightFX.Colors`");
            }
        }

        public static final boolean enabled = true;
    }

    public static final class CAN {
        private CAN() {
            throw new IllegalCallerException("Cannot instantiate `Constants.CAN`");
        }

        public static final class Misc {
            public static final int pdh = 0;

            public static final int feederLauncher = 0;
            public static final int intakeRoller = 3;
        }

        public static final class CTRE {
            public static final CANBus bus = new CANBus("canivore");

            public static final int pigeon = 0;

            public static final int swerveFrontLeftAzimuth = 16;
            public static final int swerveFrontLeftDrive = 15;

            public static final int swerveFrontRightAzimuth = 3;
            public static final int swerveFrontRightDrive = 4;

            public static final int swerveBackLeftAzimuth = 17;
            public static final int swerveBackLeftDrive = 18;

            public static final int swerveBackRightAzimuth = 1;
            public static final int swerveBackRightDrive = 2;

            public static final int uptake = 1; // TODO: find a good place for uptake and hood
            public static final int hood = 1;
            public static final int shooterFlywheelA = 1;
            public static final int shooterFlywheelB = 4;

            public static final int climber = 17;
        }
    }

    public static final class PWM {
        private PWM() {
            throw new IllegalCallerException("Cannot instantiate `Constants.PWM`");
        }

        public static final int climberRatchet = 9;

        public static final int ampBarServoA = 0;
        public static final int ampBarServoB = 1;
    }

    public static final class DIO {
        private DIO() {
            throw new IllegalCallerException("Cannot instantiate `Constants.DIO`");
        }

        public static final int release = 0;
        public static final int lockout = 1;
    }

    public static final class Drivetrain {
        private Drivetrain() {
            throw new IllegalCallerException("Cannot instantiate `Constants.Drivetrain`");
        }

        public static final class Auto {
            public static final PIDValues translationDynamic = /*new PIDValues(7.5, 0, 0.5, 0);*/
                    new PIDValues(0, 0, 0, 0);
            public static final PIDValues thetaDynamic = /*new PIDValues(5, 0, 0.0, 0);*/ new PIDValues(0, 0, 0, 0);
            public static final PIDValues centerLimelight = new PIDValues(2, 0, 0, 0);
            public static final PIDValues centerTheta = new PIDValues(4, 0, 0.2, 0);
        }

        public static final SlotConfigs azimuth =
                new SlotConfigs().withKP(-50).withKI(0).withKD(-0.5).withKS(-2);
        // .withKV(0)
        // .withKA(-0.5);

        public static final SlotConfigs drive = new SlotConfigs()
                .withKP(0) /* 0.15 */
                .withKI(0.0)
                .withKD(0)
                .withKS(0)
                .withKV(12.0 / Units.FeetPerSecond.of(15.5).in(Units.MetersPerSecond))
                .withKA(0.25);

        // todo: tune
        public static final PIDValues drivePID = new PIDValues(0.1, 0, 0, 0);
        // public static final PIDValues swerveAzimuthPID = new PIDValues(0.01, 0, 0.005, 0);
        public static final PIDValues absoluteRotationPID = new PIDValues(2.3, 0, 0.15, 0);
        public static final TrapezoidProfile.Constraints absoluteRotationConstraints =
                new TrapezoidProfile.Constraints(1, 17);
        public static final SimpleMotorFeedforward absoluteRotationFeedforward = new SimpleMotorFeedforward(2, 1);
        // todo: find
        public static final SimpleMotorFeedforward driveFFW = new SimpleMotorFeedforward(0, 2.5, 0);

        public static final double thetaCompensationFactor = 0.2;

        public static final Distance wheelBase = Units.Inches.of(27 - 2.5 * 2);
        public static final Distance trackWidth = Drivetrain.wheelBase; // For a square drivetrain
        public static final Distance halfRobotWidth = Units.Inches.of(27.0 / 2);
        public static final Distance halfRobotWidthBumpersOn = Units.Inches.of(27.0 / 2 + 3);

        // public static final Angle swerveFrontLeftOffset = Units.Rotations.of(0.227783);
        // public static final Angle swerveFrontLeftOffset = Units.Rotations.of(0.349609375);
        public static final Angle swerveFrontLeftOffset = Units.Rotations.of(-0.385009765625);
        public static final Translation2d swerveFrontLeftTranslation =
                new Translation2d(Constants.Drivetrain.wheelBase, Constants.Drivetrain.trackWidth);
        // public static final Angle swerveFrontRightOffset = Units.Rotations.of(-0.150146484375);
        public static final Angle swerveFrontRightOffset = Units.Rotations.of(-0.38671875);
        public static final Translation2d swerveFrontRightTranslation =
                new Translation2d(Constants.Drivetrain.wheelBase, Constants.Drivetrain.trackWidth.unaryMinus());
        // public static final Angle swerveBackLeftOffset = Units.Rotations.of(-0.136474609375);
        public static final Angle swerveBackLeftOffset = Units.Rotations.of(-0.19384765625);
        public static final Translation2d swerveBackLeftTranslation =
                new Translation2d(Constants.Drivetrain.wheelBase.unaryMinus(), Constants.Drivetrain.trackWidth);
        // public static final Angle swerveBackRightOffset = Units.Rotations.of(-0.4404296875);
        public static final Angle swerveBackRightOffset = Units.Rotations.of(-0.404296875);
        public static final Translation2d swerveBackRightTranslation = new Translation2d(
                Constants.Drivetrain.wheelBase.unaryMinus(), Constants.Drivetrain.trackWidth.unaryMinus());

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                Constants.Drivetrain.swerveFrontLeftTranslation,
                Constants.Drivetrain.swerveFrontRightTranslation,
                Constants.Drivetrain.swerveBackLeftTranslation,
                Constants.Drivetrain.swerveBackRightTranslation);

        // Gear ratios for SDS MK4i L2, adjust as necessary
        // public static final double driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // ~= 6.746
        public static final double driveGearRatio = (50.0 / 14) * (16.0 / 28) * (45.0 / 15); // ~= 6.746
        public static final double azimuthGearRatio = 150.0 / 7.0;

        public static final Distance wheelRadius = Units.Inches.of(1.85) /*Units.Inches.of(1.875)*/;
        public static final Distance wheelCircumference = Drivetrain.wheelRadius.times(2 * Math.PI);

        public static final LinearVelocity maxVelocity = Units.FeetPerSecond.of(15.5); // MK4i max speed L2

        // max angular velocity computes to 6.41 radians per second
        public static final AngularVelocity maxAngularVelocity =
                Units.RotationsPerSecond.of(Drivetrain.maxVelocity.in(Units.MetersPerSecond)
                        / (2
                                * Math.PI
                                * Math.hypot(
                                        Drivetrain.trackWidth.div(2).in(Units.Meters),
                                        Drivetrain.wheelBase.div(2).in(Units.Meters))));
    }

    public static class Shooter {
        private Shooter() {
            throw new IllegalCallerException("Cannot instantiate `Constants.Shooter`");
        }

        public static final Angle hoodAngle = Units.Degrees.of(0);
        public static final LinearVelocity releaseVelocity = Units.FeetPerSecond.of(0);
    }

    public static class Climber {
        private Climber() {
            throw new IllegalCallerException("Cannot instantiate `Constants.Climber`");
        }

        public static final boolean ratchetEnabled = false;

        public static final SlotConfigs configFast = new SlotConfigs().withKP(0.25);
        public static final SlotConfigs configSlow = new SlotConfigs().withKP(0.01);

        public static final double ratchetLocked = 84;
        public static final double ratchetFree = 93;

        public static final double max = 129;
        public static final double disengageDistance = 0.5;
        public static final double initializeRaiseDistance = 2;
    }
}
