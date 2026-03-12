package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
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
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); // Our district and champs both use welded field layout, not AndyMark

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
        }

        public static final class RIO {
			public static final CANBus bus = new CANBus("rio");
            public static final int intakeRoller = 0;
		}

        public static final class CTRE {
            public static final CANBus bus = new CANBus("canivore");


            public static final int kicker = 16; // TODO: find a good place for kicker and hood
            public static final int hood = 25;
            public static final int shooterFlywheelA = 27;
            public static final int shooterFlywheelB = 26;

            public static final int climber = 24;

            public static final int hopper = 23;
            public static final int indexer = 31;

            public static final int intakeExpansion = 30;
            

            public static final int candle = 99;
        }

        public static final class INTAKE_CANDI {
            /// Singleton instance of the CANdi for the Intake
            private static CANdi sInstance = null;
            /// CAN ID of the CANdi bridging the intake limit switch to the CAN bus
            private static final int canId = 32;
            /// Digital Inputs Configs for the CANdi
            private static final DigitalInputsConfigs dioConfigs = new DigitalInputsConfigs()
                    // S1In --> Expansion??
                    .withS1CloseState(
                            S1CloseStateValue.CloseWhenLow) // Intake Expansion limit switch -- closed when low
                    .withS1FloatState(S1FloatStateValue.PullHigh) // Intake Expansion limit switch -- high when open
                    // S2In --> Retraction??
                    .withS2CloseState(
                            S2CloseStateValue.CloseWhenLow) // Intake Retraction limit switch -- closed when low
                    .withS2FloatState(S2FloatStateValue.PullHigh); // Intake Retraction limit switch -- high when open

            public static synchronized CANdi getInstance() {
                if (sInstance != null) {
                    return sInstance;
                }

                // create the CANdi instance
                sInstance = new CANdi(canId, CTRE.bus);
                // set up the configuration with the internal config we defined above
                CANdiConfiguration candiConfig = new CANdiConfiguration().withDigitalInputs(INTAKE_CANDI.dioConfigs);
                sInstance.getConfigurator().apply(candiConfig);

                // Set the update frequency for the status frames for the CANdi signals
                BaseStatusSignal.setUpdateFrequencyForAll(
                        Units.Hertz.of(100), sInstance.getS1State(), sInstance.getS2State());
                return sInstance;
            }
        }
    }

    public static final class FIELD {
        private FIELD() {
            throw new IllegalCallerException("Cannot Instantiate 'Constants.FIELD'");
        }

        public static final Distance fieldLength = Units.Inch.of(651.22);
        public static final Distance distanceToMidField = Units.Inch.of(182.11);
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

            // Auto align constants
            public static final PIDValues centerLimelight = new PIDValues(10, 0, 0, 0);
            public static final PIDValues centerTheta = new PIDValues(10, 0, 0, 0);
            public static final Distance positionThreshold = Units.Centimeters.of(1);
            public static final Angle headingThreshold = Units.Degrees.of(0.5);
            public static final LinearVelocity linearSpeedThreshold = Units.MetersPerSecond.of(0.2);
            public static final AngularVelocity angularSpeedThreshold = Units.DegreesPerSecond.of(10);
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

        public static final Angle shooterAngleOffsetFromFront = Units.Degrees.of(90);
        //hoodAngle, velocity, distance
        public static final double[][] temporaryLookupTable = {{75, 240, 60}, {70, 245, 72}, {68, 245, 84}, {61, 258, 108}, {57, 270, 132}, {54, 282, 156}, {54, 302, 180}, {53, 317, 204}, {53, 328, 228}, {52, 243, 252}};

        // Gear Ratios
        public static final double flywheelGearRatio = 1.0;
        public static final double hoodGearRatio = 4.0 * 157.0 / 14;
        public static final double kickerGearRatio = 1.0;

        public static final Angle hoodAngle = Units.Degrees.of(0);
        public static final LinearVelocity releaseVelocity = Units.FeetPerSecond.of(0);
        public static final Angle toleranceFromHub = Units.Degrees.of(10);
        public static final AngularVelocity shooterVelocityTolerance = Units.RotationsPerSecond.of(5);
        public static final Angle hoodAngleTolerance = Units.Degrees.of(3);
        public static final double pivotCurrentLimit = 40;
        public static final AngularVelocity pivotMaxVelocityShoot = Units.DegreesPerSecond.of(2);
        public static InterpolatingTreeMap<Double, AimValues> lookUpTable =
                new InterpolatingTreeMap<Double, AimValues>(
                    InverseInterpolator.forDouble(),
                    (start, end, t) -> {
                        // Lerp each field: result = start + (end - start) * t
                        Angle dTheta = (end.hoodAngle.minus(start.hoodAngle)).times(t);
                        Angle interpolatedHoodAngle = start.hoodAngle.plus(dTheta);
                        AngularVelocity dOmega = (end.shooterVelocity.minus(start.shooterVelocity)).times(t);
                        AngularVelocity interpolatedFlywheelVelocity = start.shooterVelocity.plus(dOmega);
                        return new AimValues(interpolatedHoodAngle, interpolatedFlywheelVelocity);
                    });

        static {
            // Add temporary values to the tree
            // Shooter.lookUpTable.put(5.0, new AimValues(Units.Degrees.of(20), Units.RotationsPerSecond.of(50)));

            // for(double[] point: temporaryLookupTable ){
            //     Shooter.lookUpTable.put(Units.Inches.of(point[2]).in(Units.Meters), new AimValues(Units.Degrees.of(point[0]), Units.RotationsPerSecond.of(point[1]/8)));
            // }
        }

        public static class AimValues {
            public final Angle hoodAngle;
            public final AngularVelocity shooterVelocity;

            public AimValues(Angle hoodAngle, AngularVelocity shooterVelocity) {
                this.hoodAngle = hoodAngle;
                this.shooterVelocity = shooterVelocity;
            }
        }
    }

    public static class HopperFloor {
        private HopperFloor() {
            throw new IllegalCallerException("Cannot instantiate `Constants.HopperFloor`");
        }

        public static final double indexerGearRatio = 66.0 / 14.0; // Idk if this goes here

        public static final double hopperVelocity = 0;
    }

    public static class Indexer {
        private Indexer() {
            throw new IllegalCallerException("Cannot instantiate `Constants.HopperFloor`");
        }

        public static final double indexerGearRatio = 1.0;

        public static final AngularVelocity indexerVelocity = Units.RotationsPerSecond.of(1);
    }

    public static class Intake {
        private Intake() {
            throw new IllegalCallerException("Dont Call this (Constants.Intake)");
        }

        public static final Angle INTAKE_FORWARD_SOFT_LIMIT = Units.Rotations.of(2.20);
        public static final Angle INTAKE_FORWARD_HARD_LIMIT = Units.Rotations.of(2.35);
        public static final Distance INTAKE_FORWARD_DISTANCE_LIMIT = Units.Inches.of(2.20);
        public static final Distance INTAKE_RETRACTION_LIMIT = Units.Inches.of(0.818);  // TODO: figure out this value
        private static final Distance GEAR_DIAMETER = Units.Inches.of(1.5);
        private static final double INTAKE_RACK_GEARING = 2.5;
        public static final double DISTANCE_CONVERSION_RATIO = (Math.PI * GEAR_DIAMETER.in(Units.Inches)) / INTAKE_RACK_GEARING;
        public static final double INTAKE_ROLLER_GEARING = 1.0;
        public enum IntakeStates {
            FORWARD(0.4),
            REVERSE(-0.4),
            OFF(0);

            private double mSpeed;
            private IntakeStates(double speed) {
                mSpeed = speed;
            }

            public double getSpeed() {
                return mSpeed;
            }
        }
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
