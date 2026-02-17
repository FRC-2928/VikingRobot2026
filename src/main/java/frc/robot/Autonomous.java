package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.commands.drivetrain.VoltageRampCommand;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public final class Autonomous {
    public static SendableChooser<Command> createAutonomousChooser(RobotContainer cont) {
        final SendableChooser<Command> chooser = new SendableChooser<>();
        AutoFactory autoFactory = cont.drivetrain.getChoreAutoFactory();

        // Set global constraints before creating any paths
        Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
                Constants.Drivetrain.maxVelocity.in(Units.MetersPerSecond), // maxVelocityMetersPerSec
                // TODO: Find acutal values
                12.0, // maxAccelerationMetersPerSec2
                Constants.Drivetrain.maxAngularVelocity.in(Units.DegreesPerSecond), // maxVelocityDegPerSec
                860, // maxAccelerationDegPerSec2
                0.03, // endTranslationToleranceMeters
                2.0, // endRotationToleranceDeg
                0.2 // intermediateHandoffRadiusMeters
                ));

        chooser.addOption(
                "[Bline] Forward Back",
                Commands.sequence(cont.drivetrain.getPathBuilder().build(new Path("forwardBack"))));
        chooser.addOption(
                "[Test] Forward Back",
                new SequentialCommandGroup(
                        Autonomous.setInitialPose("forwardBack", cont.drivetrain), Autonomous.path("forwardBack")));
        chooser.addOption(
                "[Test] Forward Back Choreo",
                new SequentialCommandGroup(
                        Autonomous.setInitialPose("forwardBack", cont.drivetrain),
                        autoFactory.trajectoryCmd("forwardBack")));

        chooser.addOption("Center On Limelight", new CenterLimelight(cont.drivetrain));

        // Backs out 1 meter and shoots the balls
        chooser.addOption(
                "Auto0_goBackwardAndShoot",
                new SequentialCommandGroup(
                        // Go Backward for 10 sec
                        cont.drivetrain.driveForDuration(new ChassisSpeeds(-2, 0, 0), Units.Seconds.of(1)),
                        // Call shoot from superclass
                        cont.superstructure.readyAndShoot()));

        chooser.addOption("[testing] voltage ramp", new VoltageRampCommand(cont.drivetrain));
        return chooser;
    }

    public static Command bLineForwardBack(CommandSwerveDrivetrain drivetrain) {
        return Commands.sequence(drivetrain.getPathBuilder().build(new Path("forwardBack")));
    }

    public static AutoChooser getChoreoAutoChooser(CommandSwerveDrivetrain drivetrain) {
        final AutoChooser choreoChooser = new AutoChooser();
        AutoFactory autoFactory = drivetrain.getChoreAutoFactory();

        choreoChooser.addCmd("BLine-ForwardBack", () -> {
            final var idle = new SwerveRequest.Idle();

            var pathBuilder = drivetrain.getPathBuilder();
            Path forwardBack = new Path("forwardBack");
            return Commands.sequence(
                    drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    pathBuilder.build(forwardBack),
                    drivetrain.applyRequest(() -> idle));
        });

        choreoChooser.addCmd(
                "SimpleFromRight",
                () -> Commands.sequence(
                        // autoFactory.resetOdometry("SimpleFromRight"),
                        autoFactory.trajectoryCmd("StartToF"),
                        Commands.deadline(new WaitCommand(2), CenterLimelight.CenterLimelightF(drivetrain)),
                        autoFactory.trajectoryCmd("FToB2Reverse"),
                        Commands.deadline(new WaitCommand(2), CenterLimelight.CenterLimelightB2Reverse(drivetrain)),
                        autoFactory.trajectoryCmd("B1ReverseToC"),
                        Commands.deadline(new WaitCommand(2), CenterLimelight.CenterLimelightC(drivetrain)),
                        autoFactory.trajectoryCmd("CToB1Reverse"),
                        Commands.deadline(new WaitCommand(2), CenterLimelight.CenterLimelightB2Reverse(drivetrain)),
                        autoFactory.trajectoryCmd("B1ReverseToD"),
                        Commands.deadline(new WaitCommand(2), CenterLimelight.CenterLimelightD(drivetrain))
                        // Robot.cont.drivetrain.haltCommand()
                        ));

        choreoChooser.addCmd("SimpleScore", () -> Commands.sequence(autoFactory.trajectoryCmd("SimpleScore")));

        return choreoChooser;
    }

    public static Command setInitialPose(final String name, CommandSwerveDrivetrain drivetrain) {
        final Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(name);
        // try {
        final Pose2d initial = traj.get().getPoses()[0];

        return Commands.runOnce(() -> {
            drivetrain.resetPose(initial);

            Logger.recordOutput("Drivetrain/Auto/x0", initial.getX());
            Logger.recordOutput("Drivetrain/Auto/y0", initial.getY());
            Logger.recordOutput("Drivetrain/Auto/r0", initial.getRotation().getDegrees());
            Logger.recordOutput("Drivetrain/Auto/AllyPose", initial);
        });
        // } catch (Exception e) {
        // 	System.out.println(e.toString());
        // 	return new InstantCommand();
        // }
    }

    public static Command path(final String name) {
        try {
            final PathPlannerPath choreoPath = PathPlannerPath.fromChoreoTrajectory(name);
            return AutoBuilder.followPath(choreoPath);
        } catch (Exception e) {
            System.out.println(e.toString());
            return new InstantCommand();
        }
    }

    public static Command pathPlannerpath(final String name) {
        try {
            final PathPlannerPath path = PathPlannerPath.fromPathFile(name);
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            System.out.println(e.toString());
            return new InstantCommand();
        }
    }

    public static Command dynamic(final String next, final double maxvel) {
        final Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(next);

        return AutoBuilder.pathfindToPoseFlipped(
                        traj.get().getPoses()[0],
                        new PathConstraints(
                                maxvel, 2, Constants.Drivetrain.maxAngularVelocity.in(Units.RadiansPerSecond), 2))
                .alongWith(new InstantCommand(() -> Logger.recordOutput(
                        "Drivetrain/Auto/DynamicTarget", traj.get().getPoses()[0])));
    }

    public static Command dynamic(final String next) {
        return Autonomous.dynamic(next, Constants.Drivetrain.maxVelocity.in(Units.MetersPerSecond));
    }

    public static Command dynamicThen(final String next) {
        try {
            final PathPlannerPath traj = PathPlannerPath.fromChoreoTrajectory(next);

            return AutoBuilder.pathfindThenFollowPath(
                            traj,
                            new PathConstraints(
                                    Constants.Drivetrain.maxVelocity.in(Units.MetersPerSecond),
                                    3,
                                    Constants.Drivetrain.maxAngularVelocity.in(Units.RadiansPerSecond),
                                    2))
                    .alongWith(new InstantCommand(() -> Logger.recordOutput(
                            "Drivetrain/Auto/DynamicTarget",
                            Choreo.loadTrajectory(next).get().getPoses()[0])));
        } catch (Exception e) {
            System.out.println(e.toString());
            return new InstantCommand();
        }
    }

    /*
     * Returns the original or mirrored pose depending on alliance color (since the field is flipped)
     */
    private static Pose2d getPoseForAlliance(final Pose2d initialPose) {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            return new Pose2d(
                    Constants.FIELD_LAYOUT.getFieldLength() - initialPose.getX(),
                    Constants.FIELD_LAYOUT.getFieldWidth() - initialPose.getY(),
                    initialPose.getRotation().unaryMinus());
        } else return initialPose;
    }
}
