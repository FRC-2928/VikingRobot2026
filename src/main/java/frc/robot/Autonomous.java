package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class Autonomous {
    public static Command bLineForwardBack(CommandSwerveDrivetrain drivetrain) {
        return Commands.sequence(drivetrain.getPathBuilder().build(new Path("forwardBack")));
    }

    public static AutoChooser getChoreoAutoChooser(RobotContainer cont) {
        final AutoChooser choreoChooser = new AutoChooser();
        AutoFactory autoFactory = cont.drivetrain.getChoreoAutoFactory();

        choreoChooser.addCmd("BLine-ForwardBack", () -> {
            final var idle = new SwerveRequest.Idle();

            var pathBuilder = cont.drivetrain.getPathBuilder();
            Path forwardBack = new Path("forwardBack");
            return Commands.sequence(
                    cont.drivetrain.runOnce(() -> cont.drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    pathBuilder.build(forwardBack),
                    cont.drivetrain.applyRequest(() -> idle));
        });

        // Comented out because these dont use the intake and we dont want to accidently run them durring a match
        /* 
        choreoChooser.addCmd("path1_shootPickShoot", () -> {
            final var idle = new SwerveRequest.Idle();

            var pathBuilder = cont.drivetrain.getPathBuilder();
            Path path1_shootPickShoot = new Path("path1_shootPickShoot");
            return Commands.sequence(
                    cont.drivetrain.runOnce(() -> cont.drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    pathBuilder.build(path1_shootPickShoot),
                    cont.mSuperstructure.shootAutomated());
        });

        choreoChooser.addCmd("path2_shootPickShoot", () -> {
            final var idle = new SwerveRequest.Idle();

            var pathBuilder = cont.drivetrain.getPathBuilder();
            Path path2_shootPickShoot = new Path("path2_shootPickShoot");
            return Commands.sequence(
                    cont.drivetrain.runOnce(() -> cont.drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    pathBuilder.build(path2_shootPickShoot),
                    cont.mSuperstructure.shootAutomated());
        });

        choreoChooser.addCmd("path3_middlePickShoot", () -> {
            final var idle = new SwerveRequest.Idle();

            var pathBuilder = cont.drivetrain.getPathBuilder();
            Path path3_middlePickShoot = new Path("path3_middlePickShoot");
            return Commands.sequence(
                    cont.drivetrain.runOnce(() -> cont.drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    pathBuilder.build(path3_middlePickShoot),
                    cont.mSuperstructure.shootAutomated());
        });
        */

        choreoChooser.addCmd(
                "Auto0_goBackwardAndShoot",
                () -> new SequentialCommandGroup(
                        // Go Backward for 10 sec
                        cont.drivetrain.driveForDuration(new ChassisSpeeds(-2, 0, 0), Units.Seconds.of(1)),
                        // Call shoot from superclass
                        cont.mSuperstructure.shootAutomated()));

        choreoChooser.addCmd("emptyPreLoad_rightPickShoot", () -> {
            final var idle = new SwerveRequest.Idle();

            var pathBuilder = cont.drivetrain.getPathBuilder();
            Path rightPickShoot_part1 = new Path("rightPickShoot_part1");
            Path rightPickShoot_part3 = new Path("rightPickShoot_part3");

            Command shootPreLoad = new SequentialCommandGroup(
                        // Go Backward for 10 sec
                        cont.drivetrain.driveForDuration(new ChassisSpeeds(-2, 0, 0), Units.Seconds.of(1)),
                        // Call shoot from superclass
                        cont.mSuperstructure.shootAutomated()).withTimeout(5);

            Command rightPickShoot =  Commands.sequence(
                    pathBuilder.build(rightPickShoot_part1),
                    cont.mSuperstructure.pathWhileIntaking("rightPickShoot_part2"),
                    pathBuilder.build(rightPickShoot_part3),
                    cont.mSuperstructure.shootAutomated());
            
            return new SequentialCommandGroup(shootPreLoad, rightPickShoot);
        });

        choreoChooser.addCmd("middlePickShoot", () -> {
            final var idle = new SwerveRequest.Idle();

            var pathBuilder = cont.drivetrain.getPathBuilder();
            Path middlePickShoot_part1 = new Path("middlePickShoot_part1");
            Path middlePickShoot_part3 = new Path("middlePickShoot_part3");
            return Commands.sequence(
                    cont.drivetrain.runOnce(() -> cont.drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    pathBuilder.build(middlePickShoot_part1),
                    cont.mSuperstructure.pathWhileIntaking("middlePickShoot_part2"),
                    pathBuilder.build(middlePickShoot_part3),
                    cont.mSuperstructure.prepareShooter());
        });

        choreoChooser.addCmd("rightPickShoot", () -> {
            final var idle = new SwerveRequest.Idle();

            var pathBuilder = cont.drivetrain.getPathBuilder();
            Path rightPickShoot_part1 = new Path("rightPickShoot_part1");
            Path rightPickShoot_part3 = new Path("rightPickShoot_part3");
            return Commands.sequence(
                    cont.drivetrain.runOnce(() -> cont.drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    pathBuilder.build(rightPickShoot_part1),
                    cont.mSuperstructure.pathWhileIntaking("rightPickShoot_part2"),
                    pathBuilder.build(rightPickShoot_part3),
                    cont.mSuperstructure.shootAutomated());
        });

        choreoChooser.addCmd("leftPickShoot", () -> {
            final var idle = new SwerveRequest.Idle();

            var pathBuilder = cont.drivetrain.getPathBuilder();
            Path leftPickShoot_part1 = new Path("leftPickShoot_part1");
            Path leftPickShoot_part3 = new Path("leftPickShoot_part3");
            return Commands.sequence(
                    cont.drivetrain.runOnce(() -> cont.drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    pathBuilder.build(leftPickShoot_part1),
                    cont.mSuperstructure.pathWhileIntaking("leftPickShoot_part2"),
                    pathBuilder.build(leftPickShoot_part3),
                    cont.mSuperstructure.shootAutomated());
        });

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
