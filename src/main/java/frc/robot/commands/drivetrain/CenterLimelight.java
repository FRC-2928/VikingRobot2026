// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterLimelight extends Command {
    /** Creates a new centerLimelight. */
    private Distance offsetX;

    private Distance offsetY;
    private Angle offsetTheta;
    private double xSpeed;
    private double xSpeedPid;
    private double ySpeed;
    private double ySpeedPid;
    private double thetaSpeed;
    private double thetaPid;
    private PIDController centerPIDx;
    private PIDController centerPIDy;
    private PIDController centerRotaionPid;
    private final Distance halfRobotWidth = Units.Inches.of(20);
    private Pose2d robotPoseTagspace;
    private Pose2d tagPoseRobotspace;
    private int tag;
    private Pose3d tagPose;
    private List<Integer> tagsToCheck;
    private static final List<Integer> reefTags = List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    private final CommandSwerveDrivetrain mDrivetrain;

    public CenterLimelight(CommandSwerveDrivetrain drivetrain) {
        mDrivetrain = drivetrain;
        // Use addRequirements() here to declare subsystem dependencies.
        this.addRequirements(drivetrain);
        this.offsetX = halfRobotWidth;
        this.offsetY = Units.Meters.of(0);
        this.offsetTheta = Units.Radians.of(Math.PI);
        this.centerPIDx = Constants.Drivetrain.Auto.centerLimelight.createController();
        this.centerPIDy = Constants.Drivetrain.Auto.centerLimelight.createController();
        this.centerRotaionPid = Constants.Drivetrain.Auto.centerTheta.createController();
        this.centerRotaionPid.enableContinuousInput(-Math.PI, Math.PI);
        this.tagsToCheck = new ArrayList<>();
    }

    public static CenterLimelight CenterLimelightLeft(CommandSwerveDrivetrain drivetrain) {
        return new CenterLimelight(Units.Feet.of(0), Units.Inches.of(-6.5), reefTags, drivetrain);
    }

    public static CenterLimelight CenterLimelightRight(CommandSwerveDrivetrain drivetrain) {
        return new CenterLimelight(Units.Feet.of(0), Units.Inches.of(6.5), reefTags, drivetrain);
    }

    public static CenterLimelight CenterLimelightRightRotated(CommandSwerveDrivetrain drivetrain) {
        // return new
        // CenterLimelight(Units.Feet.of(0).plus(Constants.Drivetrain.halfRobotWidthBumpersOn),Units.Inches.of(6.5).plus(Constants.Drivetrain.halfRobotWidthBumpersOn), Units.Degrees.of(45), reefTags);
        return new CenterLimelight(Units.Feet.of(0), Units.Inches.of(6.5), Units.Degrees.of(180), reefTags, drivetrain);
    }

    public static CenterLimelight CenterLimelightCenter(CommandSwerveDrivetrain drivetrain) {
        return new CenterLimelight(Units.Feet.of(0), Units.Inches.of(0), reefTags, drivetrain);
    }

    public static CenterLimelight CenterLimelightC(CommandSwerveDrivetrain drivetrain) {
        return new CenterLimelight(Units.Inches.of(10), Units.Inches.of(-6.5), List.of(8, 17), drivetrain);
    }

    public static CenterLimelight CenterLimelightD(CommandSwerveDrivetrain drivetrain) {
        return new CenterLimelight(Units.Inches.of(10), Units.Inches.of(6.5), List.of(8, 17), drivetrain);
    }

    public static CenterLimelight CenterLimelightF(CommandSwerveDrivetrain drivetrain) {
        return new CenterLimelight(Units.Inches.of(10), Units.Inches.of(6.5), List.of(9, 22), drivetrain);
    }

    public static CenterLimelight CenterLimelightB1Reverse(CommandSwerveDrivetrain drivetrain) {
        return new CenterLimelight(
                Units.Inches.of(10), Units.Inches.of(-15), Units.Radians.of(Math.PI), List.of(2, 12), drivetrain);
    }

    public static CenterLimelight CenterLimelightB2Reverse(CommandSwerveDrivetrain drivetrain) {
        return new CenterLimelight(
                Units.Inches.of(10), Units.Inches.of(-8), Units.Radians.of(Math.PI), List.of(2, 12), drivetrain);
    }

    public CenterLimelight(
            Distance offsetX, Distance offsetY, final List<Integer> tagsToCheck, CommandSwerveDrivetrain drivetrain) {
        this(offsetX, offsetY, Units.Radians.of(0), tagsToCheck, drivetrain);
        // this.addRequirements(Robot.cont.drivetrain);
        // this.offsetX = offsetX.plus(halfRobotWidth);
        // this.offsetY = offsetY;
        // this.offsetTheta = Units.Radians.of(Math.PI);
        // this.centerPIDx = Constants.Drivetrain.Auto.centerLimelight.createController();
        // this.centerPIDy = Constants.Drivetrain.Auto.centerLimelight.createController();
        // this.centerRotaionPid = Constants.Drivetrain.Auto.centerTheta.createController();
        // this.centerRotaionPid.enableContinuousInput(-Math.PI, Math.PI);
        // this.tagsToCheck = tagsToCheck;
    }

    public CenterLimelight(
            Distance offsetX,
            Distance offsetY,
            Angle offsetTheta,
            final List<Integer> tagsToCheck,
            CommandSwerveDrivetrain drivetrain) {
        mDrivetrain = drivetrain;
        this.addRequirements(mDrivetrain);
        this.offsetX = offsetX.plus(Constants.Drivetrain.halfRobotWidthBumpersOn);
        this.offsetY = offsetY;
        // this.offsetX = offsetX.plus(halfRobotWidth.times(Math.cos(Math.PI + offsetTheta.in(Units.Radians))));
        // this.offsetY = offsetY.plus(halfRobotWidth.times(Math.sin(Math.PI + offsetTheta.in(Units.Radians))));
        // this.offsetX =
        // offsetX.times(Math.cos(offsetTheta.in(Units.Radians))).minus(offsetY.times(Math.sin(offsetTheta.in(Units.Radians))));
        // this.offsetY =
        // offsetX.times(Math.sin(offsetTheta.in(Units.Radians))).plus(offsetY.times(Math.cos(offsetTheta.in(Units.Radians))));
        this.offsetTheta = offsetTheta.plus(Units.Radians.of(Math.PI));
        this.centerPIDx = Constants.Drivetrain.Auto.centerLimelight.createController();
        this.centerPIDy = Constants.Drivetrain.Auto.centerLimelight.createController();
        this.centerRotaionPid = Constants.Drivetrain.Auto.centerTheta.createController();
        this.centerRotaionPid.enableContinuousInput(-Math.PI, Math.PI);
        this.tagsToCheck = tagsToCheck;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double smallst = Double.MAX_VALUE;
        tagPose = Constants.FIELD_LAYOUT.getTagPose(17).get();
        for (int tag : tagsToCheck) {
            Pose2d distance =
                    Constants.FIELD_LAYOUT.getTagPose(tag).get().toPose2d().relativeTo(mDrivetrain.getCurrentPose2D());
            if (Math.hypot(distance.getX(), distance.getY()) < smallst) {
                tagPose = Constants.FIELD_LAYOUT.getTagPose(tag).get();
                smallst = Math.hypot(distance.getX(), distance.getY());
            }
        }
    }

    @Override
    public void execute() {
        Pose2d robotPose = mDrivetrain.getCurrentPose2D();
        robotPoseTagspace = robotPose.relativeTo(tagPose.toPose2d());
        tagPoseRobotspace = tagPose.toPose2d().relativeTo(robotPose);
        // xSpeed = tagPoseRobotspace.getX();
        // ySpeed = tagPoseRobotspace.getY();
        // thetaSpeed = tagPoseRobotspace.getRotation().getRadians();
        xSpeed = robotPoseTagspace.getX();
        ySpeed = robotPoseTagspace.getY();
        thetaSpeed = robotPoseTagspace.getRotation().getRadians();
        // xSpeedPid = -centerPIDx.calculate(xSpeed,offsetX.in(Units.Meters));
        // ySpeedPid = -centerPIDy.calculate(ySpeed,offsetY.in(Units.Meters));
        // thetaPid = -centerRotaionPid.calculate(thetaSpeed,offsetTheta.in(Units.Radians));
        xSpeedPid = centerPIDx.calculate(xSpeed, offsetX.in(Units.Meters));
        ySpeedPid = centerPIDy.calculate(ySpeed, offsetY.in(Units.Meters));
        double xSpeedRotated = xSpeedPid * Math.cos(offsetTheta.in(Units.Radians))
                - ySpeedPid * Math.sin(offsetTheta.in(Units.Radians));
        double ySpeedRotated = xSpeedPid * Math.sin(offsetTheta.in(Units.Radians))
                + ySpeedPid * Math.cos(offsetTheta.in(Units.Radians));
        thetaPid = centerRotaionPid.calculate(thetaSpeed, offsetTheta.in(Units.Radians));
        mDrivetrain.controlRobotDrivetrainAutonomus(new ChassisSpeeds(xSpeedRotated, ySpeedRotated, thetaPid * 1.5));

        var limelight = mDrivetrain.limelight;
        Logger.recordOutput("Drivetrain/Auto/XSpeed", xSpeed);
        Logger.recordOutput("Drivetrain/Auto/YSpeed", ySpeed);
        Logger.recordOutput("Drivetrain/Auto/Center Is Finished", false);
        Logger.recordOutput("Drivetrain/Auto/XSpeedPid", xSpeedPid);
        Logger.recordOutput("Drivetrain/Auto/YSpeedPid", ySpeedPid);
        Logger.recordOutput("Drivetrain/Auto/limelightHasValidTargets", limelight.hasValidTargets());
        Logger.recordOutput(
                "Drivetrain/Auto/Theta",
                limelight.getBotPose3d_TargetSpace().getRotation().getAngle());
        Logger.recordOutput("Drivetrain/Auto/robotPoseTagSpace", robotPoseTagspace);
        Logger.recordOutput("Drivetrain/Auto/tagPoseRobotSpace", tagPoseRobotspace);
        Logger.recordOutput("Drivetrain/Auto/thetaSpeed", thetaSpeed);
        Logger.recordOutput("Drivetrain/Auto/thetaPid", thetaPid);
        Logger.recordOutput(
                "Drivetrain/Auto/estRotation", mDrivetrain.getCurrentPose2D().getRotation());
        Logger.recordOutput("Drivetrain/Auto/offsetX", offsetX);
        Logger.recordOutput("Drivetrain/Auto/offsetTheta", offsetTheta.in(Units.Radians));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDrivetrain.halt();
        Logger.recordOutput("Drivetrain/Auto/Center Is Finished", true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(xSpeedPid) < 0.09) && (Math.abs(ySpeedPid) < 0.2) && (Math.abs(thetaPid) < 0.15);
    }
}
