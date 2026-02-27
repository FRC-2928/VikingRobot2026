// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class CenterLimelight extends Command {
    private Transform2d offset;
    private Pose2d targetPose;
    private double xSpeedPid;
    private double ySpeedPid;
    private double thetaPid;
    private PIDController centerPIDx;
    private PIDController centerPIDy;
    private PIDController centerRotaionPid;
    private final Distance halfRobotWidth = Units.Inches.of(20); // TODO pull from constants
    private List<Pose2d> posesToCheck;
    private static final List<Integer> ladderTags = List.of(15, 31); // tags in the center of the ladder on each side
    private final CommandSwerveDrivetrain mDrivetrain;


    public static CenterLimelight ToLadderLeft(CommandSwerveDrivetrain drivetrain) {
        return new CenterLimelight(Units.Inches.of(30), Units.Inches.of(36), Units.Degrees.of(90), ladderTags, drivetrain);
    }

    public static CenterLimelight ToLadderRight(CommandSwerveDrivetrain drivetrain) {
        return new CenterLimelight(Units.Inches.of(30), Units.Inches.of(-36), Units.Degrees.of(-90), ladderTags, drivetrain);
    }

    

    public CenterLimelight(
            Distance offsetX, Distance offsetY, final List<Integer> tagsToCheck, CommandSwerveDrivetrain drivetrain) {
        this(offsetX, offsetY, Units.Radians.of(0), tagsToCheck, drivetrain);
    }

    public CenterLimelight(
            Distance offsetX,
            Distance offsetY,
            Angle offsetTheta,
            final List<Integer> tagsToCheck,
            CommandSwerveDrivetrain drivetrain) {
        mDrivetrain = drivetrain;
        this.addRequirements(mDrivetrain);
        this.offset = new Transform2d(
            offsetX.plus(Constants.Drivetrain.halfRobotWidthBumpersOn), // x offset + half robot width, so offset of 0 means the bumper will touch the tag
            offsetY,                                                    // y offset, so offset of 0 means the center of robot will be aligned with center of tag in left/right direction
            new Rotation2d(offsetTheta).plus(Rotation2d.kPi));          // theta offset + half rotation (pi radians), so offset of 0 means robot front will face the tag
        this.centerPIDx = Constants.Drivetrain.Auto.centerLimelight.createController();
        this.centerPIDy = Constants.Drivetrain.Auto.centerLimelight.createController();
        this.centerRotaionPid = Constants.Drivetrain.Auto.centerTheta.createController();
        this.centerRotaionPid.enableContinuousInput(-Math.PI, Math.PI);
        this.posesToCheck = tagsToCheck.stream()    // Using Java stream to convert list of integer (tag IDs) to list of Pose2d (tag poses)
                                .map(Constants.FIELD_LAYOUT::getTagPose)
                                .filter(Optional::isPresent)
                                .map(Optional::get)
                                .map(Pose3d::toPose2d)
                                .toList();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.targetPose = mDrivetrain.getCurrentPose2D().nearest(posesToCheck).plus(this.offset);
        Logger.recordOutput("Drivetrain/Auto/targetPose", this.targetPose);
    }

    @Override
    public void execute() {
        Pose2d robotPose = mDrivetrain.getCurrentPose2D();
        this.xSpeedPid = centerPIDx.calculate(robotPose.getX(), this.targetPose.getX());
        this.ySpeedPid = centerPIDy.calculate(robotPose.getY(), this.targetPose.getY());
        this.thetaPid = centerRotaionPid.calculate(robotPose.getRotation().getRadians(), this.targetPose.getRotation().getRadians());
        mDrivetrain.controlRobotDrivetrainAutonomus(
            ChassisSpeeds.fromFieldRelativeSpeeds(this.xSpeedPid, this.ySpeedPid, this.thetaPid, robotPose.getRotation()));

        var limelight = mDrivetrain.limelightLeft;
        Logger.recordOutput("Drivetrain/Auto/Center Is Finished", false);
        Logger.recordOutput("Drivetrain/Auto/XSpeedPid", this.xSpeedPid);
        Logger.recordOutput("Drivetrain/Auto/YSpeedPid", this.ySpeedPid);
        Logger.recordOutput("Drivetrain/Auto/thetaPid", this.thetaPid);
        Logger.recordOutput("Drivetrain/Auto/limelightHasValidTargets", limelight.hasValidTargets());
        Logger.recordOutput(
                "Drivetrain/Auto/Theta",
                limelight.getBotPose3d_TargetSpace().getRotation().getAngle());
        Logger.recordOutput(
                "Drivetrain/Auto/estRotation", mDrivetrain.getCurrentPose2D().getRotation());
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
        Pose2d robotPose = mDrivetrain.getCurrentPose2D();
        boolean isCloseToTarget = robotPose.getTranslation().getDistance(this.targetPose.getTranslation()) < 0.01; // Within 1 cm of target translation
        boolean isCorrectRotation = MathUtil.isNear(targetPose.getRotation().getDegrees(), robotPose.getRotation().getDegrees(), 0.5); // Within 0.5 degrees of correct rotation
        return isCloseToTarget && isCorrectRotation;
        // return (Math.abs(this.xSpeedPid) < 0.09) && (Math.abs(this.ySpeedPid) < 0.2) && (Math.abs(this.thetaPid) < 0.15);
        // TODO decide if we need PID check for isFinished()
        // Only needed if otherwise robot is going too fast when the command ends
    }
}
