// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileNotFoundException;

import com.ctre.phoenix6.HootAutoReplay;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private Command mAutonomousCommand;

    private final RobotContainer mRobotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay =
            new HootAutoReplay().withTimestampReplay().withJoystickReplay();

    public Robot() {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        mRobotContainer = new RobotContainer();

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotInit() {
        mRobotContainer.drivetrain.limelight.setIMUMode(1);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        mRobotContainer.drivetrain.limelight.setThrottleRate(isEnabled() ? 0 : 100);
        try {
            if (Tuning.publishData.get()) {
                // TODO PUT IN REAL VALUES!!!!
                Tuning.writeToCSV(Tuning.hoodAngle.get(), Tuning.releaseVelocity.get(), Units.Inches.of(0));
                Tuning.publishData.set(false);
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        mRobotContainer.drivetrain.limelight.setRobotOrientation(
                mRobotContainer.drivetrain.getCurrentPose2D().getRotation().getMeasure());
    }

    @Override
    public void disabledInit() {
        mRobotContainer.drivetrain.limelight.setIMUMode(1);
    }

    @Override
    public void disabledPeriodic() {
        mRobotContainer.drivetrain.disabledPeriodic();
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        mAutonomousCommand = mRobotContainer.getAutonomousCommand();

        if (mAutonomousCommand != null) {
            CommandScheduler.getInstance().schedule(mAutonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        mRobotContainer.drivetrain.limelight.setIMUMode(2);
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (mAutonomousCommand != null) {
            CommandScheduler.getInstance().cancel(mAutonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
