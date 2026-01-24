// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.ShooterDataCollector;
import frc.robot.utils.ShooterDataCollectorIO;
import frc.robot.utils.ShooterDataCollectorIOReal;
import frc.robot.utils.ShooterLookupTableBuilder;

public class Robot extends LoggedRobot {
    private Command mAutonomousCommand;

    private final RobotContainer mRobotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay =
            new HootAutoReplay().withTimestampReplay().withJoystickReplay();
	// Shooter data collection
	private ShooterLookupTableBuilder shooterDataBuilder;
	private ShooterDataCollector shooterDataCollector;

    public Robot() {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        mRobotContainer = new RobotContainer();

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotInit() {
        // mRobotContainer.drivetrain.limelight.setIMUMode(1);
        // Initialize shooter data collection
		shooterDataBuilder = new ShooterLookupTableBuilder();
		shooterDataBuilder.initialize();

		// Use real IO for robot hardware, could use sim IO for simulation
		ShooterDataCollectorIO io = new ShooterDataCollectorIOReal();
		shooterDataCollector = new ShooterDataCollector(shooterDataBuilder, io);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        LoggedPowerDistribution.getInstance(Constants.CAN.Misc.pdh, ModuleType.kRev);
        mRobotContainer.drivetrain.limelight.setThrottleRate(isEnabled() ? 0 : 100);

		// Update shooter data collector (checks for dashboard input)
		shooterDataCollector.periodic();

        // try {
        //     if (Tuning.publishData.get()) {
        //         // TODO PUT IN REAL VALUES!!!!
        //         Tuning.writeToCSV(
        //                 Tuning.hoodAngle.get(),
        //                 Tuning.releaseVelocity.get(),
        //                 mRobotContainer.drivetrain.getDistanceFromHub());
        //         Tuning.publishData.set(false);
        //     }
        // } catch (FileNotFoundException e) {
        //     e.printStackTrace();
        // }
        // mRobotContainer.drivetrain.limelight.setRobotOrientation(
        //         mRobotContainer.drivetrain.getCurrentPose2D().getRotation().getMeasure());
    }

    @Override
    public void disabledInit() {
        mRobotContainer.drivetrain.limelight.setIMUMode(1);
    }

    @Override
    public void disabledPeriodic() {
        // mRobotContainer.drivetrain.disabledPeriodic();
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
