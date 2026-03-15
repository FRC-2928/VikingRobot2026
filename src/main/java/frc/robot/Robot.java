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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.MatchRecorder;
import frc.robot.utils.ShooterDataCollector;
import frc.robot.utils.ShooterDataCollectorIO;
import frc.robot.utils.ShooterDataCollectorIOReal;
import frc.robot.utils.ShooterLookupTableBuilder;
import frc.robot.vision.Limelight;

public class Robot extends LoggedRobot {
    private Command mAutonomousCommand;

    private final RobotContainer mRobotContainer;
    private LoggedPowerDistribution pdh;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay =
            new HootAutoReplay().withTimestampReplay().withJoystickReplay();
    // Shooter data collection
    private ShooterLookupTableBuilder shooterDataBuilder;
    private ShooterDataCollector shooterDataCollector;

    public MatchRecorder matchRecorder;

    public Robot() {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();

        mRobotContainer = RobotContainer.getInstance();
        pdh = LoggedPowerDistribution.getInstance(Constants.CAN.Misc.pdh, ModuleType.kRev);

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

        // Initialize match recorder (starts background lookup-table load)
        matchRecorder = new MatchRecorder();
        mRobotContainer.matchRecorder = matchRecorder;
    }

    private boolean m_lastEnabledState = false;

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        boolean enabled = isEnabled();
        if (enabled != m_lastEnabledState) {
            mRobotContainer.drivetrain.setAllLimelightThrottleRates(enabled ? 0 : 100);
            m_lastEnabledState = enabled;
            mRobotContainer.drivetrain.setLimelightIMUModesIntent(
                    enabled ? Limelight.IMUMode.MODE_4_INTERNAL_EXTERNAL_ASSIST : Limelight.IMUMode.MODE_1_EXTERNAL_SEED);
        }

        // Update shooter data collector (checks for dashboard input)
        shooterDataCollector.periodic();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // mRobotContainer.drivetrain.disabledPeriodic();
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        mAutonomousCommand = mRobotContainer.getAutonomousCommand();
        mRobotContainer.drivetrain.setState(CommandSwerveDrivetrain.WantedState.AUTONOMOUS);

        if (mAutonomousCommand != null) {
            CommandScheduler.getInstance().schedule(mAutonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (mAutonomousCommand != null) {
            CommandScheduler.getInstance().cancel(mAutonomousCommand);
        }

        mRobotContainer.setTeleopStartTime();
        matchRecorder.teleopInit();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        matchRecorder.teleopExit();
    }

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
