// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.generated.TunerConstants;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperFloor;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDIOReal;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;

public class RobotContainer {
    /// singleton instance of the RobotContainer
    private static RobotContainer sInstance = null;
    // TODO organize Driver and Operator bindings
    public DriverOI driverOI;
    public OperatorOI operatorOI;
    // public final LoggedDashboardChooser<String> driveModeChooser;
    public AutoChooser autoChooser;
    public double MaxSpeed =
            1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public final CommandXboxController joystick1 = new CommandXboxController(0);
    public final CommandXboxController joystick2 = new CommandXboxController(1);

    public Superstructure mSuperstructure;
    public CommandSwerveDrivetrain drivetrain;
    public Shooter shooter;
    public Intake intake;
    private Telemetry logger;
    public HopperFloor hopperFloor;
    public Indexer indexer;
    public LEDSubsystem ledSubsystem;
    private double teleopStart = 0;

    public static synchronized RobotContainer getInstance() {
        if (sInstance != null) {
            return sInstance;
        }

        sInstance = new RobotContainer();
        sInstance.init();
        return sInstance;
    }

    private RobotContainer() {
        // Empty
    }

    private void init() {
        // always instantiate the Superstructure first to ensure its periodic() is always the first to be invoked
        // this is useful for having the Superstructure orchestrate the status signal refreshes for all
        // registered subsystems -- this is an optimization that should help reduce CAN usage 
        this.mSuperstructure = Superstructure.create(this);
        this.drivetrain = TunerConstants.createDrivetrain();
        this.shooter = new Shooter(this);
        this.intake = new Intake();
        this.indexer = new Indexer();
        this.hopperFloor = new HopperFloor();
        this.logger = new Telemetry(MaxSpeed, drivetrain);
        this.autoChooser = Autonomous.getChoreoAutoChooser(this);
        this.ledSubsystem = new LEDSubsystem(new LEDIOReal(), mSuperstructure.getRobotStateSupplier());
        SmartDashboard.putData("Autonomous Routine", autoChooser);
        this.driverOI = new DriverOI(joystick1, mSuperstructure);
        this.operatorOI = new OperatorOI(joystick2);
        autoChooser.select("SimpleFromRight");
        // TODO: implement drive mode chooser (point, turn modes). Joystick drive needs to consume the chosen mode
        // this.driveModeChooser = new LoggedDashboardChooser<>("Drive Mode", JoystickDrive.createDriveModeChooser());
        RobotModeTriggers.autonomous()
                .whileTrue(
                        autoChooser.selectedCommandScheduler()); // Schedules the selected auto command during auto mode
        configureBindings();
        Tuning.hoodAngle.get(); // try to load tuning controls if available
    }

    private void configureBindings() {
        this.driverOI.configureControls();
        // this.operatorOI.configureControls();

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(drivetrain.joystickDrive(driverOI));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled()
                .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick1.a().whileTrue(drivetrain.brake());
        // For testing purposes. TODO install new version of WPILIB and use 2026 field apriltag map
        joystick1
                .rightBumper()
                .whileTrue(new CenterLimelight(Units.Meters.of(0.5), Units.Meters.of(0), List.of(1), drivetrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick1.x().whileTrue(drivetrain.aimAtHubAndMove(joystick1, 1.0));
        joystick1.back().and(joystick1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick1.back().and(joystick1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick1.start().and(joystick1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick1.start().and(joystick1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        joystick1.b().whileTrue(mSuperstructure.startShooting());
        // Reset the field-centric heading on left bumper press.
        joystick1.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    public void setTeleopStartTime() {
        this.teleopStart = Timer.getFPGATimestamp();
    }

    public double getTeleopMatchTime() {
        return Timer.getFPGATimestamp() - this.teleopStart;
    }
    // public String getDriveMode() {
    //     return this.driveModeChooser.get();
    // }
}
