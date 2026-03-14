// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperFloor;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.MatchRecorder;

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
    // public Climber climber;
    // public LEDSubsystem ledSubsystem;
    public MatchRecorder matchRecorder;
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
        // this.climber = new Climber();
        this.logger = new Telemetry(MaxSpeed, drivetrain);
        this.autoChooser = Autonomous.getChoreoAutoChooser(this);
        // this.ledSubsystem = new LEDSubsystem(new LEDIOReal(), mSuperstructure.getRobotStateSupplier());
        SmartDashboard.putData("Autonomous Routine", autoChooser);
        this.driverOI = new DriverOI(joystick1, mSuperstructure);
        this.operatorOI = new OperatorOI(joystick2);
        autoChooser.select("bl_comp");
        // TODO: implement drive mode chooser (point, turn modes). Joystic drive needs to consume the chosen mode
        // this.driveModeChooser = new LoggedDashboardChooser<>("Drive Mode", JoystickDrive.createDriveModeChooser());
        RobotModeTriggers.autonomous()
                .whileTrue(
                        autoChooser.selectedCommandScheduler()); // Schedules the selected auto command during auto mode
        configureBindings();
        Tuning.hoodAngle.get(); // try to load tuning controls if available

        // init the superstructure now that all subsystems have been instantiated
        mSuperstructure.init();
    }

    private void configureBindings() {
        this.driverOI.configureControls();
        this.operatorOI.configureControls();

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled()
                .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick1.x().whileTrue(drivetrain.aimAtHubAndMove(driverOI, 1.0));
        // joystick1.back().and(joystick1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick1.back().and(joystick1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick1.start().and(joystick1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick1.start().and(joystick1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // Reset the field-centric heading on left bumper press.
        // joystick1.rightBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

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
