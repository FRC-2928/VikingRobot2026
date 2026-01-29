package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class RobotContainer {
    // public final LoggedDashboardChooser<Command> autonomousChooser;
    public final LoggedDashboardChooser<String> driveModeChooser;
    public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
    public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));
    public final Drivetrain drivetrain;
    public final Intake intake;
    public static boolean ledState = false;
    private final AutoChooser autoChooser;

    public RobotContainer() {
        Robot.instance.container = this;
        Robot.cont = this;

        Tuning.flywheelVelocity.get(); // load the class to put the tuning controls on the dashboard
        this.drivetrain = new Drivetrain();
        this.intake = new Intake();

        NamedCommands.registerCommand("ScoreL4", new RunCommand(() -> {}).withTimeout(2));

        // this.autonomousChooser = new LoggedDashboardChooser<>(
        // 	"Autonomous Routine",
        // 	Autonomous.createAutonomousChooser()
        // );

        autoChooser = Autonomous.getChoreoAutoChooser();
        SmartDashboard.putData("Autonomous Routine", autoChooser);
        autoChooser.select("SimpleFromRight");
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

        this.driveModeChooser = new LoggedDashboardChooser<>("Drive Mode", JoystickDrive.createDriveModeChooser());

        this.driverOI.configureControls(intake);
        this.operatorOI.configureControls();
    }

    // public Command getAutonomousCommand() { return this.autonomousChooser.get(); }
    public String getDriveMode() {
        return this.driveModeChooser.get();
    }
}
