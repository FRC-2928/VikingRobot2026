package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class OperatorOI extends BaseOI {
    public OperatorOI(final CommandXboxController controller) {
        super(controller);

        this.nudgeShooterAngleUp = this.controller.povUp();
        this.nudgeShooterAngleDown = this.controller.povDown();

        this.nudgeShooterSpeedUp = this.controller.povRight();
        this.nudgeShooterSpeedDown = this.controller.povLeft();

        this.nudgeClimberUp = this.controller.rightBumper();
        this.nudgeClimberDown = this.controller.leftBumper();

        this.resetNudges = this.controller.leftStick();
    }

    public final Trigger nudgeShooterAngleUp;
    public final Trigger nudgeShooterAngleDown;

    public final Trigger nudgeClimberUp;
    public final Trigger nudgeClimberDown;

    public final Trigger nudgeShooterSpeedUp;
    public final Trigger nudgeShooterSpeedDown;

    public final Trigger resetNudges;

    /* 
    public final Trigger climberOverrideLower;
    public final Trigger climberOverrideRaise;

    public final Trigger initializeClimber;

    public final Trigger intakeOut;
    public final Trigger intakeIn;

    public final Trigger foc;

    public final Trigger shootOverride;
    public final Trigger climbOverride;
    public final Trigger intakeOverride;
    */

    public void configureControls() {
        var cont = RobotContainer.getInstance();
        this.nudgeShooterAngleDown.onTrue(new InstantCommand(cont.shooter::nudgeAngleDown, cont.shooter));
        this.nudgeShooterAngleUp.onTrue(new InstantCommand(cont.shooter::nudgeAngleUp, cont.shooter));

        this.nudgeShooterSpeedDown.onTrue(new InstantCommand(cont.shooter::nudgeSpeedDown, cont.shooter));
        this.nudgeShooterSpeedUp.onTrue(new InstantCommand(cont.shooter::nudgeSpeedUp, cont.shooter));

        this.nudgeClimberUp.onTrue(new InstantCommand(() -> 
            cont.climber.moveClimberToggle(), 
            cont.climber)
        );
        this.nudgeClimberUp.onTrue((new RunCommand(() -> {
            cont.climber.climberIdle(); 
            cont.climber.changeClimberToggle();}, 
            cont.climber))
        );

        this.resetNudges.onTrue(new InstantCommand(cont.shooter::resetNudges));
    }
}
