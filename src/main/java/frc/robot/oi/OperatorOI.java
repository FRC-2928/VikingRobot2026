package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class OperatorOI extends BaseOI {
    public OperatorOI(final CommandXboxController controller) {
        super(controller);

        this.nudgeShooterAngleUp = this.controller.povRight();
        this.nudgeShooterAngleDown = this.controller.povLeft();

        this.nudgeShooterSpeedUp = this.controller.povUp();
        this.nudgeShooterSpeedDown = this.controller.povDown();

        this.resetNudges = this.controller.leftStick().or(this.controller.rightBumper());

        // this.runOverrides = this.controller.rightBumper();

        this.recordShot = this.controller.back().or(this.controller.leftBumper());

        this.extendIgnoreLimit = this.controller.rightTrigger();
        this.retractIgnoreLimit = this.controller.leftTrigger();
    }

    public final Trigger nudgeShooterAngleUp;
    public final Trigger nudgeShooterAngleDown;

    public final Trigger nudgeShooterSpeedUp;
    public final Trigger nudgeShooterSpeedDown;

    public final Trigger resetNudges;

    // public final Trigger runOverrides;

    public final Trigger recordShot;

    public final Trigger extendIgnoreLimit;
    public final Trigger retractIgnoreLimit;

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
        this.nudgeShooterAngleDown.onTrue(new InstantCommand(RobotContainer.getInstance().shooter::nudgeAngleDown));
        this.nudgeShooterAngleUp.onTrue(new InstantCommand(RobotContainer.getInstance().shooter::nudgeAngleUp));

        this.nudgeShooterSpeedDown.onTrue(new InstantCommand(RobotContainer.getInstance().shooter::nudgeSpeedDown));
        this.nudgeShooterSpeedUp.onTrue(new InstantCommand(RobotContainer.getInstance().shooter::nudgeSpeedUp));

        this.resetNudges.onTrue(new InstantCommand(RobotContainer.getInstance().shooter::resetNudges));

        this.recordShot.onTrue(new InstantCommand(() -> RobotContainer.getInstance().matchRecorder.recordShot(RobotContainer.getInstance().drivetrain, RobotContainer.getInstance().shooter)));

        this.extendIgnoreLimit.onTrue(new InstantCommand(RobotContainer.getInstance().intake::extendIgnoreLimit));
        this.retractIgnoreLimit.onTrue(new InstantCommand(RobotContainer.getInstance().intake::retractIgnoreLimit));

        // this.runOverrides.whileTrue(new ParallelCommandGroup(RobotContainer.getInstance().shooter.shootOverride(), new RunCommand(() -> Logger.recordOutput("Superstructure/triggerIsRunning", Timer.getFPGATimestamp()), null)));
    }
}
