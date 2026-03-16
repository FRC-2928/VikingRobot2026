package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class OperatorOI extends BaseOI {
    public OperatorOI(final CommandXboxController controller) {
        super(controller);

        this.nudgeShooterAngleUp = this.controller.povUp();
        this.nudgeShooterAngleDown = this.controller.povDown();

        this.nudgeShooterSpeedUp = this.controller.povRight();
        this.nudgeShooterSpeedDown = this.controller.povLeft();

        this.resetNudges = this.controller.leftStick();

        this.runOverrides = this.controller.rightBumper();

        this.extendIntake = this.controller.leftBumper();
        this.recordShot = this.controller.back();
    }

    public final Trigger nudgeShooterAngleUp;
    public final Trigger nudgeShooterAngleDown;

    public final Trigger nudgeShooterSpeedUp;
    public final Trigger nudgeShooterSpeedDown;

    public final Trigger resetNudges;

    public final Trigger runOverrides;

    public final Trigger extendIntake;
    public final Trigger recordShot;

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

        this.extendIntake.onTrue(new InstantCommand(() -> RobotContainer.getInstance().intake.setWantedState(Intake.WantedState.EXTEND)));
        this.recordShot.onTrue(new InstantCommand(() -> RobotContainer.getInstance().matchRecorder.recordShot(RobotContainer.getInstance().drivetrain, RobotContainer.getInstance().shooter)));

        // this.runOverrides.whileTrue(new ParallelCommandGroup(RobotContainer.getInstance().shooter.shootOverride(), new RunCommand(() -> Logger.recordOutput("Superstructure/triggerIsRunning", Timer.getFPGATimestamp()), null)));
    }
}
