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
        this.rezeroIntake = this.controller.start();
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
    public final Trigger rezeroIntake;

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

        // Derail recovery: drive the rack past the soft limits until it is back on its hard stop, then rezero
        final Intake intake = RobotContainer.getInstance().intake;
        this.extendIgnoreLimit
            .onTrue(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.EXTEND_IGNORE_LIMITS)))
            .onFalse(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.STOP)));
        this.retractIgnoreLimit
            .onTrue(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.RETRACT_IGNORE_LIMITS)))
            .onFalse(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.STOP)));
        // allowed while disabled so the pit crew can rezero after physically pushing the rack home
        this.rezeroIntake.onTrue(new InstantCommand(intake::rezeroAtRetracted).ignoringDisable(true));

        // this.runOverrides.whileTrue(new ParallelCommandGroup(RobotContainer.getInstance().shooter.shootOverride(), new RunCommand(() -> Logger.recordOutput("Superstructure/triggerIsRunning", Timer.getFPGATimestamp()), null)));
    }
}
