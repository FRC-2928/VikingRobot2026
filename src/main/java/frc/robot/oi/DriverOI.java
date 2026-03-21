package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.vision.Limelight;

public class DriverOI extends BaseOI {
    /// Class Members
    private final Superstructure mSuperstructure;

    /// Triggers
    /// Trigger to toggle the drive mode between Free Drive and Target-Locked
    private final Trigger toggleRotationLockedMode;
    /// Trigger to handle shoot override
    private final Trigger shootOverride;

    public final Trigger shoot;

    // public final Trigger intake;

    // public final Trigger spinKicker;
    // private final Trigger startShoot;
    // public final Trigger shotConditionsMet;
    public final Trigger unjam;
    
    public final Trigger lockWheels;

    public final Trigger resetFOD;
    public final Trigger reseedLimeLights;

    // public final Trigger autoIntake;
    public final Trigger manualIntake;
    public final Trigger retractIntake;
    public final Trigger reverseIntakeRoller;

    public final Trigger climbTrigger;

    public final Haptics haptics;

    public DriverOI(final CommandXboxController controller, Superstructure superstructure) {
        super(controller);

        this.mSuperstructure = superstructure;
        this.shootOverride = this.controller.rightTrigger();
        this.shoot = this.controller.b();
        // left bumper toggles into/out of rotation locked mode
        this.toggleRotationLockedMode = this.controller.leftBumper();

        // this.autoIntake = this.controller.leftTrigger();
        this.reverseIntakeRoller = this.controller.leftTrigger();
        this.manualIntake = this.controller.rightBumper();
        this.retractIntake = this.controller.povRight();

        this.resetFOD = this.controller.y();
        this.reseedLimeLights = this.controller.a();
        this.lockWheels = this.controller.x();
        this.unjam = this.controller.povLeft();
        this.climbTrigger = this.controller.povUp();

        this.haptics = new BaseOI.Haptics(hid);

        this.haptics.type = RumbleType.kBothRumble;
    }

    public void configureControls() {
        // FIXME: this call only works because in Java synchronized methods are reentrant...
        // normally this would be a deadlock... we should seek to avoid such patterns...
        // this comes from a circular chain of getInstance -> init -> configureControls() -> getInstance()...
        var cont = RobotContainer.getInstance();
        this.resetFOD.onTrue(new InstantCommand(cont.drivetrain::zeroAngle));
        // this.resetAngle.onTrue(cont.drivetrain.runOnce(cont.drivetrain::zeroAngle));
        this.reseedLimeLights
            .onTrue(new InstantCommand(
                () -> cont.drivetrain.setLimelightIMUModesIntent(Limelight.IMUMode.MODE_1_EXTERNAL_SEED)))
            .onFalse(new InstantCommand(
                () -> cont.drivetrain.setLimelightIMUModesIntent(Limelight.IMUMode.MODE_4_INTERNAL_EXTERNAL_ASSIST)));
        this.toggleRotationLockedMode
            .onTrue(new ParallelCommandGroup(
                        mSuperstructure.toggleStateIntent(Superstructure.StateIntent.ACTION_TOGGLE_TARGET_LOCK_MODE), 
                        new InstantCommand(() -> { haptics.toggleRumble(); }),
                        new PrintCommand("Target Lock Trigger")
            ));
        this.shootOverride
            .onTrue(mSuperstructure.requestShootOverride())
            .onFalse(mSuperstructure.clearOverrideCommand());
        this.manualIntake
            .onTrue(
                mSuperstructure.setIntent(Superstructure.StateIntent.ACTION_INTAKE_DRIVE, true)
            )
            .onFalse(
                mSuperstructure.setIntent(Superstructure.StateIntent.ACTION_INTAKE_DRIVE, false)
            );

        this.retractIntake
            .onTrue(
                new InstantCommand(() -> cont.intake.setWantedState(Intake.WantedState.RETRACT), cont.intake))
            .onFalse(
                new InstantCommand(() -> cont.intake.setWantedState(Intake.WantedState.STOP))
            );

        this.reverseIntakeRoller
            .onTrue(
                new InstantCommand(() -> cont.intake.setWantedState(Intake.WantedState.REVERSE_ROLLER))
            )
            .onFalse(
                new InstantCommand(() -> cont.intake.setWantedState(Intake.WantedState.STOP))
            );
        // this.shoot
        //     .onTrue(mSuperstructure.setIntent(StateIntent.ACTION_SHOOT_HUB, true))
        //     .onFalse(mSuperstructure.setIntent(StateIntent.ACTION_SHOOT_HUB, false));
        // this.autoIntake
        //     .onTrue(mSuperstructure.setIntent(StateIntent.ACTION_INTAKE_AUTO, true))
        //     .onFalse(mSuperstructure.setIntent(StateIntent.ACTION_INTAKE_AUTO, false));
        // this.manualIntake
        //     .onTrue(mSuperstructure.setIntent(StateIntent.ACTION_INTAKE_MANUAL, true))
        //     .onFalse(mSuperstructure.setIntent(StateIntent.ACTION_INTAKE_MANUAL, false));
        // this.retractIntake
        //     .onTrue(mSuperstructure.setIntent(StateIntent.ACTION_INTAKE_RETRACT, true))
        //     .onFalse(mSuperstructure.setIntent(StateIntent.ACTION_INTAKE_RETRACT, false));
    }
}
