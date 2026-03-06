package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.StateIntent;

public class DriverOI extends BaseOI {
    /// Class Members
    private final Superstructure mSuperstructure;

    /// Triggers
    /// Trigger to toggle the drive mode between Free Drive and Target-Locked
    private final Trigger toggleRotationLockedMode;
    /// Trigger to handle shoot override
    private final Trigger shootOverride;

    // public final Trigger intake;

    // public final Trigger spinKicker;
    // private final Trigger startShoot;
    // public final Trigger shotConditionsMet;
    public final Trigger unjam;
    
    public final Trigger lockWheels;

    public final Trigger resetFOD;
    public final Trigger resetAngle;

    public final Trigger autoIntake;
    public final Trigger manualIntake;


    public DriverOI(final CommandXboxController controller, Superstructure superstructure) {
        super(controller);

        this.mSuperstructure = superstructure;
        this.shootOverride = this.controller.rightTrigger();
        // left bumper toggles into/out of rotation locked mode
        this.toggleRotationLockedMode = this.controller.leftBumper();

        // this.shotConditionsMet = new Trigger(() -> true);
        // new Trigger(() -> {
        //     /*boolean facingHub = Robot.cont.drivetrain
        //     .getAngleToHub(Constants.Shooter.shooterAngleOffsetFromFront)
        //     .lte(Constants.Shooter.toleranceFromHub);*/
        //     boolean correctHoodAngle = robotContainer.shooter.getHoodAngle().lte(Constants.Shooter.hoodAngleTolerance);
        //     boolean correctFlywheelVelocity =
        //             robotContainer.shooter.getFlywheelVelocity().lte(Constants.Shooter.shooterVelocityTolerance);
        //     return /*facingHub &&*/ correctHoodAngle && correctFlywheelVelocity;
        // });
        this.autoIntake = this.controller.leftTrigger();
        this.manualIntake = this.controller.rightBumper();

        this.resetFOD = this.controller.y();
        this.resetAngle = this.controller.a();
        this.lockWheels = this.controller.x();
        this.unjam = this.controller.povLeft();
    }

    public void configureControls() {
        // FIXME: this call only works because in Java synchronized methods are reentrant...
        // normally this would be a deadlock... we should seek to avoid such patterns...
        // this comes from a circular chain of getInstance -> init -> configureControls() -> getInstance()...
        var cont = RobotContainer.getInstance();
        // this.lockWheels.whileTrue(new LockWheels(cont.drivetrain, this));
        this.resetFOD.onTrue(new InstantCommand(cont.drivetrain::resetAngle));
        // this.intake.whileTrue(cont.superstructure.extendAndIntake());
        this.resetAngle.whileTrue(new RunCommand(cont.drivetrain::seedLimelightImu));
        this.resetAngle.whileFalse(new RunCommand(cont.drivetrain::setImuMode2));
        this.toggleRotationLockedMode.onTrue(mSuperstructure.toggleStateIntent(Superstructure.StateIntent.ACTION_TOGGLE_TARGET_LOCK_MODE));
        this.shootOverride
            .onTrue(mSuperstructure.requestShootOverride())
            .onFalse(mSuperstructure.clearOverrideCommand());
        this.autoIntake
            .onTrue(mSuperstructure.setIntent(StateIntent.ACTION_INTAKE_AUTO, true))
            .onFalse(mSuperstructure.setIntent(StateIntent.ACTION_INTAKE_AUTO, false));
        this.manualIntake
            .onTrue(mSuperstructure.setIntent(StateIntent.ACTION_INTAKE_MANUAL, true))
            .onFalse(mSuperstructure.setIntent(StateIntent.ACTION_INTAKE_MANUAL, false));
        // this.spinKicker.onTrue(cont.shooter.startKicker());
        // this.shotConditionsMet
        //         .and(() -> cont.drivetrain
        //                 .getAngleToHub(Constants.Shooter.shooterAngleOffsetFromFront)
        //                 .lte(Constants.Shooter.toleranceFromHub))
        //         .whileTrue(mSuperstructure.getReadyToShoot());
        // this.startShoot.whileTrue(mSuperstructure.shootAutomated());
    }
}
