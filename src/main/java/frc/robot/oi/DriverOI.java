package frc.robot.oi;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

public class DriverOI extends BaseOI {
    /// Class Members
    private final Superstructure mSuperstructure;

    /// Triggers
    /// Trigger to toggle the drive mode between Free Drive and Target-Locked
    private final Trigger toggleRotationLockedMode;
    /// Trigger to handle shoot override
    private final Trigger shootOverride;

    public final Supplier<Double> driveAxial;
    public final Supplier<Double> driveLateral;

    public final Supplier<Double> driveFORX;
    public final Supplier<Double> driveFORY;
    public final Trigger manualRotation;

    public final Trigger intake;

    // public final Trigger spinKicker;
    // private final Trigger startShoot;
    // public final Trigger shotConditionsMet;
    public final Trigger unjam;
    
    public final Trigger lockWheels;

    public final Trigger resetFOD;
    public final List<Integer> reefTags = List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    public final List<Integer> proccesorTags = List.of(3, 16);
    public final List<Integer> humanStationTags = List.of(1, 2, 12, 13);
    public final List<Integer> bargeTags = List.of(4, 5, 14, 15);
    public final Trigger resetAngle;

    public DriverOI(final CommandXboxController controller, Superstructure superstructure) {
        super(controller);

        this.mSuperstructure = superstructure;
        this.shootOverride = this.controller.rightTrigger();
        this.driveAxial = this.controller::getLeftY;
        this.driveLateral = this.controller::getLeftX;
        this.intake = this.controller.b();
        // left bumper toggles
        this.toggleRotationLockedMode = this.controller.leftBumper();

        if (Constants.mode == Mode.REAL) {
            this.driveFORX = this.controller::getRightX;
            this.driveFORY = () -> -this.controller.getRightY();
        } else {
            this.driveFORX = () -> this.hid.getRawAxis(2);
            this.driveFORY = () -> this.hid.getRawAxis(3);
        }
        this.manualRotation = this.controller.rightStick();

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

        // this.spinKicker = this.controller.rightTrigger().and(shotConditionsMet);

        // this.startShoot = this.controller.leftTrigger();
        this.resetFOD = this.controller.y();

        this.resetAngle = this.controller.a();

        this.lockWheels = this.controller.x();

        // this.climb = this.controller.leftBumper();

        this.unjam = this.controller.povLeft();
    }

    public void configureControls() {
        var cont = RobotContainer.getInstance();
        // this.lockWheels.whileTrue(new LockWheels(cont.drivetrain, this));
        this.resetFOD.onTrue(new InstantCommand(cont.drivetrain::resetAngle));
        // this.intake.whileTrue(cont.superstructure.extendAndIntake());
        this.resetAngle.whileTrue(new RunCommand(cont.drivetrain::seedLimelightImu));
        this.resetAngle.whileFalse(new RunCommand(cont.drivetrain::setImuMode2));
        var toggleRotationLockedModeCmd = new InstantCommand(
            () -> mSuperstructure.toggleIntent(Superstructure.StateIntent.ACTION_TOGGLE_TARGET_LOCK_MODE),
            mSuperstructure);
        this.toggleRotationLockedMode.onTrue(toggleRotationLockedModeCmd);
        this.shootOverride
            .onTrue(mSuperstructure.requestShootOverride())
            .onFalse(mSuperstructure.clearOverrideCommand());
        // this.spinKicker.onTrue(cont.shooter.startKicker());
        // this.shotConditionsMet
        //         .and(() -> cont.drivetrain
        //                 .getAngleToHub(Constants.Shooter.shooterAngleOffsetFromFront)
        //                 .lte(Constants.Shooter.toleranceFromHub))
        //         .whileTrue(mSuperstructure.getReadyToShoot());
        // this.startShoot.whileTrue(mSuperstructure.shootAutomated());
    }
}
