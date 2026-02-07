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
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.commands.drivetrain.RunIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriverOI extends BaseOI {
    public DriverOI(final CommandXboxController controller, CommandSwerveDrivetrain drivetrain) {
        super(controller);

        this.driveAxial = this.controller::getLeftY;
        this.driveLateral = this.controller::getLeftX;

        if (Constants.mode == Mode.REAL) {
            this.driveFORX = this.controller::getRightX;
            this.driveFORY = () -> -this.controller.getRightY();
        } else {
            this.driveFORX = () -> this.hid.getRawAxis(2);
            this.driveFORY = () -> this.hid.getRawAxis(3);
        }
        this.manualRotation = this.controller.rightStick();

        this.intake = this.controller.b();

        this.shotConditionsMet = new Trigger(() -> true);

        this.spinKicker = this.controller.rightTrigger().and(shotConditionsMet);

        this.getReadyToShoot = this.controller.leftTrigger();

        this.resetFOD = this.controller.y();

        this.resetAngle = this.controller.a();

        this.lockWheels = this.controller.x();
    }

    public final Supplier<Double> driveAxial;
    public final Supplier<Double> driveLateral;

    public final Supplier<Double> driveFORX;
    public final Supplier<Double> driveFORY;
    public final Trigger manualRotation;

    public final Trigger intake;

    public final Trigger spinKicker;
    public final Trigger getReadyToShoot;
    public final Trigger shotConditionsMet;

    public final Trigger lockWheels;

    public final Trigger resetFOD;
    public final List<Integer> reefTags = List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    public final List<Integer> proccesorTags = List.of(3, 16);
    public final List<Integer> humanStationTags = List.of(1, 2, 12, 13);
    public final List<Integer> bargeTags = List.of(4, 5, 14, 15);
    public final Trigger resetAngle;

    public void configureControls(RobotContainer cont) {

        this.lockWheels.whileTrue(new LockWheels(cont.drivetrain, this));
        this.resetFOD.onTrue(new InstantCommand(cont.drivetrain::resetAngle));
        this.intake.whileTrue(new RunIntake(cont.intake));
        this.resetAngle.whileTrue(new RunCommand(cont.drivetrain::seedLimelightImu));
        this.resetAngle.whileFalse(new RunCommand(cont.drivetrain::setImuMode2));
        this.spinKicker.onTrue(cont.shooter.startKicker());
        this.getReadyToShoot.onTrue(cont.shooter.getReadyToShoot(() -> 0.0)); // TODO: Put actual supplier into this.
    }
}
