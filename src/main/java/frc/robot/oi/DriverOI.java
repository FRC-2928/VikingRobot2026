package frc.robot.oi;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.commands.drivetrain.RunIntake;
import frc.robot.subsystems.Intake;

public class DriverOI extends BaseOI {
    public DriverOI(final CommandXboxController controller) {
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

    public final Trigger lockWheels;

    public final Trigger resetFOD;
    public final List<Integer> reefTags = List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    public final List<Integer> proccesorTags = List.of(3, 16);
    public final List<Integer> humanStationTags = List.of(1, 2, 12, 13);
    public final List<Integer> bargeTags = List.of(4, 5, 14, 15);
    public final Trigger resetAngle;

    public void configureControls(Intake intake) {

        this.lockWheels.whileTrue(new LockWheels());
        this.resetFOD.onTrue(new InstantCommand(Robot.cont.drivetrain::resetAngle));
        this.intake.whileTrue(new RunIntake(intake));
        this.resetAngle.whileTrue(new RunCommand(Robot.cont.drivetrain::seedLimelightImu));
        this.resetAngle.whileFalse(new RunCommand(Robot.cont.drivetrain::setImuMode2));
    }
}
