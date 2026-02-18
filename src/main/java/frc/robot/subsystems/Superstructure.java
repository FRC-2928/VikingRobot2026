package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ExtendAndRunIntake;

public class Superstructure extends SubsystemBase {

    public enum State {

        IDLE(getDoNothingCommand()),
        INTAKE(Commands.run(intake::extendAndIntake)),
        AIM(superstructure.aimWithTransition()),
        SHOOT(superstructure.shoot());

        private final Trigger isCurrentState;
        private final Command command;

        State(Command doWhileTrue) {
            isCurrentState = new Trigger(() -> superstructure.isCurrentState(this));
            isCurrentState.whileTrue(doWhileTrue);
        }
    }

    private RobotContainer cont;
    private State currentState;

    public Superstructure(RobotContainer cont) {
        this.cont = cont;
        this.currentState = getStartingState();
    }

    // Runs flywheels and kicker. Command will not end on its own
    public Command startShooting() {
        return new RunCommand(
                        () -> {
                            Distance distance = cont.drivetrain.getDistanceFromHub();
                            cont.shooter.shoot(distance);
                        },
                        cont.shooter)
                .alongWith(cont.drivetrain.brake());
    }

    // Spins up flywheels to speed and turns hood to correct angle. Command will not end on its own
    public Command getReadyToShoot() {
        return new RunCommand(
                        () -> {
                            Distance distance = cont.drivetrain.getDistanceFromHub();
                            cont.shooter.aim(distance);
                        },
                        cont.shooter)
                .alongWith(cont.drivetrain.aimAtHubAndMove(0, 0, 0));
    }

    public Command readyAndShoot() {
        return new SequentialCommandGroup(getReadyToShoot().until(cont.driverOI.shotConditionsMet), startShooting());
    }

    // Stops robot from shooting
    public Command idle() {
        return new RunCommand(() -> cont.shooter.home());
    }

    public Command extendAndIntake() {
        return new ExtendAndRunIntake(cont.intake);
    }
}
