package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.HopperFloorIOReal;

public class OperatorOI extends BaseOI {
    public OperatorOI(final CommandXboxController controller, HopperFloorIOReal hopper) {
        super(controller);

        this.climberDown = this.controller.x();
        this.climberUp = this.controller.y();

        this.climberOverrideLower = this.controller.povDown();
        this.climberOverrideRaise = this.controller.povUp();

        this.initializeClimber = this.controller.rightStick();

        this.intakeOut = this.controller.b();
        this.intakeIn = this.controller.a();

        this.fixedShoot = this.controller.leftTrigger(); // This trigger is for driver intention to prepare the shot
        this.haltShotTrigger = this.controller.rightTrigger(); // This is the override to stop shooting
        this.doShoot = this.fixedShoot.and(haltShotTrigger.negate());  // This is the composite trigger for the robot to do the shoot command

        this.foc = this.controller.rightBumper();

        this.hopper = hopper;

        
    }

    public final Trigger climberDown;
    public final Trigger climberUp;

    public final Trigger climberOverrideLower;
    public final Trigger climberOverrideRaise;

    public final Trigger initializeClimber;

    public final Trigger intakeOut;
    public final Trigger intakeIn;
    public final Trigger fixedShoot;
    public final Trigger doShoot;

    public final Trigger haltShotTrigger;

    public final Trigger foc;

    public final HopperFloorIOReal hopper;

    public void configureControls() {
        this.doShoot.whileTrue(null); // TODO: add shooting command
    }
}
