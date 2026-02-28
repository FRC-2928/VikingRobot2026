package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorOI extends BaseOI {
    public OperatorOI(final CommandXboxController controller) {
        super(controller);

        this.climberDown = this.controller.x();
        this.climberUp = this.controller.y();

        this.climberOverrideLower = this.controller.povDown();
        this.climberOverrideRaise = this.controller.povUp();

        this.initializeClimber = this.controller.rightStick();

        this.intakeOut = this.controller.b();
        this.intakeIn = this.controller.a();

        this.foc = this.controller.rightBumper();

        this.intakeOverride = this.controller.x();
        this.climbOverride = this.controller.leftBumper();
        this.shootOverride = this.controller.rightBumper();
    }

    public final Trigger climberDown;
    public final Trigger climberUp;

    public final Trigger climberOverrideLower;
    public final Trigger climberOverrideRaise;

    public final Trigger initializeClimber;

    public final Trigger intakeOut;
    public final Trigger intakeIn;

    public final Trigger foc;

    public final Trigger shootOverride;
    public final Trigger climbOverride;
    public final Trigger intakeOverride;

    public void configureControls() {}
}
