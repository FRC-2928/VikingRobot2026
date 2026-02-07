package frc.robot.commands.drivetrain;

import javax.swing.plaf.basic.BasicEditorPaneUI;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Tuning;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.Tuning;

public class IntakeGround extends Command {
    public static double lastTime =
            0; // this is a bad way to do this but its necessary for right now, please do real path planning in the
    // future

	public final boolean correction;

    private final RobotContainer robotContainer;

    private final double speedMultiplier;

	private final CommandSwerveDrivetrain drivetrain;

	private final Intake intake;

	private final PIDController adjustX;

	private final PIDController adjustY;

    public IntakeGround(final boolean correction, RobotContainer robotContainer, double speedMultiplier) {
        this.robotContainer = robotContainer;

        this.correction = correction;
		
		this.intake = robotContainer.intake;

		this.drivetrain = robotContainer.drivetrain;

		this.adjustX = new PIDController(0.5,0,0);
		this.adjustY = new PIDController(0.5,0,0);

        this.addRequirements(robotContainer.intake);
        if (correction) this.addRequirements(robotContainer.drivetrain);

        this.speedMultiplier = speedMultiplier;
    }



    @Override
    public void execute() {
        // TODO: get reasonable speed
        intake.intakeIO.setSpeed(Tuning.intakeVelocity.get());

        if (this.correction){
            drivetrain.setControl(
                    drivetrain
                    .drive
                    .withVelocityX((robotContainer.joystick.getLeftX() * speedMultiplier + this.calculateSpeedX())
                            * robotContainer.MaxSpeed)
                    .withVelocityY((robotContainer.joystick.getLeftY() * speedMultiplier + this.calculateSpeedY())
                            * robotContainer.MaxSpeed));
		}
    }

    public double calculateSpeedX() {
		double output = adjustX.calculate(-1*drivetrain.limelight.getTargetHorizontalOffset().in(Units.Degrees),0);
        Logger.recordOutput(
                "Drivetrain/auto/SpeedXIntakeGroun",
                output);
        return output;
    }

    public double calculateSpeedY() {
        double output = adjustY.calculate(Math.abs(1/drivetrain.limelight.getTargetHorizontalOffset().in(Units.Degrees)),0);
		Logger.recordOutput(
                "Drivetrain/auto/SpeedYIntakeGround",
                output);
        return output;
    }

    @Override
    public void end(final boolean interrupted) {
        robotContainer.intake.intakeIO.setSpeed(0);

        robotContainer.drivetrain.halt();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
