package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.oi.BaseOI;
import org.littletonrobotics.junction.Logger;

public class IntakeGround extends Command {
	public static double lastTime = 0; // this is a bad way to do this but its necessary for right now, please do real path planning in the future

	public IntakeGround(final boolean correction, RobotContainer robotContainer) {
		this.robotContainer = robotContainer;

		this.correction = correction;

		this.addRequirements(robotContainer.intake);
		if(correction) this.addRequirements(robotContainer.drivetrain);
	}


	public final boolean correction;

	private final RobotContainer robotContainer;

	@Override
	public void execute() {
		//TODO: get reasonable speed
		robotContainer.intake.intakeIO.setSpeed(0);

		if(this.correction)
			robotContainer.drivetrain
				.control(
					robotContainer.drivetrain.drive
					.withVelocityX(robotContainer.joystick.getLeftX())
					.withVelocityY(robotContainer.joystick.getLeftY())
						.plus(
							Robot.cont.drivetrain
								.rod(
									new ChassisSpeeds(
										this.calculateSpeedX(),
										Robot.cont.drivetrain.limelightNote
											.getTargetHorizontalOffset()
											.in(Units.Rotations)
											* 10,
										0
									).times(pivotReady ? 1 : 1)
								)
						)
				);

		this.haptics.update();
		
	}
	public double calculateSpeedX(){
		Logger.recordOutput("Drivetrain/auto/SpeedXIntakeGroun",(-10/(Math.abs(Robot.cont.drivetrain.limelightNote.getTargetHorizontalOffset().in(Units.Degrees))+1)));
		return( 
				(-10/(Math.abs(Robot.cont.drivetrain.limelightNote.getTargetHorizontalOffset().in(Units.Degrees))+1))
		);
	}
	
	@Override
	public void end(final boolean interrupted) {
		Robot.cont.shooter.io
			.rotate(
				Robot.cont.shooter.inputs.holdingNote ? Constants.Shooter.readyDrive : Constants.Shooter.readyIntake
			);
		Robot.cont.shooter.io.runFlywheels(0);
		Robot.cont.shooter.io.runFeeder(Demand.Halt);
		Robot.cont.shooter.io.runIntake(Demand.Halt);

		Robot.cont.drivetrain.control(new ChassisSpeeds());

		this.haptics.stop();
	}

	@Override
	public boolean isFinished() { return Robot.cont.shooter.inputs.holdingNote; }
}
