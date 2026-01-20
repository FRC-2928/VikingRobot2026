package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase{
	public Shooter() {
		this.io = switch(Constants.mode) {
		case REAL -> new ShooterIOReal(this);
		default -> throw new Error();
		};
	}

	public final ShooterIO io;
	public final ShooterIOInputs inputs = new ShooterIOInputs();

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
	}
}
