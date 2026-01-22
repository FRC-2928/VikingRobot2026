// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberIO.ClimberIOInputs;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {
    this.io = switch(Constants.mode) {
		case REAL -> new ClimberIOReal();
		default -> throw new Error();
		};
  }

  public final ClimberIO io;
  public final ClimberIOInputs inputs = new ClimberIOInputs(){};

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    this.io.periodic();
  }
}




//kraken 66
//climb command

//2024 system code

/*public class Climber extends SubsystemBase {
	public Climber() {
		this.io = switch(Constants.mode) {
		case REAL -> new ClimberIOReal();
		default -> throw new Error();
		};
	}

	public final ClimberIO io;
	public final ClimberIOInputs inputs = new ClimberIOInputs(){};

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		//Logger.processInputs("Climber", this.inputs);

		this.io.periodic();
	}
}
*/