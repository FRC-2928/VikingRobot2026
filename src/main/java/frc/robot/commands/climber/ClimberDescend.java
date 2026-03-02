// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberDescend extends Command {
  /** Creates a new ClimberDescend. */
  public ClimberDescend() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}package frc.robot.commands.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberHeight;
import frc.robot.subsystems.Climber.ClimberState;

public class ClimberDescend extends Command {
    public ClimberDescend() {
        this.addRequirements(Robot.cont.climber);
    }

    private Climber climber;

    private Climber.ClimberHeight targetHeight;
    private double startPos;

    @Override
    public void initialize() {
        this.startPos = Robot.cont.climber.inputs.height;
        this.targetHeight = ClimberHeight.HOMEPOS;
    }

    @Override
    public void execute() {
        Logger.recordOutput("Climber/Initialize/State", "Start");
        climber.descend();
    }

    @Override
    public void end(boolean interrupted) { // stops the climb
        Robot.cont.climber.climberIO.override();
        climber.currentState = ClimberState.FAILED;

        Logger.recordOutput("Climber/Initialize/State", "End"); // logs the end of the climb
    }

    @Override
    public boolean isFinished() {
        return Robot.cont.climber.inputs.home;
    }
}



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
