package frc.robot.commands.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;


public class climberCommand extends Command {
  public climberCommand() { this.addRequirements(Robot.cont.climber); }

  private boolean descending;
  private double startPos;

  @Override
  public void initialize() {
    this.descending = false;
    this.startPos = Robot.cont.climber.inputs.position;
  }

  @Override
  public void execute() {
    //checks if the robot is descending or ascending 
    if(this.descending) {
      //add code to slow the descent

      Logger.recordOutput("Climber/Initialize/State", "Descending"); //logs the robot descending
    } else {

      Logger.recordOutput("Climber/Initialize/Threshold", this.startPos + Constants.Climber.initializeRaiseDistance); //logs the threshold the robot is ascending
      if(Robot.cont.climber.inputs.position > this.startPos + Constants.Climber.initializeRaiseDistance) { //checks if the current position is greater than the previous position
        this.descending = true; //sets the robot to descending
      }
      Logger.recordOutput("Climber/Initialize/State", "Ascending"); //logs the robot ascending
    }
  }

  @Override
  public void end(boolean interrupted) {//stops the climb
    Robot.cont.climber.io.override(0);

    Logger.recordOutput("Climber/Initialize/State", "End"); //logs the end of the climb
  }


  @Override 
	public boolean isFinished() { return Robot.cont.climber.inputs.home; }
}
