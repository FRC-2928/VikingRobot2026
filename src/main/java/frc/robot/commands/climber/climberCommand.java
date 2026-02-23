package frc.robot.commands.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;


public class climberCommand extends Command {
  public climberCommand() { this.addRequirements(Robot.cont.climber); }

  //different states the climber can be in
  public enum ClimberHeight {
    HOMEPOS(0), //height in inches
    L1(30),
    L2(48),
    L3(66);

    public final double height;
    ClimberHeight(double height) {this.height = height; }
  }

  //state of the climber
  public enum ClimberState {
    IDLE,
    ASCENDING,
    DESCENDING;
  }
  
  private double startPos;
  private ClimberHeight targetHeight;

  @Override
  public void initialize() {
    //stops the robot from descending
    this.startPos = Robot.cont.climber.inputs.position;
    this.targetHeight = ClimberHeight.HOMEPOS;
  }

  @Override
  public void ascend() {
    if (targetHeight == ClimberHeight.HOMEPOS) {
      targetHeight = ClimberHeight.L1;
      Robot.cont.climber.inputs.state = ClimberState.ASCENDING;

    } else if (targetHeight == ClimberHeight.L1) {
      targetHeight = ClimberHeight.L2;
      Robot.cont.climber.inputs.state = ClimberState.ASCENDING;

    } else if (targetHeight == ClimberHeight.L2) {
      targetHeight = ClimberHeight.L3;
      Robot.cont.climber.inputs.state = ClimberState.ASCENDING;

    } else if (targetHeight == ClimberHeight.L3) {
      Robot.cont.climber.inputs.state = ClimberState.IDLE;
    }
  }

  @Override 
  public void descend() {
    if (targetHeight == ClimberHeight.L1) {
      targetHeight = ClimberHeight.HOMEPOS;
      Robot.cont.climber.inputs.state = ClimberState.DESCENDING;
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
