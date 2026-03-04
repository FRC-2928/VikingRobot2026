// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberHeight;
import frc.robot.subsystems.Climber.ClimberState;

public class ClimberDescend extends Command {
    public ClimberDescend() {
        // this.addRequirements(Robot..climber);
    }

    private Climber climber;

    private Climber.ClimberHeight targetHeight;
    private double startPos;

    @Override
    public void initialize() {
        // this.startPos = Robot.cont.climber.inputs.height;
        this.targetHeight = ClimberHeight.HOMEPOS;
    }

    @Override
    public void execute() {
        Logger.recordOutput("Climber/Initialize/State", "Start");
        // climber.descend();
    }

    @Override
    public void end(boolean interrupted) { // stops the climb
        // Robot.cont.climber.climberIO.override();
        // climber.currentState = ClimberState.FAILED;

        Logger.recordOutput("Climber/Initialize/State", "End"); // logs the end of the climb
    }

    @Override
    public boolean isFinished() {
      return true;
        // return Robot.cont.climber.inputs.home;
    }
}
