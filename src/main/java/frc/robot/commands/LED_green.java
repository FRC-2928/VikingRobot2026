// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LED_green extends Command {
    private LEDSubsystem ledSubsystem;

    public LED_green(LEDSubsystem ledSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.ledSubsystem = ledSubsystem;
        this.addRequirements(ledSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.GREEN);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.OFF);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
