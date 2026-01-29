// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Tuning;
import frc.robot.subsystems.Intake;;

public class RunIntake extends Command {
    /** Creates a new RunIntake. */
    private Intake intake;
    // public double speed;

    public RunIntake() {
        intake = new Intake();
        this.addRequirements(intake);
        // this.speed = Tuning.intakeSpeed.get();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = 0; 
        Logger.recordOutput("Intake/Speed", speed);
        intake.runIntake(MathUtil.clamp(speed, -1, 1));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { intake.runIntake(0); }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() { return false; }
}
