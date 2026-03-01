// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Tuning;
import frc.robot.subsystems.Intake;

public class ExtendAndRunIntake extends Command {
    /** Creates a new RunIntake. */
    private Intake intake;
    // public double speed;

    public ExtendAndRunIntake(Intake intake) {
        this.intake = intake;
        this.addRequirements(intake);
        Tuning.intakeSpeed.get();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // AngularVelocity speed = Units.RotationsPerSecond.of(10);
        intake.extend();
        double speed = Tuning.intakeSpeed.get();
        Logger.recordOutput("Intake/Speed", speed);
        // intake.setIntakeSpeed(speed);
        // Logger.recordOutput("Intake/Speed", speed);
        // intake.intakeIO.setSpeed();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // intake.setIntakeSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
