// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
    /** Creates a new RunIntake. */
    private Intake intake;
    // public double speed;

    public RunIntake(Intake intake) {
        this.intake = intake;
        this.addRequirements(intake);
        // this.speed = Tuning.intakeSpeed.get();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // double speed = Tuning.intakeSpeed.get();
        AngularVelocity speed = Units.RotationsPerSecond.of(10);
        Logger.recordOutput("Intake/Speed", speed);
        intake.intakeIO.setSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.intakeIO.setSpeed(Units.RotationsPerSecond.of(0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
