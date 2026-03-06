// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberIO.ClimberIOInputs;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    public Climber() {
        this.io = switch (Constants.mode) {
            case REAL -> new ClimberIOReal();
            default -> throw new Error();};

        initDefaultCommand();
    }

    // heights for the climber
    public enum ClimberHeight {
        HOMEPOS(0), // height in inches
        L1(30),
        L2(48),
        L3(66);

        public final double height;

        ClimberHeight(double height) {
            this.height = height;
        }
    }
    // states for the climber
    public enum ClimberState {
        IDLE,
        AUTO_ASCENDING,
        TELEOP_ASCENDING,
        DESCENDING,
        FAILED;
    }

    public StatusSignal<Angle> height; // status signal for the postion of the climber based on the angle
    public StatusSignal<ReverseLimitValue> home;

    private boolean engaged = false; // if the climber is hooked on

    private ClimberHeight targetHeight = ClimberHeight.HOMEPOS;
    public ClimberState currentState = ClimberState.IDLE;

    public final ClimberIO io;
    public final ClimberIOInputs inputs = new ClimberIOInputs() {};

    public boolean climberDownToggle = false;


    // changes the target heights based on the current height
    public void ascend() {
        //checks the driver station mode the robot is in
        if (DriverStation.isTeleopEnabled()) {
            this.targetHeight = ClimberHeight.L1;
            this.currentState = ClimberState.TELEOP_ASCENDING; //sets the state to be the teleop ascending mode
        } else if (DriverStation.isAutonomousEnabled()){
            this.targetHeight = ClimberHeight.L1;
            this.currentState = ClimberState.AUTO_ASCENDING; //sets the state to be the auto ascending mode
        } 
    }

    public void descend() {
        //checks the driver station mode the robot is in
        this.targetHeight = ClimberHeight.HOMEPOS;
        this.currentState = ClimberState.DESCENDING;
    }

    public void ClimberToggleState(Boolean hasClimbed) {
        if (hasClimbed) {
            descend();
            return;
        }
        ascend();
    }

    public void initDefaultCommand() {
        setDefaultCommand(reset()); // sets the default command to go home
    }

    private boolean isEngaged() {
        return this.io.isEngaged(); //will always return true for week 1
    }

    //command that raises the arm of the climber for driving into climber zone.
    public Command prepClimber(ClimberHeight target) {
        return new InstantCommand(() -> {
            this.io.extend();
        });
    }

    public Command runClimber(Boolean hasClimbed)  {
        return new RunCommand(()  -> {
            ClimberToggleState(hasClimbed);
        }).until(() -> currentState == ClimberState.IDLE);
    }

    //command to handle setting the state and target for climbing down
    public Command reset() {
        return new InstantCommand(() -> {
            this.currentState = ClimberState.FAILED;
        });
    }

    // ---- TOGGLES FOR THE OPERATOR OI ---- \\
    public void moveClimberToggle() {
        Distance distance;
        if (climberDownToggle) {
            distance = Units.Inches.of(Math.abs(this.height.getValueAsDouble() - 0.5));
            this.io.climb(distance);
        }
        distance = Units.Inches.of(Math.abs(this.height.getValueAsDouble() + 0.5));
        this.io.climb(distance);
    }

    public  void changeClimberToggle(){
        climberDownToggle = !climberDownToggle;
    }

    public void climberIdle(){
        currentState = ClimberState.IDLE;
    }
    
    
    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);

        Logger.recordOutput("Climber/State", this.currentState); //records the current state of the robot
        Logger.recordOutput("Climber/TargetHeight", targetHeight.height); //records the target height of the climber
        Logger.recordOutput(
                "Climber/CurrentHeight", height.getValueAsDouble()); //records the current height of the climber

        switch (currentState) {
            case TELEOP_ASCENDING: {
                if (this.height.getValueAsDouble() == targetHeight.height) {
                    this.engaged = isEngaged();
                } else if (this.engaged) {
                    Distance distance = Units.Inches.of(ClimberHeight.HOMEPOS.height);
                    this.io.climb(distance);
                } else if (this.height.getValueAsDouble() == 0 && this.engaged == true) {
                    currentState = ClimberState.IDLE;
                }
                break;
            }

            case AUTO_ASCENDING: {
                if (this.height.getValueAsDouble() == targetHeight.height) {
                    this.engaged = isEngaged();
                } else if (this.engaged) {
                    Distance distance = Units.Inches.of(
                                Math.abs(targetHeight.height - 5)); // the robot goes 5 inches off the ground
                    this.io.climb(distance); // set the position
                } else if (this.height.getValueAsDouble() == 0 && this.engaged == true) {
                    currentState = ClimberState.IDLE;
                }
                break;
            }

            case DESCENDING: {
                if (this.height.getValueAsDouble() != ClimberHeight.HOMEPOS.height) {
                    this.io.goHome(); // makes the climber go home
                    this.engaged = false;
                } else {
                    this.currentState = ClimberState.IDLE;
                }
                break;
            }

            case IDLE: {
                //does nothing
            }

            case FAILED: {
                if (this.height.getValueAsDouble() != ClimberHeight.HOMEPOS.height) {
                    this.io.goHome(); // makes the climber go home
                } else {
                    this.currentState = ClimberState.IDLE;
                }
                break;
            }

        }
    }
}

