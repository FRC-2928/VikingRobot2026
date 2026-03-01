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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.climber.ClimberDescend;
import frc.robot.subsystems.ClimberIO.ClimberIOInputs;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    public Climber() {
        this.climberIO = switch (Constants.mode) {
            case REAL -> new ClimberIOReal();
            default -> throw new Error();};
    }

    


    //heights for the climber
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
    //states for the climber 
    public enum ClimberState {
        IDLE,
        AUTO_ASCENDING,
        TELEOPP_ASCENDING,
        DESCENDING,
        FAILED;
    }

    public StatusSignal<Angle> height; // status signal for the postion of the climber based on the angle
    public StatusSignal<ReverseLimitValue> home;

    private boolean engaged = false; //if the climber is hooked on

    private ClimberHeight targetHeight = ClimberHeight.HOMEPOS;
    public ClimberState currentState = ClimberState.IDLE;



    public final ClimberIO climberIO;
    public final ClimberIOInputs inputs = new ClimberIOInputs() {};


    //changes the target heights based on the current height
    public void ascend() {
        if (height.getValueAsDouble() == ClimberHeight.HOMEPOS.height /*&& is the robot not in auto */) {
            //when the robot is not in auto
            this.targetHeight = ClimberHeight.L1;
            this.currentState = ClimberState.TELEOPP_ASCENDING; 
        } /*else if (is robot in auto){
            this.targetHeight = ClimberHeight.L1;
            this.currentState = ClimberState.AUTO_ASCENDING;
        } */
    }

    public void descend() {
        if (height.getValueAsDouble() == ClimberHeight.L1.height) {
            this.targetHeight = ClimberHeight.HOMEPOS;
            this.currentState = ClimberState.DESCENDING;
        }
    }

    public void initDefaultCommand() {
        setDefaultCommand(new ClimberDescend()); //sets the default command to go home
    }

    private boolean isEngaged() {

        return climberIO.isEngaged();
    }


    @Override
    public void periodic() {
        this.climberIO.updateInputs(this.inputs);
        
        Logger.recordOutput("Climber/State", this.currentState);
        Logger.recordOutput("Climber/TargetHeight", targetHeight.height); // records the target height of the climber
        Logger.recordOutput("Climber/CurrentHeight", height.getValueAsDouble()); //records the current height of the climber

        if (targetHeight.height != height.getValueAsDouble()) {
            
            switch (currentState) {

                case TELEOPP_ASCENDING: {
                    if (this.height.getValueAsDouble() <= 30 && this.engaged == false) {
                        climberIO.goUp();
                    } else if (this.height.getValueAsDouble() == targetHeight.height) {
                        Distance distance = Units.Inches.of(Math.abs(targetHeight.height - 1)); // the robot goes 1 inch off the ground to see if the climber is hooked
                        climberIO.goToPosition(distance);
                        this.engaged = isEngaged();
                        if (!this.engaged) {
                            distance = Units.Inches.of(targetHeight.height);
                            climberIO.goToPosition(distance);
                        }
                    } else if (this.engaged) {
                        climberIO.goDown();
                    } else if (this.height.getValueAsDouble() == 0 && this.engaged == true) {
                        currentState = ClimberState.IDLE; 
                    }
                    break;
                }  
            
                case AUTO_ASCENDING: {
                    if (this.height.getValueAsDouble() <= 30 && this.engaged == false) {
                        climberIO.goUp();
                    } else if (this.height.getValueAsDouble() == targetHeight.height) {
                        Distance distance = Units.Inches.of(Math.abs(targetHeight.height - 1)); // the robot goes 1 inch off the ground to see if the climber is hooked
                        climberIO.goToPosition(distance);
                        this.engaged = isEngaged();
                        if (!this.engaged) {
                            distance = Units.Inches.of(targetHeight.height);
                            climberIO.goToPosition(distance);
                        }
                    } else if (this.engaged) {
                        Distance distance = Units.Inches.of(Math.abs(targetHeight.height - 5)); // the robot goes 5 inches off the ground
                        climberIO.goToPosition(distance); //set the position
                    } else if (this.height.getValueAsDouble() == 0 && this.engaged == true) {
                        currentState = ClimberState.IDLE; 
                    }
                    break;
                }  

                case DESCENDING: {
                    climberIO.goDown(); //makes the climber go home
                    break;
                }

                case FAILED: {
                    climberIO.goDown();
                }
            }


        }

        
    }
}

// if you speak minecraft enchantment table: ℸ⍑ᒷ ᓵꖎ╎ᒲʖᒷ∷ ꖎ╎ꖌᒷᓭ ᓵꖎ╎ᒲʖ╎リ⊣