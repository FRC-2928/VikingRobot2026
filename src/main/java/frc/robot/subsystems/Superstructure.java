package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ExtendAndRunIntake;
import frc.robot.oi.DriverOI;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

    public enum RobotState {
        DRIVE_HOME_ZONE,
        DRIVE_MID_ZONE,
        AIM_HOME_ZONE,
        SHOOT,
        DRIVE,
        MANUAL_INTAKE,
        INTAKE,
        MID_FIELD,
        GET_READY_CLIMB,
        UNJAM
    }

    private RobotContainer cont;
    private DriverOI driverOI;
    private RobotState currentState;
    private List<Trigger> stateTriggers;
    private Map<RobotState, Runnable> transitionFunctions;

    public Superstructure(RobotContainer cont) {
        this.cont = cont;
        this.driverOI = cont.driverOI;
        this.stateTriggers = new ArrayList<>();

        // Init each state's command to run
        initState(RobotState.DRIVE_HOME_ZONE, idle());
        initState(RobotState.DRIVE_MID_ZONE, idle());
        initState(RobotState.AIM_HOME_ZONE, idle());
        initState(RobotState.SHOOT, idle());
        initState(RobotState.DRIVE, driveCommand());

        transitionFunctions = new HashMap<>();

        // Bind transition function for each state
        transitionFunctions.put(RobotState.DRIVE_HOME_ZONE, this::checkHomeZoneTransitions);
        transitionFunctions.put(RobotState.SHOOT, this::checkShootTransitions);
        transitionFunctions.put(RobotState.DRIVE, this::checkDriveTransitions);
        this.currentState = RobotState.DRIVE;
    }

    // Initializes trigger for when given state is active, and runs given command when trigger is active
    private void initState(RobotState state, Command runWhenCurrentState) {
        stateTriggers.add(new Trigger(() -> this.currentState == state).whileTrue(runWhenCurrentState));
    }

    @Override
    public void periodic() {
        // Get the transition function for the current state and execute it
        transitionFunctions
                .getOrDefault(currentState, () -> {
                    // TODO Log warning about missing transition function for current state
                })
                .run();
        Logger.recordOutput("RobotState/Shooter Velocity", cont.shooter.getFlywheelVelocity());
        Logger.recordOutput("RobotState/Ready to Shoot", cont.driverOI.shotConditionsMet.getAsBoolean());
        Logger.recordOutput("RobotState/Intake Deployed", cont.intake.checkExtended());
        Logger.recordOutput("RobotState/Current State", currentState.toString());
        Logger.recordOutput("RobotState/ShootTrigger", cont.driverOI.startShoot.getAsBoolean());
        Logger.recordOutput("RobotState/Superstructure", stateTriggers.get(4).getAsBoolean());
    }

    private void checkHomeZoneTransitions() {
        // TODO: Check possible transitions out of home zone, and set currentState accordingly
        // e.g. if(!poseInHomeZone()) {
        //          currentState = RobotState.DRIVE_MID_ZONE;
        // }
    }

    private void checkShootTransitions() {
        // TODO: Check possible transitions out of home zone, and set currentState accordingly
        // e.g. if(!poseInHomeZone()) {
        //          currentState = RobotState.DRIVE_MID_ZONE;
        // }
    }

    private void checkDriveTransitions() {
        if(cont.driverOI.startShoot.getAsBoolean()){
            currentState = RobotState.SHOOT;
            return;
        }

        else if(cont.driverOI.intake.getAsBoolean()){
            currentState = RobotState.INTAKE;
            return;
        }

        else if(cont.driverOI.climb.getAsBoolean()){
            currentState = RobotState.GET_READY_CLIMB;
            return;
        }

        else if(cont.drivetrain.getCurrentPose2D().getMeasureX().gt(frc.robot.Constants.FIELD.distanceToMidField) 
            && cont.drivetrain.getCurrentPose2D().getMeasureX().lt(frc.robot.Constants.FIELD.fieldLength
                .minus(frc.robot.Constants.FIELD.distanceToMidField))){
                currentState = RobotState.MID_FIELD;
                return;
            }

        else if(cont.driverOI.unjam.getAsBoolean()){
            currentState = RobotState.UNJAM;
            return;
        }
    }

    public Command driveCommand() {
        return cont.drivetrain.joystickDrive(cont.driverOI);
    }

    // Runs flywheels and kicker. Command will not end on its own
    public Command startShooting() {
        return new RunCommand(
                        () -> {
                            Distance distance = cont.drivetrain.getDistanceFromHub();
                            cont.shooter.shoot(distance);
                        },
                        cont.shooter)
                .alongWith(cont.drivetrain.brake())
                .alongWith(cont.hopperFloor.runHopperCommand());
    }

    // Spins up flywheels to speed and turns hood to correct angle. Command will not end on its own
    public Command getReadyToShoot() {
        return new RunCommand(
                        () -> {
                            Distance distance = cont.drivetrain.getDistanceFromHub();
                            cont.shooter.aim(distance);
                        },
                        cont.shooter)   
                .alongWith(cont.drivetrain.aimAtHubAndMove(cont.driverOI.controller, 0.0));
    }

    public Command readyAndShoot() {
        return new SequentialCommandGroup(getReadyToShoot().until(cont.driverOI.shotConditionsMet), startShooting());
    }

    // Stops robot from shooting
    public Command idle() {
        return new RunCommand(() -> cont.shooter.home());
    }

    public Command extendAndIntake() {
        return new ExtendAndRunIntake(cont.intake);
    }

    public Command pathWileINtaking(String pathFileName) {
        return new ParallelDeadlineGroup(cont.drivetrain.runPath(pathFileName), this.extendAndIntake())
                .finallyDo(() -> {
                    cont.intake.setIntakeSpeed(0);
                    cont.intake.retract();
                });
    }
}
