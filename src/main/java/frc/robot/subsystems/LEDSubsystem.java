// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.RobotState;

public class LEDSubsystem extends SubsystemBase {
    /// interface to LED hardware
    private LEDIO ledIO;
    /// robot state supplier
    private final Supplier<RobotState> mRobotStateSupplier;

    private WantedAction wantedAction;

    private static final int kStartIndex = 8;
    private static final int kEndIndex = 9;

    // TODO: remove along with LED_green
    public enum WantedAction {
        OFF,
        GREEN,
        RED,
        BLUE,
        PURPLE
    }

    public LEDSubsystem(LEDIO ledIO, Supplier<RobotState> robotStateSupplier) {
        this.ledIO = ledIO;
        this.mRobotStateSupplier = robotStateSupplier;
    }

    // TODO: remove along with LED_green
    public void setWantedAction(WantedAction wantedAction) {
        this.wantedAction = wantedAction;
    }

    @Override
    public void periodic() {
        var robotState = mRobotStateSupplier.get();
        switch (robotState) {
            case DISABLED: {
                ledIO.clearAnimation();
                ledIO.setLEDs(new SolidColor(kStartIndex, kEndIndex).withColor(new RGBWColor(0, 0, 0)));
                break;
            }
            case FREE_DRIVE: {
                ledIO.clearAnimation();
                ledIO.setLEDs(new SolidColor(kStartIndex, kEndIndex).withColor(new RGBWColor(0, 0, 0)));
                break;
            }
            case DRIVE_TARGET_LOCK: {
                ledIO.setLEDs(new SolidColor(kStartIndex, kEndIndex).withColor(new RGBWColor(0, 255, 0)));
                break;
            }
            case SHOOTING: {
                ledIO.setLEDs(new SolidColor(kStartIndex, kEndIndex).withColor(new RGBWColor(255, 0, 0)));
                break;
            }
            case MANUAL_INTAKE: {
                ledIO.setLEDs(new SolidColor(kStartIndex, kEndIndex).withColor(new RGBWColor(0, 0, 255)));
                break;
            }
            default: {
                break;
            }
        }
    }
}
