// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    /** Creates a new LED. */
    private LEDIO ledIO;

    private WantedAction wantedAction;

    private static final int kStartIndex = 8;
    private static final int kEndIndex = 9;

    public enum WantedAction {
        OFF,
        GREEN
    }

    public LEDSubsystem(LEDIO ledIO) {
        this.ledIO = ledIO;
    }

    public void setWantedAction(WantedAction wantedAction) {
        this.wantedAction = wantedAction;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        switch (wantedAction) {
            case OFF:
                ledIO.clearAnimation();
                ledIO.setLEDs(new SolidColor(kStartIndex, kEndIndex).withColor(new RGBWColor(0, 0, 0)));
                break;
            case GREEN:
                ledIO.setLEDs(new SolidColor(kStartIndex, kEndIndex).withColor(new RGBWColor(0, 255, 0)));
                break;
            default:
                break;
        }
    }
}
