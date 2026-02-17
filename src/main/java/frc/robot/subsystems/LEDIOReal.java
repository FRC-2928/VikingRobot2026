// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class LEDIOReal extends SubsystemBase {
    /** Creates a new LEDIOReal. */
    private CANdle candle;

    public LEDIOReal() {
        this.candle = new CANdle(Constants.CAN.CTRE.candle, Constants.CAN.CTRE.bus);

        /* Configure CANdle */
        var cfg = new CANdleConfiguration();
        /* set the LED strip type and brightness */
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        /* disable status LED when being controlled */
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        candle.getConfigurator().apply(cfg);
    }

    public void setLEDs(SolidColor solidColor) {
        candle.setControl(solidColor);
    }

    public void clearAnimation() {
        candle.setControl(new EmptyAnimation(0));
    }
}
