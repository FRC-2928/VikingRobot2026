// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;

public interface LEDIO {
    public default void setLEDs(SolidColor solidColor) {}

    public default void clearAnimation() {}
}
