// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public Angle drivePosition = Units.Degrees.zero();
        public LinearVelocity driveVelocity = Units.MetersPerSecond.zero();
        public Current driveCurrent = Units.Amps.zero();

        public Angle angle = Units.Degrees.zero();
        public Current azimuthCurrent = Units.Amps.zero();
    }

    public default void setDriveVoltage(final double volts) {}

    public default void drive(final LinearVelocity desired) {}

    public default void azimuth(final Angle desired) {}

    public default void updateInputs(final ModuleIOInputs inputs) {}
}
