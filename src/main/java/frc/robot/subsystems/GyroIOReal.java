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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

/** IO implementation for Pigeon2 */
public class GyroIOReal implements GyroIO {
    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;

    public GyroIOReal() {
        // instantiate the Pigeon2 object
        pigeon = new Pigeon2(Constants.CAN.CTRE.pigeon, Constants.CAN.CTRE.bus);

        // apply the default configs and reset the heading to 0
        // TODO: determine if this is even necessary
        this.pigeon.getConfigurator().apply(new Pigeon2Configuration());
        this.pigeon.reset();

        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();
        StatusSignal.setUpdateFrequencyForAll(Units.Hertz.of(100), yaw, yawVelocity);
    }

    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        StatusCode refreshStatus = BaseStatusSignal.refreshAll(this.yaw, this.yawVelocity);
        inputs.refreshStatus = refreshStatus;
        inputs.connected = refreshStatus.equals(StatusCode.OK);
        inputs.yawPosition = Units.Degrees.of(this.yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.DegreesPerSecond.of(this.yawVelocity.getValueAsDouble());
    }

    @Override
    public void setYaw(Angle yaw) {
        this.pigeon.setYaw(yaw.in(Units.Degrees));
    }

    @Override
    public void reset() {
        this.pigeon.reset();
    }
}
