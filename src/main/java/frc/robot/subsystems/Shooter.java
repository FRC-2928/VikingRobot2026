package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.AimValues;
import frc.robot.Tuning;

public class Shooter extends SubsystemBase {
    public Shooter() {
        this.io = switch (Constants.mode) {
            case REAL -> new ShooterIOReal(this);
            default -> new ShooterIOReal(this);};
    }

    public final ShooterIO io;
    public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
    }

    public void aim(Distance distance) {
        AimValues val = Constants.Shooter.lookUpTable.get(distance.in(Units.Meters));
        this.io.runFlywheelsVelocity(val.shooterVelocity);
        this.io.rotateHood(val.hoodAngle);
    }

    public void shoot(Distance distance) {
        aim(distance);
        this.io.runKicker(Units.Volts.of(Tuning.kickerSpeed.get()));
    }

    public void home() {
        this.io.runFlywheelsVelocity(Units.DegreesPerSecond.zero());
        this.io.rotateHood(Units.Degrees.zero());
        this.io.runKicker(Units.Volts.zero());
    }
}
