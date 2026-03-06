package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.AimValues;
import frc.robot.RobotContainer;
import frc.robot.Tuning;

public class Shooter extends SubsystemBase {
    public Shooter(RobotContainer cont) {
        this.io = switch (Constants.mode) {
            case REAL -> new ShooterIOReal(this);
            default -> new ShooterIOReal(this);
        };
        this.cont = cont;
        this.setDefaultCommand(homeCommand());
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final RobotContainer cont;

    public Angle getHoodAngle() {
        return inputs.hoodAngle;
    }

    public AngularVelocity getFlywheelVelocity() {
        return inputs.flywheelSpeedA;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Shooter", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simPeriodic();
        // Logger.recordOutput("Shooter/lookupTable", Constants.Shooter.lookUpTable.());
    }

    public void aim() {
        AimValues val = Constants.Shooter.lookUpTable.get(RobotContainer.getInstance().drivetrain.getDistanceFromHub().in(Units.Meters));
        if (val != null) {
            Logger.recordOutput("Shooter/AimValue", val.hoodAngle);
            this.io.runFlywheelsVelocity(val.shooterVelocity);
            // Hood angle is between 0 (home) and 40 (up) degrees
            // Aimvalues expects shoot angle between 80 (home) and 40 (up) degrees
            // This line converts the requested angle to hood setpoint, and clamps between 0 and 40
            Angle requestedAngle = Units.Degrees.of(MathUtil.clamp(80 - val.hoodAngle.in(Units.Degrees), 0, 40));
            this.io.rotateHood(requestedAngle);
        }
    }

    public void aimHome() { // TODO: Find actual values for these
        AimValues val = new AimValues(Units.Degrees.of(40), Units.RotationsPerSecond.of(40));
        Logger.recordOutput("Shooter/AimValue", val.hoodAngle);
        this.io.runFlywheelsVelocity(val.shooterVelocity);
        this.io.rotateHood(val.hoodAngle);
    }

    public void shoot() {
        //aim();
        this.io.runKicker(Units.Volts.of(7));
    }

    public void home() {
        this.io.stopFlyWheels();
        this.io.rotateHood(Units.Degrees.zero());
        this.io.runKicker(Units.Volts.zero());
    }

    public Command aimAtHub() {
        return new FunctionalCommand(
            this::aim,
            () -> {},
            (interrupted) -> {
                if (interrupted) {
                    home();
                }
            },
            this::isAimed,
            this
        );
    }

    public Command aimAtHome() {
        return new FunctionalCommand(
            this::aimAtHome,
            () -> {},
            (interrupted) -> {
                if (interrupted) {
                    home();
                }
            },
            this::isAimed,
            this
        );
    }

    public boolean isAimed() {
        boolean correctHoodAngle = this.getHoodAngle().lte(Constants.Shooter.hoodAngleTolerance);
        boolean correctFlywheelVelocity =
            this.getFlywheelVelocity().lte(Constants.Shooter.shooterVelocityTolerance);
        return correctHoodAngle && correctFlywheelVelocity;
    }

    public void nudgeAngleUp() {
        this.io.nudgeAngleUp();
    }

    public void nudgeAngleDown() {
        this.io.nudgeAngleDown();
    }

    public void nudgeSpeedUp() {
        this.io.nudgeSpeedUp();
    }

    public void nudgeSpeedDown() {
        this.io.nudgeSpeedDown();
    }

    public void resetNudges() {
        this.io.resetNudges();
    }

    public Command homeCommand() {
        return new RunCommand(this::home, this);
    }

    public Command shootCommand() {
        return new FunctionalCommand(
            this::shoot,
            () -> {}, 
            (interrupted) -> {
                if (interrupted) {
                    home();
                }
            },
            () -> { return false; },
            this
        );
    }
}
