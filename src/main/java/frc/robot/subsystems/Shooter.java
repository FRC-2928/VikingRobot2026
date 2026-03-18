package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.AimValues;
import frc.robot.RobotContainer;

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
    private double lastMetersToHub = 0.0;
    private WantedState mDesiredState = WantedState.HALT;
    private SystemState mCurrentState = SystemState.HALT;

    public enum WantedState {
        SHOOT,
        HALT,
        AIM_HUB_AUTO,
        SHOOT_OVERRIDE,
        SHOOT_HOME,
        HOME
    }

    public enum SystemState {
        SHOOT,
        HALT,
        AIM_HUB_AUTO,
        SHOOT_OVERRIDE,
        SHOOT_HOME,
        HOME
    }

    public Angle getHoodAngle() {
        return inputs.hoodAngle;
    }

    public AngularVelocity getFlywheelVelocity() {
        return inputs.shooterAVelocity;
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
        var metersToHub = RobotContainer.getInstance().drivetrain.getDistanceFromHub().in(Units.Meters);
        lastMetersToHub = metersToHub;
        AimValues val = Constants.Shooter.lookUpTable.get(metersToHub);
        Logger.recordOutput("Shooter/AimValueMetersToHub", metersToHub);
        if (val != null) {
            Logger.recordOutput("Shooter/AimValueHoodAngleDegrees", val.hoodAngle.in(Units.Degrees));
            Logger.recordOutput("Shooter/AimValueFlywheelSpeedsRPS", val.shooterVelocity.in(Units.RotationsPerSecond));
            this.io.runFlywheelsVelocity(val.shooterVelocity);
            // Hood angle is between 0 (home) and 40 (up) degreesaq
            // Aimvalues expects shoot angle between 80 (home) and 40 (up) degrees
            // This line converts the requested angle to hood setpoint, and clamps between 0 and 40
            Angle requestedAngle = Units.Degrees.of(MathUtil.clamp(80 - val.hoodAngle.in(Units.Degrees), 0, 40));
            this.io.rotateHood(requestedAngle);
        }
    }

    public void aimHome() { // TODO: Find actual values for these
        var metersToHome = RobotContainer.getInstance().drivetrain.getDistanceFromHome().in(Units.Meters);
        AimValues val = Constants.Shooter.lookUpTableShootHome.get(metersToHome);
        Logger.recordOutput("Shooter/AimValueMetersToHub", metersToHome);
        Logger.recordOutput("Shooter/AimValueHoodAngleDegrees", val.hoodAngle.in(Units.Degrees));
        Logger.recordOutput("Shooter/AimValueFlywheelSpeedsRPS", val.shooterVelocity.in(Units.RotationsPerSecond));
        this.io.runFlywheelsVelocity(val.shooterVelocity);
        this.io.rotateHood(val.hoodAngle);
    }

    public void shoot() {
        aim();

        // only run the kicker -- flywheels + hood are already commanded to hold setpoints by aim() methods
        this.io.runKicker(Units.RotationsPerSecond.of(64));
        // this.aimAtHub();
        // this.runShooter();
    }

    private void shootOverride() {
        runShooter();
    }

    public void runShooter(){
        this.io.runKicker(Units.RotationsPerSecond.of(64));
        this.io.runFlywheelsVelocity(Units.RotationsPerSecond.of(38));
        this.io.rotateHood(Units.Degrees.of(13));   
    }

    public Command runShooterAuto(){
        var cmd = new FunctionalCommand(
            this::runShooter,
            () -> {},
            (interrupted) -> { 
                home();
            },
            () -> { return false; },
            this);
        return cmd;
        // return new RunCommand(
        //     () -> {
        //         this.io.runKicker(Units.Volts.of(7));
        //         this.io.runFlywheelsVelocity(Units.RotationsPerSecond.of(32));
        //         this.io.rotateHood(Units.Degrees.of(1.5));
        //     }, this
        // );
    }

    public Command runFlywheelCommand(){
        return new RunCommand(() -> {
                this.io.runFlywheelsVelocity(Units.RotationsPerSecond.of(32));
            }, this
        );
    }

    public void home() {
        this.io.stopFlyWheels();
        this.io.rotateHood(Units.Degrees.zero());
        this.io.stopKicker();
    }

    public Command shootOverrideCommand() {
        return new FunctionalCommand(
            this::shootOverride,
            () -> {
                Logger.recordOutput("Shooter/OverrideRunning", true);
            },
            (interrupted) -> {
                // not returing to home to allow overriden command to control transitions
                Logger.recordOutput("Shooter/OverrideRunning", false);
            },
            () -> { return false; },
            this);
    }

    public void adjustAim () {
        // FIXME: this is not the right check -- fix this or delete it...
        // The intention is to:
        // 1. Account for potentially bad pose estimates in the initial lookup
        // 2. Help eliminate variation in shot profile due to noise in the system once we have a good pose estimate
        // However, the current logic here won't scale for when we incorporate shoot-on-the-fly
        if (lastMetersToHub < 1.5 || lastMetersToHub > 3) {
            var metersToHub = RobotContainer.getInstance().drivetrain.getDistanceFromHub().in(Units.Meters);

            AimValues val = Constants.Shooter.lookUpTable.get(metersToHub);
            Logger.recordOutput("Shooter/AimValueMetersToHub", metersToHub);
            if (val != null) {
                Logger.recordOutput("Shooter/AimValueHoodAngleDegrees", val.hoodAngle.in(Units.Degrees));
                Logger.recordOutput("Shooter/AimValueFlywheelSpeedsRPS", val.shooterVelocity.in(Units.RotationsPerSecond));
                this.io.runFlywheelsVelocity(val.shooterVelocity);
                // Hood angle is between 0 (home) and 40 (up) degrees
                // Aimvalues expects shoot angle between 80 (home) and 40 (up) degrees
                // This line converts the requested angle to hood setpoint, and clamps between 0 and 40
                Angle requestedAngle = Units.Degrees.of(MathUtil.clamp(80 - val.hoodAngle.in(Units.Degrees), 0, 40));
                this.io.rotateHood(requestedAngle);
            }
        }
    }

    public Command aimAtHub() {
        return new FunctionalCommand(
            this::aim,
            this::aim,
            (interrupted) -> {
                if (interrupted) {
                    home();  // TODO: probably don't want to do this, because interrupt could come from override
                }
            },
            this::isAimed,
            this
        );
    }

    public Command aimAtHomeCommand() {
        return new FunctionalCommand(
            this::aimHome,
            this::aimHome,
            (interrupted) -> {
                if (interrupted) {
                    home();  // TODO: probably don't want to do this, because interrupt could come from override
                }
            },
            this::isAimed,
            this
        );
    }

    public boolean isAimed() {
        return inputs.hoodAngleInTolerance && inputs.flywheelsInTolerance;
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
