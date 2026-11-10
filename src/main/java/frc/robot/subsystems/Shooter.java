package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.AimValues;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain.DrivetrainState;
import frc.robot.subsystems.ShooterIO.ShooterIOInputs;
import frc.robot.subsystems.shooter.ShooterGoal;
import frc.robot.superstructure.SuperstructureContext;

public class Shooter extends SubsystemBase {
     // ── Goal / state machine ──────────────────────────────────────────────────

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

    private WantedState wantedState     = WantedState.HOME;
    private SystemState systemState     = SystemState.HOME;
    private WantedState prevWantedState = WantedState.HOME;

    SuperstructureContext superstructureContext;

     // ── IO / inputs ───────────────────────────────────────────────────────────

    public final ShooterIO io;
    public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // ── Constructor ───────────────────────────────────────────────────────────

    public Shooter() {
        this.io = switch (Constants.mode) {
            case REAL -> new ShooterIOReal(this);
            default   -> new ShooterIO() {};
        };
        //SmartDashboard.putData("Shooter/PitchPID", pitchPID);  delete 
    }

    // ── Signal registration ─────────────────────────────────────────────────

    public com.ctre.phoenix6.BaseStatusSignal[] getStatusSignals() {
        return io.getStatusSignals();
    }

    // ── Goal API ──────────────────────────────────────────────────────────────

    /**
     * Called every 20 ms by {@link frc.robot.superstructure.Superstructure#periodic()}.
     * Sets {@code wantedState} from the goal; does not write to hardware.
     */
    public void applyGoal(final SuperstructureContext ctx) {
        ShooterGoal desired = ctx.goal().shooter();
        superstructureContext = ctx;

        wantedState = switch (desired) {
            case SHOOT          -> WantedState.SHOOT;
            case HALT           -> WantedState.HALT;
            case AIM_HUB_AUTO   -> WantedState.AIM_HUB_AUTO;
            case SHOOT_OVERRIDE -> WantedState.SHOOT_OVERRIDE;
            case SHOOT_HOME     -> WantedState.SHOOT_HOME;
            case HOME           -> WantedState.HOME;
        };
    }

    // ── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        systemState = handleStateTransition();
        applyState();

        Logger.recordOutput("Shooter/WantedState", wantedState.toString());
        Logger.recordOutput("Shooter/SystemState",  systemState.toString());
    }

    // ── State machine ─────────────────────────────────────────────────────────

    private SystemState handleStateTransition() {
        if (wantedState != prevWantedState) {
            prevWantedState = wantedState;
        }

        return switch (wantedState) {
            case SHOOT          -> SystemState.SHOOT;
            case HALT           -> SystemState.HALT;
            case AIM_HUB_AUTO   -> SystemState.AIM_HUB_AUTO;
            case SHOOT_OVERRIDE -> SystemState.SHOOT_OVERRIDE;
            case SHOOT_HOME     -> SystemState.SHOOT_HOME;
            case HOME           -> SystemState.HOME;
        };
    }

    private void applyState() {
        switch (systemState) {
            case SHOOT                                   -> applyShoot();
            case HALT                                    -> applyHalt();
            case AIM_HUB_AUTO                            -> applyAimHubAuto();
            case SHOOT_OVERRIDE                          -> applyShootOverride();
            case SHOOT_HOME                              -> applyShootHome();
            case HOME                                    -> applyHome();
        }
    }

    // ── Per-state hardware routines ───────────────────────────────────────────

    private void applyShoot() {
        aim();
        // only run the kicker -- flywheels + hood are already commanded to hold setpoints by aim() methods
        this.io.runKicker(Units.RotationsPerSecond.of(64));
        // this.aimAtHub();
        // this.runShooter();  
    }

    private void applyHalt() {
        this.io.stopFlyWheels();
        this.io.stopKicker();
    }

    private void applyAimHubAuto() {
        aim();
    }

    private void applyShootOverride() {
        this.io.runKicker(Units.RotationsPerSecond.of(64));
        this.io.runFlywheelsVelocity(Units.RotationsPerSecond.of(38));
        this.io.rotateHood(Units.Degrees.of(13));
    }

    private void applyShootHome() {
        aimHome();
        // only run the kicker -- flywheels + hood are already commanded to hold setpoints by aim() methods
        this.io.runKicker(Units.RotationsPerSecond.of(64));
    }

    private void applyHome() {
        this.io.stopFlyWheels();
        this.io.rotateHood(Units.Degrees.zero());
        this.io.stopKicker();
    }

    

   
    private double lastMetersToHub = 0.0;


    public Angle getHoodAngle() {
        return inputs.hoodAngle;
    }

    public AngularVelocity getFlywheelVelocity() {
        return inputs.shooterAVelocity;
    }


    @Override
    public void simulationPeriodic() {
        io.simPeriodic();
        // Logger.recordOutput("Shooter/lookupTable", Constants.Shooter.lookUpTable.());
    }

    public void aim() {
        DrivetrainState drivetrainState = superstructureContext.drivetrainState();
        boolean isAtHome = drivetrainState.isAtHome();

        var metersToTarget = isAtHome ? drivetrainState.distanceFromHub().in(Units.Meters) : drivetrainState.distanceFromHome().in(Units.Meters);
        lastMetersToHub = metersToTarget;  // TODO: lastMetersToHub is likely unused, remove
        AimValues val = isAtHome ?
             Constants.Shooter.lookUpTable.get(metersToTarget) : Constants.Shooter.lookUpTableShootHome.get(metersToTarget);
        Logger.recordOutput("Shooter/AimValueMetersToTarget", metersToTarget);
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

    public void shoot() {
         aim();
        this.io.runKicker(Units.RotationsPerSecond.of(64));
    }

    public void aimHome() { // TODO: Find actual values for these
        DrivetrainState drivetrainState = superstructureContext.drivetrainState();

        var metersToHome = drivetrainState.distanceFromHome().in(Units.Meters);
        AimValues val = Constants.Shooter.lookUpTableShootHome.get(metersToHome);
        Logger.recordOutput("Shooter/AimValueMetersToHub", metersToHome);
        Logger.recordOutput("Shooter/AimValueHoodAngleDegrees", val.hoodAngle.in(Units.Degrees));
        Logger.recordOutput("Shooter/AimValueFlywheelSpeedsRPS", val.shooterVelocity.in(Units.RotationsPerSecond));
        this.io.runFlywheelsVelocity(val.shooterVelocity);
        this.io.rotateHood(val.hoodAngle);
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

    public void adjustAim () {
        // FIXME: this is not the right check -- fix this or delete it...
        // The intention is to:
        // 1. Account for potentially bad pose estimates in the initial lookup
        // 2. Help eliminate variation in shot profile due to noise in the system once we have a good pose estimate
        // However, the current logic here won't scale for when we incorporate shoot-on-the-fly
        if (lastMetersToHub < 1.5 || lastMetersToHub > 3) {
            DrivetrainState drivetrainState = superstructureContext.drivetrainState();
            var metersToHub = drivetrainState.distanceFromHub().in(Units.Meters);

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

    private void shootOverride() {
        runShooter();
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

     // ── State snapshot (used by SuperstructureContext and auto routines) ───────

    /**
     * Read-only snapshot of shooter state for {@link frc.robot.superstructure.SuperstructureContext}.
     *
     * @param holdingNote  true when beam-break sensor detects a note
     * @param shotComplete true when feeder has been running for at least {@code fireTimeout}
     * @param atSafeAngle  true when pivot is low enough for climber deployment
     */
    public record ShooterState() {}

    public ShooterState getState() {
        return new ShooterState();
    }

    // ── Helper ────────────────────────────────────────────────────────────────
    // Space for any future helper methods :)
}
