package frc.robot.oi;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class BaseOI {
    public static final class Haptics {
        public Haptics(final XboxController hid) {
            this.hid = hid;
        }

        private final XboxController hid;

        public RumbleType type;
        public double interval;
        public double dutyCycle;
        public double powerTrue;
        public double powerFalse;

        public void update() {
            if (DriverStation.isAutonomous()) return;

            this.hid.setRumble(
                    this.type,
                    (Timer.getFPGATimestamp() % this.interval / (double) this.interval) <= this.dutyCycle
                            ? this.powerTrue
                            : this.powerFalse);
        }

        public void stop() {
            this.hid.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    public CommandXboxController controller;
    protected final XboxController hid;

    /**
     * C-Stop, the most useful command you'll ever have Press this magical button (start/left "window" button) to stop *all running commands*.
     */
    public final Trigger cstop;

    protected BaseOI(final CommandXboxController controller) {
        this.controller = controller;
        this.hid = controller.getHID();

        this.cstop = this.controller.start();
    }

    public XboxController getHID() {
        return this.hid;
    }
}
