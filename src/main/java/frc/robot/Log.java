package frc.robot;

import java.util.Arrays;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// todo: implement AdvantageKit and remove this

public final class Log extends SubsystemBase {
    private static Log instance;

    /// The maximum number of lines the buffer should contain at any given moment
    ///
    /// Note that `writelnFast` will bypass this, so ensure you call
    /// `trimExcessLeadingLines` after using `writelnFast`
    public static int lineLimit = Integer.MAX_VALUE;
    private int lines = 0;
    private final StringBuilder log = new StringBuilder(1024);
    private boolean dirty = false;

    private Log() {}

    /// Starts the log, you should never need to use this, save for
    /// the first line of the `main` function
    public static void start() throws Exception {
        Log.instance = new Log();
    }

    /// Writes data to stderr, writes it to the buffer, and dirties
    /// the network entry.
    public static void write(final Object... input) {
        Log.writeFast(input);
        Log.trimExcessLeadingLines();
    }

    /// Writes a line to stderr, writes it to the buffer, trims old
    /// leading lines, and dirties the network entry.
    public static void writeln(final Object... input) {
        Log.write(input, '\n');
    }

    /// Writes to the buffer, does NOT trim any old leading lines, and
    /// dirties the network entry.
    /// Use this to buffer many line calls together before calling
    /// `trimExcessLeadingLines` or `writeln`
    public static void writeFast(final Object... input) {
        for (final Object entry : input) {
            if (entry.getClass().isArray()) {
                Object[] array;

                if (entry instanceof int[])
                    array = Arrays.stream((int[]) entry).boxed().toArray(Integer[]::new);
                else if (entry instanceof long[])
                    array = Arrays.stream((long[]) entry).boxed().toArray(Long[]::new);
                else if (entry instanceof double[])
                    array = Arrays.stream((double[]) entry).boxed().toArray(Double[]::new);
                else array = (Object[]) entry;

                Log.writeFast(array);
            } else {
                Log.writeFast(entry.toString());
            }
        }
    }

    public static void writeFast(final String str) {
        DriverStationJNI.sendConsoleLine(str);

        System.err.print(str);

        Log.instance.log.append(str);
        Log.instance.lines += str.lines().count();
        Log.instance.dirty = true;
    }

    /// Writes to the buffer, does NOT trim any old leading lines, and
    /// dirties the network entry.
    /// Use this to buffer many line calls together before calling
    /// `trimExcessLeadingLines` or `writeln`
    public static void writelnFast(final Object... input) {
        Log.writeFast(input, '\n');
    }

    /// Panic the runtime with an uncatchable exception
    /// This function will *never* return
    public static void panic(final Exception fuckup) {
        Log.error(fuckup);

        System.exit(1);
    }

    /// Writes an error to the buffer, trims old leading lines, dirties the network entry, and transmits the error to
    // the driver station with a full stack trace
    public static void error(final Exception error) {
        Log.writeln("! ERROR: ", error.getLocalizedMessage(), " !");
        DriverStation.reportError(error.getLocalizedMessage(), error.getStackTrace());
    }

    /// Writes a warning to the buffer, trims old leading lines, dirties the network entry, and transmits the warning to
    // the driver station without a stack trace
    public static void warning(String str) {
        DriverStationJNI.sendError(true, 0, false, str, "", "", true);

        str += '\n';

        Log.instance.log.append(str);
        Log.instance.lines += str.lines().count();
        Log.instance.dirty = true;

        System.err.print(str);
    }

    /// Writes a warning to the buffer, trims old leading lines, dirties the network entry, and transmits the warning to
    // the driver station with a full stack trace
    public static void warningFull(final String warning) {
        Log.writeln("! WARNING: ", warning, " !");
        DriverStation.reportWarning(warning, Thread.currentThread().getStackTrace());
    }

    /// Trims as many leading lines is necessary so that the buffer contains less
    /// than `lineLimit` lines long
    public static void trimExcessLeadingLines() {
        while (Log.instance.lines > Log.lineLimit) Log.instance.trimSingleLine();
    }

    /// Unconditionally trims a single line off the front
    private void trimSingleLine() {
        this.log.delete(0, Math.max(this.log.indexOf("\n") + 1, 0));
        this.lines--;
        this.dirty = true;
    }

    @Override
    public void periodic() {
        if (this.dirty || true) {
            SmartDashboard.putString("SystemLog", this.log.toString());
            this.dirty = false;
        }
    }
}
