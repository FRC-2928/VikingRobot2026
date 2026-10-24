package frc.robot;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoChooser;

/**
 * Choreo AutoChooser that records its selection to the AdvantageKit log.
 *
 * <p>Relies on AutoChooser's Sendable calling {@link #select(String)} both when the dashboard
 * changes the selection and on every SmartDashboard update (via the "active" property), so the
 * values are logged once per loop while the chooser is published. Re-verify this after ChoreoLib
 * upgrades.
 *
 * <p>The base constructor calls {@link #select(String)} before this class is initialized, so do not
 * add fields that the override depends on.
 */
public class LoggedAutoChooser extends AutoChooser {
    @Override
    public String select(String requested) {
        String active = super.select(requested);
        // Requested is what the dashboard asked for; Active is the routine that will actually run.
        // Choreo only regenerates while disabled with a known alliance, so these can differ.
        Logger.recordOutput("Autonomous/Requested", requested);
        Logger.recordOutput("Autonomous/Active", active);
        return active;
    }
}
