# Implementation Tasks

## Tasks

- [x] 1. Create MatchShotRecord data class
  - [x] 1.1 Create `src/main/java/frc/robot/utils/MatchShotRecord.java` with fields: matchTimestamp (double, seconds), distanceMeters (double), rotationToTargetDeg (double), computedShotAngleDeg (double), actualShotAngleDeg (double), targetHoodAngleDeg (double), currentHoodAngleDeg (double), targetFlywheelRPS (double), currentFlywheelRPS (double)
  - [x] 1.2 Add `toCSV()` method returning a comma-separated string of all fields
  - [x] 1.3 Add static `getCsvHeader()` method returning the header row matching field order

- [x] 2. Create Lookup_Table_File under `src/main/deploy/`
  - [x] 2.1 Create `src/main/deploy/shooter_lookup_table.csv` with a header row and one data row per entry in `Constants.Shooter.temporaryLookupTable` (columns: distance_m, hood_angle_deg, flywheel_rps)
  - [x] 2.2 Verify the file encodes all existing `temporaryLookupTable` entries correctly

- [x] 3. Create MatchRecorder class
  - [x] 3.1 Create `src/main/java/frc/robot/utils/MatchRecorder.java` as a self-contained class with no subsystem requirements
  - [x] 3.2 In the constructor, spawn a background thread that reads `shooter_lookup_table.csv` from `/home/lvuser/deploy/`, parses each row, and populates `Constants.Shooter.lookUpTable`; catch all exceptions, log via `System.err`, and fall back to compile-time values
  - [x] 3.3 Add `teleopInit(DriverStation)` method that builds the output file path as `shots_<eventCode>_match<matchNumber>_<alliance><station>.csv` using `DriverStation.getEventName()`, `DriverStation.getMatchNumber()`, `DriverStation.getAlliance()`, and `DriverStation.getLocation()`; create `/home/lvuser/match_shots/` if absent; open a `BufferedWriter` and write the CSV header row
  - [x] 3.4 Add `recordShot(RobotContainer rc)` method that captures all fields defined in Requirement 2 AC 1, logs them to AdvantageKit under `MatchRecorder/Shot/`, and dispatches a short-lived `Thread` to append the CSV row to the open writer; the thread closes itself after the write; wrap all I/O in try/catch and log failures via AdvantageKit without throwing
  - [x] 3.5 Add `teleopExit()` method that flushes and closes the `BufferedWriter`; wrap in try/catch and log failures
  - [x] 3.6 Guard `recordShot` so it is a no-op when teleop is not active (check `DriverStation.isTeleop()`)

- [x] 4. Wire MatchRecorder into Robot.java
  - [x] 4.1 Declare a `private MatchRecorder matchRecorder` field in `Robot`
  - [x] 4.2 Instantiate `matchRecorder = new MatchRecorder()` in `robotInit()` (background lookup-table load starts here)
  - [x] 4.3 Call `matchRecorder.teleopInit(DriverStation.getInstance())` in `teleopInit()`
  - [x] 4.4 Call `matchRecorder.teleopExit()` in `teleopExit()`

- [x] 5. Add record-shot button binding in OperatorOI
  - [x] 5.1 Declare `public final Trigger recordShot = this.controller.back()` in `OperatorOI` (back button, easily changed)
  - [x] 5.2 Add a single line in `OperatorOI.configureControls()`: `this.recordShot.onTrue(new InstantCommand(() -> Robot.getInstance().matchRecorder.recordShot(RobotContainer.getInstance())));`
  - [x] 5.3 Uncomment the `this.operatorOI.configureControls()` call in `RobotContainer.configureBindings()` (or add the single binding call there if preferred)

- [x] 6. Create post-match analysis Python script
  - [x] 6.1 Create `scripts/analyze.py` with a `fetch` sub-command that uses `scp` to copy all files from `/home/lvuser/match_shots/` on the RIO (default address `10.TE.AM.2`) to a local `match_data/` directory; print each file name and size; print a message and exit 0 if no files are found
  - [x] 6.2 Add a `plot` sub-command that reads a given Output_Data_File and produces a scatter plot of distance vs. actual shot angle and distance vs. actual flywheel speed, overlaid with the current lookup table curve
  - [x] 6.3 Add a `fit` sub-command that fits a piecewise-linear curve through the recorded data, writes an updated `src/main/deploy/shooter_lookup_table.csv`, and outputs the original points re-projected onto the new curve
