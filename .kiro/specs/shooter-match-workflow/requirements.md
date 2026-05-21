# Requirements Document

## Introduction

The Shooter Match Workflow extends the existing shop-tuning `ShooterDataCollector` into a match-day development loop. The workflow has three phases: pre-match (deploy a versioned lookup table to the RIO), in-match (operator-triggered recording of shot telemetry to AdvantageKit logs and an output file), and post-match (retrieve the output file, plot it, and update the lookup table for the next deploy). All robot-code changes must have a minimal surface area so they can be disabled quickly if they cause loop overruns or crashes.

## Glossary

- **Match_Recorder**: The new in-robot component responsible for capturing shot telemetry during a match.
- **Lookup_Table_File**: A JSON or CSV file checked into source control that encodes the `InterpolatingTreeMap` data points deployed to the RIO on every build.
- **Output_Data_File**: A CSV file written by the Match_Recorder to the RIO's persistent storage during a match, containing one row per recorded shot.
- **Analysis_Script**: An offline Python script that reads the Output_Data_File, plots the data, and generates an updated Lookup_Table_File.
- **RIO**: The roboRIO onboard computer running the robot code.
- **DS_Laptop**: The Driver Station laptop connected to the robot over the field network.
- **AdvantageKit**: The structured logging framework (AdvantageKit / WPILog) already in use on the robot.
- **AimValues**: The existing record in `Constants.Shooter` holding a hood angle and flywheel velocity for a given distance.
- **Nudge**: A runtime offset applied to the shooter angle or speed via operator input, tracked by the existing `ShooterIOReal` nudge mechanism.
- **Match_Timestamp**: The elapsed teleop time in seconds, obtained from `RobotContainer.getTeleopMatchTime()`.
- **Operator_Controller**: The second Xbox controller (`joystick2`) managed by `OperatorOI`.

---

## Requirements

### Requirement 1: Versioned Lookup Table File

**User Story:** As a drive team member, I want the shooter lookup table to be stored in a file that is checked into source control and deployed to the RIO on every build, so that I can revert to a known-good table between matches using `git revert`.

#### Acceptance Criteria

1. THE Lookup_Table_File SHALL be stored under `src/main/deploy/` so that the WPILib Gradle build system automatically copies it to the RIO's `/home/lvuser/deploy/` directory on every code deploy.
2. THE Lookup_Table_File SHALL encode all distance-to-AimValues mappings currently defined in `Constants.Shooter.temporaryLookupTable` in a human-readable format (CSV or JSON).
3. WHEN the robot boots, THE Match_Recorder SHALL read the Lookup_Table_File from the RIO filesystem on a background thread so that the robot loop is not blocked during startup.
4. WHEN the Lookup_Table_File is successfully loaded, THE Match_Recorder SHALL populate `Constants.Shooter.lookUpTable` with the parsed entries, replacing the compile-time static initializer values.
5. IF the Lookup_Table_File cannot be read or parsed, THEN THE Match_Recorder SHALL log an error via `System.err` and retain the compile-time static initializer values already present in `Constants.Shooter.lookUpTable`.
6. THE Lookup_Table_File SHALL be tracked in the git repository so that changes are auditable and reversible.

---

### Requirement 2: In-Match Shot Recording

**User Story:** As a drive team member, I want to press a button on the operator controller during a match to capture a snapshot of the current shot parameters, so that I can analyze real-match data in the pits.

#### Acceptance Criteria

1. WHEN the operator presses the designated record button during teleop, THE Match_Recorder SHALL capture and store one shot record containing:
   - Match timestamp (seconds of elapsed teleop time)
   - Distance to target (meters, from `CommandSwerveDrivetrain.getDistanceFromHub()`)
   - Rotation to target (degrees, from the drivetrain's heading-to-hub calculation)
   - Computed target shot angle (degrees, from `Constants.Shooter.lookUpTable` at the current distance)
   - Actual target shot angle (degrees, including any active Nudge offsets)
   - Target hood angle (degrees, the setpoint sent to the hood motor)
   - Current hood angle (degrees, from `Shooter.getHoodAngle()`)
   - Target flywheel speed (rotations per second, the setpoint)
   - Current flywheel speed (rotations per second, from `Shooter.getFlywheelVelocity()`)
2. THE Match_Recorder SHALL log each captured shot record to AdvantageKit under the key prefix `MatchRecorder/Shot/`.
3. THE Match_Recorder SHALL append each captured shot record as a CSV row to the Output_Data_File on the RIO filesystem by dispatching the write to a short-lived worker thread, so that the robot loop thread is never blocked on file I/O.
4. WHEN the CSV write worker thread completes the write, THE worker thread SHALL terminate itself.
5. THE Match_Recorder SHALL use a single operator button binding added to `OperatorOI` so that the entire feature can be disabled by commenting out one line in `RobotContainer` or `OperatorOI`.
6. WHILE teleop is not active, THE Match_Recorder SHALL NOT record shot data when the record button is pressed.
7. IF writing to the Output_Data_File fails, THEN THE Match_Recorder SHALL log the error via AdvantageKit and continue robot operation without throwing an exception.

---

### Requirement 3: Output Data File Management

**User Story:** As a drive team member, I want each match's shot data saved to a predictable file on the RIO, so that I can retrieve it after the match for analysis.

#### Acceptance Criteria

1. THE Match_Recorder SHALL write the Output_Data_File to `/home/lvuser/match_shots/` on the RIO filesystem.
2. WHEN teleop begins, THE Match_Recorder SHALL create a new Output_Data_File named `shots_<eventCode>_match<matchNumber>_<alliance><station>.csv`, where `<eventCode>` is obtained from `DriverStation.getEventName()`, `<matchNumber>` from `DriverStation.getMatchNumber()`, `<alliance>` from `DriverStation.getAlliance()`, and `<station>` from `DriverStation.getLocation()`, so that files from different matches do not overwrite each other and the filename is meaningful without relying on the RIO wall-clock time.
3. THE Output_Data_File SHALL begin with a CSV header row matching the fields defined in Requirement 2, Acceptance Criterion 1.
4. THE Match_Recorder SHALL flush and close the Output_Data_File when teleop ends so that the file is not corrupted if the RIO loses power.
5. IF the `/home/lvuser/match_shots/` directory does not exist, THEN THE Match_Recorder SHALL create it before writing.

---

### Requirement 4: Post-Match File Retrieval

**User Story:** As a drive team member, I want to retrieve the Output_Data_File from the RIO to the DS laptop after a match, so that I can run the analysis script in the pits.

#### Acceptance Criteria

1. THE Analysis_Script SHALL include a retrieval sub-command (e.g., `python analyze.py fetch`) that uses `scp` or the WPILib `RobotPy` file transfer utilities to copy all files from `/home/lvuser/match_shots/` on the RIO to a local `match_data/` directory on the DS laptop.
2. WHEN the retrieval sub-command is run, THE Analysis_Script SHALL print the name and size of each file copied.
3. IF no new files are found on the RIO, THEN THE Analysis_Script SHALL print a message indicating no data was retrieved and exit with code 0.

---

### Requirement 5: Data Analysis and Curve Fitting

**User Story:** As a drive team member, I want a script that plots the match shot data and fits a new lookup table curve, so that I can update the robot's lookup table between matches.

#### Acceptance Criteria

1. WHEN given an Output_Data_File, THE Analysis_Script SHALL produce a scatter plot of distance vs. actual shot angle and distance vs. actual flywheel speed, overlaid with the current lookup table curve.
2. THE Analysis_Script SHALL fit a new smooth curve through the recorded data points using a standard interpolation method (e.g., piecewise linear or polynomial regression).
3. THE Analysis_Script SHALL output an updated Lookup_Table_File in the same format as the deployed file (Requirement 1, Criterion 2), containing the refitted data points.
4. THE Analysis_Script SHALL also output the original data points re-projected onto the new curve so that the full dataset remains consistent.
5. WHEN the Analysis_Script writes the updated Lookup_Table_File, THE Analysis_Script SHALL write it to the `src/main/deploy/` path so it is ready for the next `git commit` and deploy.

---

### Requirement 6: Minimal Robot Code Surface Area

**User Story:** As a developer, I want all match-recording robot code changes to be isolated and easy to disable, so that I can quickly remove the feature if it causes loop overruns or crashes during a match.

#### Acceptance Criteria

1. THE Match_Recorder SHALL be implemented as a single self-contained class with no required changes to existing subsystem logic in `Shooter.java`, `ShooterIOReal.java`, or `Superstructure.java`.
2. THE Match_Recorder SHALL be instantiated and wired in `Robot.java` and `RobotContainer.java` only, so that removing it requires deleting at most two call sites.
3. THE Match_Recorder's operator button binding SHALL be added as a single line in `OperatorOI.configureControls()` so that it can be commented out independently of all other bindings.
4. THE Match_Recorder SHALL NOT require any changes to the WPILib command scheduler's subsystem requirements or default commands.
5. IF the Match_Recorder's background file-read thread throws an uncaught exception, THEN THE Match_Recorder SHALL catch the exception, log it, and allow the robot to continue operating with compile-time default values.
