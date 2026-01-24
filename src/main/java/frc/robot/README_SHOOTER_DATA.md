# Shooter Lookup Table Data Collection

## Overview
This system allows you to collect empirical shooter data during testing to build a lookup table for optimal shooter settings at various distances.

## Components

### ShooterDataPoint
Represents a single trial with:
- Distance from goal (meters)
- Shooter velocity (rotations per second)
- Hood angle (degrees)
- Success flag (did the shot score?)
- Notes (optional observations)
- Timestamp (automatically recorded)

### ShooterLookupTableBuilder
Manages data collection and storage:
- Saves data to CSV files on the robot
- Maintains both a session file and master file
- Session file: current testing session only
- Master file: all historical data

### RecordShooterDataCommand
Command to record a data point during testing.

## Usage

### 1. Initialize in Robot.java (with AdvantageKit integration)

```java
private ShooterLookupTableBuilder shooterDataBuilder;
private ShooterDataCollector shooterDataCollector;

@Override
public void robotInit() {
    // ... existing code ...

    shooterDataBuilder = new ShooterLookupTableBuilder();
    shooterDataBuilder.initialize();

    // Use real IO for robot, or create a sim/replay IO for testing
    ShooterDataCollectorIO io = new ShooterDataCollectorIOReal();
    shooterDataCollector = new ShooterDataCollector(shooterDataBuilder, io);

    // Optional: start a new session
    // shooterDataBuilder.startNewSession();
}

@Override
public void robotPeriodic() {
    // ... existing code ...

    // Check for dashboard input and log to AdvantageKit
    shooterDataCollector.periodic();
}
```

### 2. Record Data Points

#### Option A: Via Elastic Dashboard (Recommended)
Add these widgets to your Elastic dashboard:
- **Number Input**: `ShooterData/distance_m` - Distance to goal in meters
- **Number Input**: `ShooterData/velocity_rps` - Shooter velocity in RPS
- **Number Input**: `ShooterData/hood_angle_deg` - Hood angle in degrees
- **Toggle**: `ShooterData/successful` - Was the shot successful?
- **Text Input**: `ShooterData/notes` - Optional notes about the shot
- **Toggle Button**: `ShooterData/record_trigger` - Press to record data point
- **Boolean Indicator**: `ShooterData/recording_status` - Flashes when data is recorded

**Workflow:**
1. Enter distance, velocity, and hood angle values
2. Take the shot
3. Mark successful/unsuccessful
4. Add any notes
5. Press the record trigger button
6. Watch for recording status flash

#### Option B: Manual Recording via Button
```java
// In configureBindings() or similar
driverController.a().onTrue(
    new RecordShooterDataCommand(
        shooterDataBuilder,
        getDistanceToGoal(),  // Your method to get distance
        shooterSubsystem.getVelocityRPS(),
        shooterSubsystem.getHoodAngle(),
        true,  // or false if shot missed
        "Test shot from left side"
    )
);
```

#### Option C: Direct Recording
```java
shooterDataBuilder.recordDataPoint(
    2.5,      // distance in meters
    45.0,     // velocity in RPS
    30.0,     // hood angle in degrees
    true,     // shot was successful
    "Good shot, slight left drift"
);
```

### 3. Access Data Files
Data is saved to: `/home/lvuser/shooter_data/`
- `current_session.csv` - Current testing session
- `shooter_lookup_data.csv` - All historical data
- `current_session_YYYYMMDD_HHMMSS.csv` - Archived sessions

## Data Format (CSV)
```
distance_m,velocity_rps,hood_angle_deg,timestamp,successful,notes
2.500,45.000,30.000,1706140800000,true,Good shot
3.200,48.500,32.500,1706140850000,false,Shot high
```

## Workflow

1. Position robot at known distance from goal
2. Set shooter velocity and hood angle
3. Take shot
4. Record data point with success/failure and notes
5. Repeat at different distances and settings
6. Download CSV files from robot for analysis
7. Build interpolation table from successful shots

## AdvantageKit Integration

All shooter data collection is automatically logged to AdvantageKit:

- **Inputs logged**: All dashboard inputs (distance, velocity, angle, success, notes, trigger)
- **Outputs logged**: Last recorded data point details and total points count
- **Replay support**: You can replay log files and see exactly what data was collected
- **Analysis**: Use AdvantageScope to visualize your data collection sessions

**Logged topics:**

- `ShooterDataCollector/distanceMeters`
- `ShooterDataCollector/velocityRPS`
- `ShooterDataCollector/hoodAngleDegrees`
- `ShooterDataCollector/successful`
- `ShooterDataCollector/notes`
- `ShooterDataCollector/recordTrigger`
- `ShooterDataCollector/totalRecordedPoints`
- `ShooterDataCollector/LastRecorded/*` - Details of most recent data point

## Tips
- Record both successful and unsuccessful shots for analysis
- Include detailed notes about conditions (battery voltage, etc.)
- Test multiple shots at each distance for consistency
- Use `startNewSession()` to separate different testing days
