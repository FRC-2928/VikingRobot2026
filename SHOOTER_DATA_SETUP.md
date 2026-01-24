# Shooter Data Collection - Quick Setup Guide

## What I've Done

I've integrated the shooter data collection system into your robot code. Here's what's ready to go:

### Files Added
- `src/main/java/frc/robot/utils/ShooterDataPoint.java` - Data structure for a single trial
- `src/main/java/frc/robot/utils/ShooterLookupTableBuilder.java` - Manages CSV file storage
- `src/main/java/frc/robot/utils/ShooterDataCollectorIO.java` - AdvantageKit IO interface
- `src/main/java/frc/robot/utils/ShooterDataCollectorIOReal.java` - NetworkTables implementation
- `src/main/java/frc/robot/utils/ShooterDataCollector.java` - Main collector with logging
- `src/main/java/frc/robot/commands/RecordShooterDataCommand.java` - Optional command wrapper
- `elastic_dashboard_layout.json` - Sample dashboard configuration

### Files Modified
- `src/main/java/frc/robot/Robot.java` - Added initialization and periodic calls

## How to Use It

### Step 1: Deploy to Robot
Just deploy your code as normal. The system is already integrated.

### Step 2: Set Up Elastic Dashboard

Add these widgets to your Elastic dashboard (or import `elastic_dashboard_layout.json`):

1. **Number Input** - Topic: `ShooterData/distance_m`
   - Label: "Distance (m)"
   - Min: 0, Max: 10, Step: 0.1

2. **Number Input** - Topic: `ShooterData/velocity_rps`
   - Label: "Velocity (RPS)"
   - Min: 0, Max: 100, Step: 0.5

3. **Number Input** - Topic: `ShooterData/hood_angle_deg`
   - Label: "Hood Angle (deg)"
   - Min: 0, Max: 90, Step: 0.5

4. **Toggle Button** - Topic: `ShooterData/successful`
   - Label: "Shot Successful?"

5. **Text View** - Topic: `ShooterData/notes`
   - Label: "Notes"

6. **Toggle Button** - Topic: `ShooterData/record_trigger`
   - Label: "RECORD DATA"
   - Color when true: Green
   - Color when false: Red

7. **Boolean Box** - Topic: `ShooterData/recording_status`
   - Label: "Recording Status"
   - Shows green flash when data is saved

### Step 3: Collect Data

**Testing Workflow:**
1. Position robot at a known distance from the goal
2. Enter the distance in the dashboard
3. Set your shooter velocity and hood angle (either manually or from your shooter subsystem)
4. Enter those values in the dashboard
5. Take the shot
6. Mark it as successful or not
7. Add any notes (optional)
8. Press the "RECORD DATA" button
9. Watch for the green flash confirming it was saved

**Repeat** at different distances and settings to build your dataset.

### Step 4: Retrieve Your Data

After testing, download the CSV files from the robot:

**Location:** `/home/lvuser/shooter_data/`

**Files:**
- `current_session.csv` - Your current testing session
- `shooter_lookup_data.csv` - All historical data
- `current_session_YYYYMMDD_HHMMSS.csv` - Archived sessions

**How to download:**
- Use WinSCP, FileZilla, or `scp` command
- Connect to: `lvuser@10.29.28.2` (adjust for your team number)
- Password: (blank)

### Step 5: Analyze and Build Lookup Table

Open the CSV files in Excel, Python, or your preferred tool:
- Filter for successful shots only
- Plot distance vs velocity and distance vs hood angle
- Fit curves or create interpolation tables
- Implement the lookup table in your shooter subsystem

## AdvantageKit Integration

All data is automatically logged! You can:
- View collection sessions in AdvantageScope
- Replay log files to see what you tested
- Analyze trends across multiple sessions

**Logged Topics:**
- `ShooterDataCollector/distanceMeters`
- `ShooterDataCollector/velocityRPS`
- `ShooterDataCollector/hoodAngleDegrees`
- `ShooterDataCollector/successful`
- `ShooterDataCollector/notes`
- `ShooterDataCollector/totalRecordedPoints`
- `ShooterDataCollector/LastRecorded/*` - Most recent data point

## Optional: Button Trigger

If you want to trigger recording with a controller button instead of the dashboard:

```java
// In DriverOI or OperatorOI configureControls()
controller.a().onTrue(
    new RecordShooterDataCommand(
        Robot.instance.shooterDataBuilder,
        getDistanceToGoal(),  // Your method
        getShooterVelocity(), // Your method
        getHoodAngle(),       // Your method
        true,                 // or determine programmatically
        "Auto-recorded shot"
    )
);
```

## Tips

- **Start a new session** for each testing day:
  ```java
  // In Robot.java robotInit() or via dashboard command
  shooterDataBuilder.startNewSession();
  ```

- **Record failed shots too** - they help you understand the boundaries

- **Include detailed notes** - battery voltage, field conditions, etc.

- **Test multiple shots** at each distance for consistency

- **Check AdvantageScope** to verify data is being logged correctly

## Troubleshooting

**Dashboard not showing topics?**
- Make sure robot code is deployed and running
- Check NetworkTables connection
- Restart Elastic dashboard

**Data not saving?**
- Check console output for error messages
- Verify `/home/lvuser/shooter_data/` directory exists
- Check robot storage space

**Recording status not flashing?**
- Data might still be saving (check console)
- Verify the record trigger button is working
- Check AdvantageKit logs

## Next Steps

Once you have enough data:
1. Download and analyze your CSV files
2. Create interpolation functions or lookup tables
3. Implement in your shooter subsystem
4. Test and refine!

Good luck with your shooter tuning! 🎯
