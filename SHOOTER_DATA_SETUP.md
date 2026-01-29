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

**Option A: Import the Layout (Easiest)**

1. Open Elastic Dashboard
2. Go to File → Open Layout
3. Select `elastic_dashboard_layout.json` from your project root
4. The "Shooter Data Collection" tab will be created with all widgets configured

**Option B: Manual Setup**

Add these widgets manually to your Elastic dashboard:

1. **Number Slider** - Topic: `/ShooterData/distance_m`
   - Label: "Distance (m)"
   - Min: 0, Max: 10, Block increment: 0.1

2. **Number Slider** - Topic: `/ShooterData/velocity_rps`
   - Label: "Velocity (RPS)"
   - Min: 0, Max: 100, Block increment: 0.5

3. **Number Slider** - Topic: `/ShooterData/hood_angle_deg`
   - Label: "Hood Angle (deg)"
   - Min: 0, Max: 90, Block increment: 0.5

4. **Toggle Switch** - Topic: `/ShooterData/successful`
   - Label: "Shot Successful?"

5. **Text View** - Topic: `/ShooterData/notes`
   - Label: "Notes"

6. **Toggle Button** - Topic: `/ShooterData/record_trigger`
   - Label: "RECORD DATA"

7. **Boolean Box** - Topic: `/ShooterData/recording_status`
   - Label: "Recording"
   - True color: Green (#00FF00)
   - False color: Gray (#808080)

**Important:** Make sure to include the leading `/` in topic names!

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
- Topics are published on initialization - check console for "ShooterData NetworkTables topics published"
- Check NetworkTables connection in Elastic (should show connected)
- Restart Elastic dashboard if needed
- Make sure topic names include the leading `/` (e.g., `/ShooterData/distance_m`)

**Can't import the layout JSON?**
- Make sure you're using Elastic Dashboard (not Shuffleboard or Glass)
- Try File → Open Layout and select the JSON file
- If import fails, add widgets manually using the topic names listed above

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
