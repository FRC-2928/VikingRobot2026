package frc.robot.vision;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class Limelight {
    public final NetworkTable nt;
    public final String limelightName;
    private Vector<N3> limelightTrust;

    public enum IMUMode {
        MODE_0_EXTERNAL_ONLY(0),             // No internal IMU processing. MT2 uses interpolated yaw from robot's gyro
                                             // sent via SetRobotOrientation().
                                             //
        MODE_1_EXTERNAL_SEED(1),             // Internal IMU offset is calibrated to match external yaw each
                                             // frame (seeding). MT2 still uses external yaw for botpose.
                                             //
        MODE_2_INTERNAL_ONLY(2),             // Uses internal IMU's fused yaw only. No external input required.
                                             //
        MODE_3_INTERNAL_MT1_ASSIST(3),       // Complementary filter fuses internal IMU with MT1 vision yaw. When MT1
                                             // gets a valid pose, it slowly corrects internal IMU drift.
                                             //
        MODE_4_INTERNAL_EXTERNAL_ASSIST(4);  // Complementary filter fuses internal IMU with external yaw from 
                                             // SetRobotOrientation(). This is the recommended mode, as the internal
                                             // IMU's 1khz update rate is utilized for frame-by-frame motion while the
                                             // robot's IMU corrects for any drift over time.

        /// integer representing the IMU Mode
        private int mIMUMode;
        private IMUMode(int imuMode) {
            mIMUMode = imuMode;
        }

        public int getImuMode() {
            return mIMUMode;
        }
    }

    public Limelight(final String limelightName) {
        this.nt = NetworkTableInstance.getDefault().getTable(limelightName);
        this.limelightName = limelightName;
        this.setStream(0);
    }

    public void setStream(final int stream) {
        this.nt.getEntry("stream").setNumber(stream);
    }

    public void setPipeline(final int pipeline) {
        this.nt.getEntry("pipeline").setNumber(pipeline);
    }

    // Whether the limelight has any valid targets (0 or 1)
    public boolean hasValidTargets() {
        if (RobotBase.isReal()) {
            return this.nt.getEntry("tv").getDouble(0) == 1;
        } else {
            // return this value in simulation
            return false;
        }
    }

    public LimelightResults getResults() {
        return LimelightHelpers.getLatestResults(this.limelightName);
    }

    public int getTargetAprilTagID() {
        return (int) LimelightHelpers.getFiducialID(this.limelightName);
    }

    public String getLimelightName(){
		return limelightName;
	}

    public void setLimelighTrust(double x, double y, double theta){
        this.limelightTrust = VecBuilder.fill(x, y, theta);
    }

    public Vector<N3> getLimelightTrust(){
        if(limelightTrust == null){
            return VecBuilder.fill(0.7,0.7,9999999);
        }
        return limelightTrust;
    }


    // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    // @AutoLogOutput(key = "Limelight/Horizontal Offset")
    public Angle getTargetHorizontalOffset() {
        return Units.Degrees.of(this.nt.getEntry("tx").getDouble(0));
    }

    // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    // @AutoLogOutput(key = "Limelight/Vertical Offset")
    public Angle getTargetVerticalOffset() {
        return Units.Degrees.of(this.nt.getEntry("ty").getDouble(0));
    }

    public int getNumberOfAprilTags() {
        final LimelightResults reultsOfJson = LimelightHelpers.getLatestResults(this.limelightName);
        return reultsOfJson.targets_Fiducials.length;
    }

    // Target Area (0% of image to 100% of image)
    public double getTargetArea() {
        return this.nt.getEntry("ta").getDouble(0);
    }

    public double getTargetSkew() {
        return this.nt.getEntry("ts").getDouble(0);
    }

    public double getLatency() {
        return (this.nt.getEntry("tl").getDouble(0) + this.nt.getEntry("cl").getDouble(0));
    }

    // Robot transform in 3D field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
    public Pose3d getPose3d() {
        return LimelightHelpers.getBotPose3d(this.limelightName);
    }

    public PoseEstimate getPoseMegatag2() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.limelightName);
    }

    public PoseEstimate getPoseMegatag1() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(this.limelightName);
    }

    public void setRobotOrientation(Angle yaw) {
        LimelightHelpers.SetRobotOrientation(limelightName, yaw.in(Units.Degrees), 0, 0, 0, 0, 0);
    }

    public void setIMUMode(IMUMode mode) {
        LimelightHelpers.SetIMUMode(limelightName, mode.getImuMode());
    }

    public double getImuMode() {
        return LimelightHelpers.getLimelightNTDouble(this.limelightName, "imumode_set");
    }

    public void setThrottleRate(int rate) {
        Logger.recordOutput("Odometry/Limelight/ThrottleRate", rate);
        LimelightHelpers.SetThrottle(this.limelightName, rate);
    }

    // Robot transform in 2D field-space. Translation (X,Y) Rotation(Z)
    @AutoLogOutput(key = "Odometry/Limelight/Pose")
    public Pose2d getPose2d() {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            return LimelightHelpers.getBotPose2d_wpiRed(this.limelightName);
        } else {
            return LimelightHelpers.getBotPose2d_wpiBlue(this.limelightName);
        }
    }

    // @AutoLogOutput(key = "Odometry/BotPose")
    public Pose2d getBotPose2d() {
        return LimelightHelpers.getBotPose2d(this.limelightName);
    }

    // Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(X,Y,Z)
    public Pose3d getBluePose3d() {
        return LimelightHelpers.getBotPose3d_wpiBlue(this.limelightName);
    }

    public Pose3d getBotPose3d_TargetSpace() {
        return LimelightHelpers.getBotPose3d_TargetSpace(this.limelightName);
    }

    public Pose2d getBluePose2d() {
        return LimelightHelpers.getBotPose2d_wpiBlue(this.limelightName);
    }

    public Pose2d getRedPose2d() {
        return LimelightHelpers.getBotPose2d_wpiRed(this.limelightName);
    }

    // Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(X,Y,Z)
    public Pose3d getRedPose3d() {
        return LimelightHelpers.getBotPose3d_wpiRed(this.limelightName);
    }

    // 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
    // @AutoLogOutput(key = "limelight/Pose3d")
    public Pose3d getRobotTagPose3d() {
        return LimelightHelpers.getTargetPose3d_RobotSpace(this.limelightName);
    }

    // 3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
    public Pose3d getCameraTagPose3d() {
        return LimelightHelpers.getTargetPose3d_CameraSpace(this.limelightName);
    }

    public static int getClosestTagId(final PoseEstimate pose) {
        double closestTagDistance = 999999;
        int closestTagId = 0;
        for (RawFiducial tag : pose.rawFiducials) {
            if (tag.distToCamera < closestTagDistance) {
                closestTagDistance = tag.distToCamera;
                closestTagId = tag.id;
            }
        }
        return closestTagId;
    }
}
