package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;

public class VisionManager extends SubsystemBase {

  private static VisionManager instance = new VisionManager();

  public static VisionManager getInstance() {
    return instance;
  }

  public VisionManager() {
    System.out.println("====================Vision Manager Online====================");
  }

  @Override
  public void periodic() {
    logPose();
    updatePoseEstimator();
  }

  /**
   * Updates the drivetrain pose estimator with vision measurements
   */
  private void updatePoseEstimator() {
    // Try MegaTag2 first (requires robot orientation updates)
    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-dihlite");
    
    // Fallback to MegaTag1 if MegaTag2 is unavailable
    if (!LimelightHelpers.validPoseEstimate(mt2)) {
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-dihlite");
    }
    
    if (LimelightHelpers.validPoseEstimate(mt2)) {
      // Reject measurements that are unreliable
      if (mt2.avgTagDist > 4.0) {
        SmartDashboard.putBoolean("Vision/ValidMeasurement", false);
        SmartDashboard.putString("Vision/RejectReason", "Distance > 4m");
        return;
      }
      
      if (mt2.tagCount < 1) {
        SmartDashboard.putBoolean("Vision/ValidMeasurement", false);
        SmartDashboard.putString("Vision/RejectReason", "No tags");
        return;
      }
      
      // Calculate standard deviations dynamically based on measurement quality
      // Closer targets and more tags = more confidence (lower std dev)
      double xyStdDev = 0.01 * Math.pow(mt2.avgTagDist, 2.0) / mt2.tagCount;
      double thetaStdDev = 0.01 * Math.pow(mt2.avgTagDist, 2.0) / mt2.tagCount;
      
      // Clamp standard deviations to reasonable ranges
      xyStdDev = Math.max(xyStdDev, 0.02);    // Minimum 2cm std dev
      thetaStdDev = Math.max(thetaStdDev, 0.05); // Minimum ~3 degree std dev
      
      // Add the vision measurement to the drivetrain's pose estimator
      RobotContainer.drivetrain.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds
      );
      
      // Telemetry
      SmartDashboard.putBoolean("Vision/ValidMeasurement", true);
      SmartDashboard.putNumber("Vision/TagCount", mt2.tagCount);
      SmartDashboard.putNumber("Vision/AvgDist", mt2.avgTagDist);
      SmartDashboard.putNumber("Vision/XYStdDev", xyStdDev);
      SmartDashboard.putNumber("Vision/ThetaStdDev", thetaStdDev);
      SmartDashboard.putString("Vision/RejectReason", "None");
    } else {
      SmartDashboard.putBoolean("Vision/ValidMeasurement", false);
      SmartDashboard.putString("Vision/RejectReason", "Invalid estimate");
    }
  }

  public Pose3d getPose() {
    return LimelightHelpers.getCameraPose3d_TargetSpace("limelight-dihlite");
  }

  public void logPose() {
    var pose = getPose();

    if (pose == null) {
      SmartDashboard.putNumber("LRPOSE", 0);
      SmartDashboard.putNumber("FBPOSE", 0);
      SmartDashboard.putNumber("ROTPOSE", 0);
      return;
    }

    SmartDashboard.putNumber("LRPOSE", Math.round(pose.getX() * 100000.0) / 100000.0);
    SmartDashboard.putNumber("FBPOSE", Math.round(pose.getZ() * 100000.0) / 100000.0);

    if (pose.getRotation() != null) {
      SmartDashboard.putNumber("ROTPOSE", Math.toDegrees(pose.getRotation().getY()));
    } else {
      SmartDashboard.putNumber("ROTPOSE", 0);
    }
  }

  public double deriveLRPose() {
    var pose = getPose();
    if (pose == null) {
      return 0.0;
    } else {
      return pose.getX();
    }
  }

  public double deriveFBPose() {
    var pose = getPose();
    if (pose == null) {
      return 0.0;
    } else {
      return pose.getZ();
    }
  }

  public double deriveRotPose() {
    var pose = getPose();

    if (pose == null) {
      return 0.0;
    } else {
      if (pose.getRotation() != null) {
        return Math.toDegrees(pose.getRotation().getY());
      } else {
        return 0.0;
      }
    }
  }
}