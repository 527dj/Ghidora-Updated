package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.RobotContainer;

public class VisionManager extends SubsystemBase {

  public static VisionManager getInstance() {
    return instance;
  }

  private static VisionManager instance = new VisionManager();

  public VisionManager() {
    System.out.println("====================Vision Manager Online====================");
  }

  @Override
  public void periodic() {
    logPose();
  }

  public Pose3d getPose() {
    LimelightResults results = LimelightHelpers.getLatestResults("dihlite");

    if (results.targets_Fiducials.length > 0) {
      LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
      return tag.getRobotPose_TargetSpace();
    } else {
      return null;
    }
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