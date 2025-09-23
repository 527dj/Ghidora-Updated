package frc.robot.commands;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Drivetrain;

/**
 * Auto-aligns the robot to a REEF face using ONLY Limelight data.
 * - X (forward) and Y (strafe) are driven using TargetPose in ROBOT-SPACE
 * - Rotation is driven from tx_nocrosshair (falls back to tx) with a profiled
 * PID
 * - No odometry, no field pose, no external truth.
 *
 * Coordinate assumptions (Limelight Robot-Space):
 * X = forward to target, Y = left (+) to target, Z = up. (LL Robot-Space
 * config)
 * We use TX (zero-crosshair if available) for heading.
 *
 * Tag filtering:
 * Defaults to REEF tag IDs {6..11, 17..22}. You can override via ctor.
 *
 * Finish condition:
 * Hold inside X/Y/theta tolerances for a short time or timeout.
 */
public class DrivetrainReefAutoAlignProfiled extends Command {

    public enum Branch {
        LEFT, CENTER, RIGHT
    }

    private final Drivetrain drivetrain;
    private final String llName;
    private final Branch branch;
    private final Set<Integer> allowedTagIds;

    // ---------- Tunables (most from Constants; override via fluent setters)
    // ----------
    // Desired standoff (distance from robot to tag center along X in robot-space)
    // Use your Constants.FB_Setpoint (meters)
    private double xSetpoint_m = Constants.FB_Setpoint;

    // Lateral offsets to branches (meters, robot-space Y left-positive)
    private double ySetpointLeft_m = Constants.L_Setpoint;
    private double ySetpointRight_m = Constants.R_Setpoint;

    // Rotational target: we zero tx (degrees). (You may change to use tag yaw if
    // desired)
    private double txSetpoint_deg = 0.0;

    // PID: forward / strafe use meters, deg PID uses degrees (converted to rad/s on
    // output)
    private final ProfiledPIDController pidX = new ProfiledPIDController(
            Constants.FB_kP, Constants.FB_kI, Constants.FB_kD,
            new Constraints(Constants.FB_kVelo, Constants.FB_kAccel));

    private final ProfiledPIDController pidY = new ProfiledPIDController(
            Constants.LR_kP, Constants.LR_kI, Constants.LR_kD,
            new Constraints(Constants.LR_kVelo, Constants.LR_kAccel));

    private final ProfiledPIDController pidYawDeg = new ProfiledPIDController(
            Constants.Rot_kP, Constants.Rot_kI, Constants.Rot_kD,
            new Constraints(Math.toDegrees(Constants.Rot_kVelo),
                    Math.toDegrees(Constants.Rot_kAccel)));

    // Tolerances
    private double tolX_m = 0.02;
    private double tolY_m = 0.02;
    private double tolYaw_deg = 0.6;

    // Output limits
    private double maxVX_mps = Constants.DrivetrainMaxSpeed;
    private double maxVY_mps = Constants.DrivetrainMaxSpeed;
    private double maxOmega_radps = Constants.DrivetrainMaxAngularRate;

    // Behavior / timing
    private double stableHoldS = 0.18;
    private double timeoutS = 2.5;
    private double searchOmega_radps = 0.8;

    // Internal
    private boolean xReset = false, yReset = false, yawReset = false;
    private final Timer stableTimer = new Timer();
    private final Timer totalTimer = new Timer();

    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    /** Use default REEF tags per 2025 manual (6–11, 17–22). */
    public DrivetrainReefAutoAlignProfiled(Drivetrain drivetrain, String limelightName, Branch branch) {
        this(drivetrain, limelightName, branch,
                new HashSet<>(Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22)));
    }

    public DrivetrainReefAutoAlignProfiled(Drivetrain drivetrain, String limelightName, Branch branch,
            Set<Integer> allowedTagIds) {
        this.drivetrain = drivetrain;
        this.llName = (limelightName == null || limelightName.isBlank()) ? "limelight" : limelightName;
        this.branch = branch;
        this.allowedTagIds = allowedTagIds;
        addRequirements(drivetrain);

        pidX.setTolerance(tolX_m);
        pidY.setTolerance(tolY_m);
        pidYawDeg.setTolerance(tolYaw_deg);
    }

    // ---------- Fluent tuning ----------
    public DrivetrainReefAutoAlignProfiled withStandoff(double meters) {
        this.xSetpoint_m = meters;
        return this;
    }

    public DrivetrainReefAutoAlignProfiled withBranchOffsets(double left_m, double right_m) {
        this.ySetpointLeft_m = left_m;
        this.ySetpointRight_m = right_m;
        return this;
    }

    public DrivetrainReefAutoAlignProfiled withTolerances(double tolX_m, double tolY_m, double tolYaw_deg) {
        this.tolX_m = tolX_m;
        this.tolY_m = tolY_m;
        this.tolYaw_deg = tolYaw_deg;
        pidX.setTolerance(tolX_m);
        pidY.setTolerance(tolY_m);
        pidYawDeg.setTolerance(tolYaw_deg);
        return this;
    }

    public DrivetrainReefAutoAlignProfiled withLimits(double maxVX, double maxVY, double maxOmega) {
        this.maxVX_mps = maxVX;
        this.maxVY_mps = maxVY;
        this.maxOmega_radps = maxOmega;
        return this;
    }

    public DrivetrainReefAutoAlignProfiled withTiming(double holdS, double timeoutS) {
        this.stableHoldS = holdS;
        this.timeoutS = timeoutS;
        return this;
    }

    public DrivetrainReefAutoAlignProfiled withSearchOmega(double radps) {
        this.searchOmega_radps = radps;
        return this;
    }

    @Override
    public void initialize() {
        stableTimer.stop();
        stableTimer.reset();
        totalTimer.stop();
        totalTimer.reset();
        totalTimer.start();
        xReset = yReset = yawReset = false;
    }

    @Override
    public void execute() {
        LimelightResults results = LimelightHelpers.getLatestResults(llName);
        if (results == null || !results.valid || results.targets_Fiducials == null
                || results.targets_Fiducials.length == 0) {
            // No tag -> slow search spin (robot-centric)
            drivetrain.setControl(request
                    .withVelocityX(0).withVelocityY(0).withRotationalRate(searchOmega_radps));
            resetStable();
            return;
        }

        // Choose a suitable tag (reef IDs by default), prefer closest by XY range
        Optional<LimelightTarget_Fiducial> chosen = chooseReefTarget(results.targets_Fiducials);
        if (chosen.isEmpty()) {
            drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(searchOmega_radps));
            resetStable();
            return;
        }
        LimelightTarget_Fiducial tag = chosen.get();

        // ROBOT-SPACE target pose -> meters (X forward, Y left). (from Limelight JSON
        // t6t_rs)
        var tgtPoseRS = tag.getTargetPose_RobotSpace();

        double x_meas_m = tgtPoseRS.getX(); // forward distance to tag center
        double y_meas_m = tgtPoseRS.getY(); // left (+) of tag center
        // Rotation from image-space: prefer zero-crosshair tx (deg), fallback to tx
        // (deg)
        double tx_deg = (Math.abs(tag.tx_nocrosshair) > 1e-6) ? tag.tx_nocrosshair : tag.tx;

        // Choose Y setpoint based on requested branch
        double y_set_m = switch (branch) {
            case LEFT -> ySetpointLeft_m; // usually negative (tag center should be to robot right)
            case RIGHT -> ySetpointRight_m; // usually positive (tag center to robot left)
            case CENTER -> 0.0;
        };

        // First good sample -> reset profiled PIDs to the current measurement
        if (!xReset) {
            pidX.reset(x_meas_m);
            xReset = true;
        }
        if (!yReset) {
            pidY.reset(y_meas_m);
            yReset = true;
        }
        if (!yawReset) {
            pidYawDeg.reset(tx_deg);
            yawReset = true;
        }

        // Calculate profiled outputs (velocity-like commands)
        double vx_cmd = MathUtil.clamp(pidX.calculate(x_meas_m, xSetpoint_m), -maxVX_mps, maxVX_mps);
        double vy_cmd = MathUtil.clamp(pidY.calculate(y_meas_m, y_set_m), -maxVY_mps, maxVY_mps);

        // Yaw controller runs in *degrees*, convert output deg/s -> rad/s for swerve
        // request
        double omega_degps = pidYawDeg.calculate(tx_deg, txSetpoint_deg);
        double omega_cmd = MathUtil.clamp(Math.toRadians(omega_degps), -maxOmega_radps, maxOmega_radps);

        drivetrain.setControl(request
                .withVelocityX(vx_cmd)
                .withVelocityY(vy_cmd)
                .withRotationalRate(omega_cmd));

        // Finish gating
        boolean atX = pidX.atSetpoint();
        boolean atY = pidY.atSetpoint();
        boolean atYaw = pidYawDeg.atSetpoint();

        SmartDashboard.putBoolean("ReefAlign/AtX", atX);
        SmartDashboard.putBoolean("ReefAlign/AtY", atY);
        SmartDashboard.putBoolean("ReefAlign/AtYaw", atYaw);
        SmartDashboard.putNumber("ReefAlign/vx_cmd", vx_cmd);
        SmartDashboard.putNumber("ReefAlign/vy_cmd", vy_cmd);
        SmartDashboard.putNumber("ReefAlign/omega_cmd", omega_cmd);

        if (atX && atY && atYaw) {
            if (!stableTimer.isRunning()) {
                stableTimer.reset();
                stableTimer.start();
            }
        } else {
            resetStable();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        if (totalTimer.hasElapsed(timeoutS))
            return true;
        return stableTimer.isRunning() && stableTimer.hasElapsed(stableHoldS);
    }

    private void resetStable() {
        stableTimer.stop();
        stableTimer.reset();
    }

    /** Pick allowed tag with smallest range in XY (robot-space). */
    private Optional<LimelightTarget_Fiducial> chooseReefTarget(LimelightTarget_Fiducial[] arr) {
        LimelightTarget_Fiducial best = null;
        double bestDist = Double.POSITIVE_INFINITY;
        for (var t : arr) {
            int id = (int) Math.round(t.fiducialID);
            if (!allowedTagIds.isEmpty() && !allowedTagIds.contains(id))
                continue;
            var p = t.getTargetPose_RobotSpace();
            double d = Math.hypot(p.getX(), p.getY());
            if (d < bestDist) {
                bestDist = d;
                best = t;
            }
        }
        return Optional.ofNullable(best);
    }
}
