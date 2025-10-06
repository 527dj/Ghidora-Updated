package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/**
 * Command to turn the robot to a specific angle using a profiled PID controller.
 * The angle can be specified as absolute (field-relative) or relative (add to current angle).
 */
public class DrivetrainTurnToAngle extends Command {
    private final Drivetrain drivetrain;
    private final double targetAngleDegrees;
    private final boolean isRelative;
    
    // PID Controller for rotation (uses degrees internally, outputs rad/s)
    private final ProfiledPIDController rotationPID;
    
    // Tolerances
    private double angleTolerance = 2.0; // degrees
    private double velocityTolerance = 5.0; // degrees per second
    
    // Timing
    private double stableTime = 0.2; // seconds to hold at target before finishing
    private double timeout = 3.0; // seconds before giving up
    private final Timer stableTimer = new Timer();
    private final Timer totalTimer = new Timer();
    
    // Swerve request
    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    
    private boolean hasReset = false;
    private double actualTargetDegrees;
    
    /**
     * Turn to an absolute field-relative angle
     * @param drivetrain The drivetrain subsystem
     * @param targetAngleDegrees Target angle in degrees (0 = forward, 90 = left, etc.)
     */
    public DrivetrainTurnToAngle(Drivetrain drivetrain, double targetAngleDegrees) {
        this(drivetrain, targetAngleDegrees, false);
    }
    
    /**
     * Turn to an angle, either absolute or relative
     * @param drivetrain The drivetrain subsystem
     * @param targetAngleDegrees Target angle in degrees
     * @param isRelative If true, add this angle to current heading. If false, turn to absolute angle.
     */
    public DrivetrainTurnToAngle(Drivetrain drivetrain, double targetAngleDegrees, boolean isRelative) {
        this.drivetrain = drivetrain;
        this.targetAngleDegrees = targetAngleDegrees;
        this.isRelative = isRelative;
        
        // Create PID controller with constraints from Constants or use reasonable defaults
        this.rotationPID = new ProfiledPIDController(
            0.08,  // kP - tune this value
            0.0,   // kI
            0.005, // kD - tune this value
            new Constraints(
                Math.toRadians(360), // max velocity (deg/s converted to rad/s)
                Math.toRadians(720)  // max acceleration (deg/s² converted to rad/s²)
            )
        );
        
        rotationPID.setTolerance(angleTolerance, velocityTolerance);
        rotationPID.enableContinuousInput(-180, 180); // Handle wraparound
        
        addRequirements(drivetrain);
    }
    
    // Fluent configuration methods
    public DrivetrainTurnToAngle withTolerance(double angleDeg, double velocityDegPerSec) {
        this.angleTolerance = angleDeg;
        this.velocityTolerance = velocityDegPerSec;
        rotationPID.setTolerance(angleDeg, velocityDegPerSec);
        return this;
    }
    
    public DrivetrainTurnToAngle withTiming(double stableTime, double timeout) {
        this.stableTime = stableTime;
        this.timeout = timeout;
        return this;
    }
    
    public DrivetrainTurnToAngle withPID(double kP, double kI, double kD) {
        rotationPID.setP(kP);
        rotationPID.setI(kI);
        rotationPID.setD(kD);
        return this;
    }
    
    public DrivetrainTurnToAngle withConstraints(double maxVelDegPerSec, double maxAccelDegPerSecSq) {
        rotationPID.setConstraints(new Constraints(
            Math.toRadians(maxVelDegPerSec),
            Math.toRadians(maxAccelDegPerSecSq)
        ));
        return this;
    }
    
    @Override
    public void initialize() {
        stableTimer.stop();
        stableTimer.reset();
        totalTimer.stop();
        totalTimer.reset();
        totalTimer.start();
        hasReset = false;
        
        // Calculate actual target based on whether it's relative or absolute
        double currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();
        if (isRelative) {
            actualTargetDegrees = currentAngle + targetAngleDegrees;
        } else {
            actualTargetDegrees = targetAngleDegrees;
        }
        
        // Normalize to -180 to 180
        actualTargetDegrees = MathUtil.inputModulus(actualTargetDegrees, -180, 180);
        
        System.out.println("TurnToAngle: Current=" + currentAngle + 
                         " Target=" + actualTargetDegrees + 
                         " Relative=" + isRelative);
    }
    
    @Override
    public void execute() {
        // Get current angle
        double currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();
        
        // Reset PID on first good reading
        if (!hasReset) {
            rotationPID.reset(currentAngle);
            hasReset = true;
        }
        
        // Calculate rotation rate in degrees/sec, then convert to rad/s
        double rotationDegPerSec = rotationPID.calculate(currentAngle, actualTargetDegrees);
        double rotationRadPerSec = Math.toRadians(rotationDegPerSec);
        
        // Clamp to max angular rate
        rotationRadPerSec = MathUtil.clamp(
            rotationRadPerSec, 
            -Constants.DrivetrainMaxAngularRate, 
            Constants.DrivetrainMaxAngularRate
        );
        
        // Apply rotation (no translation)
        drivetrain.setControl(request
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rotationRadPerSec));
        
        // Check if at setpoint
        boolean atSetpoint = rotationPID.atSetpoint();
        
        SmartDashboard.putBoolean("TurnToAngle/AtSetpoint", atSetpoint);
        SmartDashboard.putNumber("TurnToAngle/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("TurnToAngle/TargetAngle", actualTargetDegrees);
        SmartDashboard.putNumber("TurnToAngle/Error", actualTargetDegrees - currentAngle);
        SmartDashboard.putNumber("TurnToAngle/OutputRadPerSec", rotationRadPerSec);
        
        // Manage stable timer
        if (atSetpoint) {
            if (!stableTimer.isRunning()) {
                stableTimer.reset();
                stableTimer.start();
            }
        } else {
            stableTimer.stop();
            stableTimer.reset();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(request
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
        
        System.out.println("TurnToAngle: " + (interrupted ? "Interrupted" : "Completed"));
    }
    
    @Override
    public boolean isFinished() {
        // Finish on timeout
        if (totalTimer.hasElapsed(timeout)) {
            System.out.println("TurnToAngle: Timeout");
            return true;
        }
        
        // Finish when stable at target
        return stableTimer.isRunning() && stableTimer.hasElapsed(stableTime);
    }
}