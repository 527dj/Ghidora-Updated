package frc.robot.commands;
 import static edu.wpi.first.units.Units.MetersPerSecond;
 import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
 import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
 import com.ctre.phoenix6.swerve.SwerveRequest;
 import edu.wpi.first.math.controller.ProfiledPIDController;
 import edu.wpi.first.math.geometry.Pose3d;
 import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.Command;
 import frc.robot.Constants;
 import frc.robot.TunerConstants;
 import frc.robot.subsystems.Drivetrain;
 import frc.robot.subsystems.VisionManager;
 
 public class DrivetrainRightAlign extends Command {
     private final Drivetrain drivetrain;
     private final VisionManager visionManager;

    public enum ALIGN_STATES {
        LEFT,
        RIGHT,
        MIDDLE
    };
    private ALIGN_STATES targetPosition;
     ProfiledPIDController FBPIDController = new ProfiledPIDController(2.75, 0, 0.01, new Constraints(4.0, 4.0)); //2.75, 0, 0, 4
     ProfiledPIDController LRPIDController = new ProfiledPIDController(6, 0, 0.0, new Constraints(4.0, 4.0));
     ProfiledPIDController rotationPIDController = new ProfiledPIDController(0.1, 0, 0, new Constraints(1.0, 1.0));   
     
     boolean FBPositionHasReset = false;
     boolean LRPositionHasReset = false;
     boolean RotPositionHasReset = false;
 
     public DrivetrainRightAlign(Drivetrain drivetrain, VisionManager visionManager, ALIGN_STATES targetPosition) {
         this.drivetrain = drivetrain;
         this.visionManager = VisionManager.getInstance();
        this.targetPosition = targetPosition;
        FBPIDController.setTolerance(0.1);
        LRPIDController.setTolerance(0.1);
         addRequirements(drivetrain);
         addRequirements(visionManager);
 
         //HotRefreshFBAlignPID
         SmartDashboard.putNumber("FBAlign kP", 0.0);
         SmartDashboard.putNumber("FBAlign kI", 0.0);
         SmartDashboard.putNumber("FBAlign kD", 0.0);
         SmartDashboard.putNumber("FBAlign kVelo", 0.0);
         SmartDashboard.putNumber("FBAlign kAccel", 0.0);
 
         //HotRefreshLRAlignPID
         SmartDashboard.putNumber("LRAlign kP", 0.0);
         SmartDashboard.putNumber("LRAlign kI", 0.0);
         SmartDashboard.putNumber("LRAlign kD", 0.0);
         SmartDashboard.putNumber("LRAlign kVelo", 0.0);
         SmartDashboard.putNumber("LRAlign kAccel", 0.0);
 
         //HotRefreshRotAlignPID
         SmartDashboard.putNumber("RotAlign kP", 0.0);
         SmartDashboard.putNumber("RotAlign kI", 0.0);
         SmartDashboard.putNumber("RotAlign kD", 0.0);
         SmartDashboard.putNumber("RotAlign kVelo", 0.0);
         SmartDashboard.putNumber("RotAlign kAccel", 0.0);
     }    
 
     @Override
     public void initialize() {
         System.out.println("DrivetrainPoseAlign Online");
 
         HotRefreshFBAlignPID();
         HotRefreshLRAlignPID();
         HotRefreshRotAlignPID();

        FBPositionHasReset = false;
        LRPositionHasReset = false;
        RotPositionHasReset = false;

        FBPIDController.reset(0);
        LRPIDController.reset(0);
        rotationPIDController.reset(0);
    
     }
 
     @Override
     public void execute() {
        double FB_Reading = visionManager.deriveFBPose();
        double LR_Reading = visionManager.deriveLRPose();
        double Rot_Reading = visionManager.deriveRotPose();

        if (!FBPositionHasReset && FB_Reading!= 0) {
            FBPIDController.reset(FB_Reading);
            FBPositionHasReset = true;
        } 

        if (!LRPositionHasReset && LR_Reading!= 0) {
            LRPIDController.reset(LR_Reading);
            LRPositionHasReset = true;
        } 

        if (!RotPositionHasReset && Rot_Reading!= 0) {
            rotationPIDController.reset(Rot_Reading);
            RotPositionHasReset = true;
        } 

         //FB Speed Calculation
         double FBSpeed;
         if (visionManager.deriveFBPose() != 0.0) {
             FBSpeed = FBPIDController.calculate(visionManager.deriveFBPose(), -0.55);
         } else {
             FBSpeed = 0.0;
         }
 
         //LR Speed Calculation
         double LRSpeed;
         double goal;
         if (visionManager.deriveLRPose() != 0.0) {
            if(this.targetPosition == ALIGN_STATES.LEFT)
                goal = -0.1775;
            else if(this.targetPosition == ALIGN_STATES.RIGHT)
                goal = 0.1775;
            else
                goal = 0;
             LRSpeed = -LRPIDController.calculate(visionManager.deriveLRPose(), goal);
         } else {
             LRSpeed = 0.0;
         }
 
         // Rot Speed Calculation
         double RotSpeed;
         if (visionManager.deriveRotPose() != 0.0) {
             RotSpeed = rotationPIDController.calculate(visionManager.deriveRotPose(), 0.0);
         } else {
             RotSpeed = 0.0;
         }
  
         LRSpeed = LRPIDController.atGoal() ? 0 : LRSpeed;
         FBSpeed = FBPIDController.atGoal() ? 0 : FBSpeed;


         SwerveRequest.RobotCentric drivetrainRequest = new SwerveRequest.RobotCentric()
                 .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                 .withSteerRequestType(SteerRequestType.MotionMagicExpo);
         drivetrain.setControl(drivetrainRequest
                 .withVelocityX(0.0) //FBSpeed
                 .withVelocityY(LRSpeed) //LRSpeed
                 .withRotationalRate(RotSpeed)); //RotSpeed
 
         SmartDashboard.putNumber("FBSpeed", FBSpeed);
         SmartDashboard.putNumber("LRSpeed", LRSpeed);
         SmartDashboard.putNumber("RotSpeed", RotSpeed);
     }
 
     @Override
     public void end(boolean interrupted) {
         System.out.println("DrivetrainPoseAlign Offline");
     }
 
     @Override
     public boolean isFinished() {
         return false;
     }
 
     public void HotRefreshFBAlignPID() {
         FBPIDController.setP(SmartDashboard.getNumber("FBAlign kP", 0.0));
         FBPIDController.setI(SmartDashboard.getNumber("FBAlign kI", 0.0));
         FBPIDController.setD(SmartDashboard.getNumber("FBAlign kD", 0.0));
         FBPIDController.setConstraints(new Constraints(SmartDashboard.getNumber("FBAlign kVelo", 0.0), SmartDashboard.getNumber("FBAlign kAccel", 0.0)));
         System.out.println("HotRefreshFBAlignPID Complete");
     }
 
     public void HotRefreshLRAlignPID() {
         LRPIDController.setP(SmartDashboard.getNumber("LRAlign kP", 0.0));
         LRPIDController.setI(SmartDashboard.getNumber("LRAlign kI", 0.0));
         LRPIDController.setD(SmartDashboard.getNumber("LRAlign kD", 0.0));
         LRPIDController.setConstraints(new Constraints(SmartDashboard.getNumber("LRAlign kVelo", 0.0), SmartDashboard.getNumber("LRAlign kAccel", 0.0)));
         System.out.println("HotRefreshLRAlignPID Complete");
     }
 
    public void HotRefreshRotAlignPID() {
        rotationPIDController.setP(SmartDashboard.getNumber("RotAlign kP", 0.0));
         rotationPIDController.setI(SmartDashboard.getNumber("RotAlign kI", 0.0));
         rotationPIDController.setD(SmartDashboard.getNumber("RotAlign kD", 0.0));
         rotationPIDController.setConstraints(new Constraints(SmartDashboard.getNumber("RotAlign kVelo", 0.0), SmartDashboard.getNumber("RotAlign kAccel", 0.0)));
         System.out.println("HotRefreshRotAlignPID Complete");
     }
 }