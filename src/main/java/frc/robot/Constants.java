package frc.robot;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class Constants {
    //====================General====================
    public static final double Absolute_Zero = 0.0;
    public static final double PID_Setpoint_Tolerance = 0.12;

    //====================Drivetrain====================


    //    ProfiledPIDController FBPIDController = new ProfiledPIDController(2.75, 0, 0.0, new Constraints(4.0, 4.0)); //2.75, 0, 0, 4
    //ProfiledPIDController LRPIDController = new ProfiledPIDController(6.0, 0, 0.1, new Constraints(0.2, 0.2));
    //ProfiledPIDController rotationPIDController = new ProfiledPIDController(0.1, 0, 0, new Constraints(1.0, 1.0)); 
    //tolerance
    //front back setpoint
    //lr setpoint
    //rotation setpoint     

    //Default
    public static final double DrivetrainMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double DrivetrainMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    public static final double Drivetrain_Speed_Multiplier = 1.0;
    public static final double Drivetrain_Turn_Multiplier = 1.0;

    //Elevator Placement
    public static final double Drivetrain_Elevator_Speed_Multiplier = 0.15;
    public static final double Drivetrain_Elevator_Turn_Multiplier = 0.7;
    public static final double Drivetrain_Elevator_Tolerance = 1.0;

    //FBPIDController
    public static final double FB_kP = 2.75;
    public static final double FB_kI = 0.0;
    public static final double FB_kD = 0.0;
    public static final double FB_kVelo = 4.0;
    public static final double FB_kAccel = 4.0;
    public static final double FB_Setpoint = 0.55;

    //LRPIDController
    public static final double LR_kP = 3.45;
    public static final double LR_kI = 0.0;
    public static final double LR_kD = 0.0;
    public static final double LR_kVelo = 4.0;
    public static final double LR_kAccel = 4.0;
    public static final double L_Setpoint = -0.16;
    public static final double R_Setpoint = 0.16;

    //LRPIDController
    public static final double Rot_kP = 0.00775;
    public static final double Rot_kI = 0.0;
    public static final double Rot_kD = 0.0;
    public static final double Rot_kVelo = 1.0;
    public static final double Rot_kAccel = 1.0;
    public static final double Rot_Setpoint = 0.0;

    //====================Intake====================
    //Through Bore Encoder
    public static final double Intake_Wrist_Through_Bore_Gear_Ratio = (50/8) * (62/16);
    public static final double Intake_Wrist_Through_Bore_Offset = 0.0; //0.94312122289

    //Intake Motion Magic
    public static final double Intake_Wrist_kG = 0.0;
    public static final double Intake_Wrist_kP = 2.0; //2.0
    public static final double Intake_Wrist_kI = 0.03; //0.03
    public static final double Intake_Wrist_kD = 0.0;
    public static final double Intake_Wrist_Velocity = 100.0; //75.0
    public static final double Intake_Wrist_Acceleration = 200.0; //150

    //Intake Current Limits
    public static final double Intake_Wrist_Current_Limit = 80.0;
    public static final double Intake_Roller_Current_Limit = 90.0;
    public static final double INTAKE_CURRENT_SPIKE = 60;
    public static final double Indexer_Current_Limit = 110;

    //Intake Speeds
    public static final double Intake_Ground_Run_Speed = 0.75;
    public static final double Outake_Ground_Run_Speed = -0.2;

    //Intake Setpoints
    public static final double Intake_Zero_Setpoint = 0.0;
    public static final double Intake_Stow_Setpoint = -6.7;
    public static final double Intake_Ground_Deploy_Setpoint = -10.2;
    public static final double Intake_Between_Setpoint = -8;
    public static final double IntakeHighStow = -5;

    //====================End Effector====================
    //End Effector Motion Magic
    public static final double End_Effector_Wrist_kP = 2.0;
    public static final double End_Effector_Wrist_kI = 0.0;
    public static final double End_Effector_Wrist_kD = 0.03;
    public static final double End_Effector_Wrist_Velocity = 300.0; //150
    public static final double End_Effector_Wrist_Acceleration   = 600.0; //300
    public static final double End_Effector_Wrist_Velocity_Slow = 150;
    public static final double End_Effector_Wrist_Acceleration_Slow = 300;


    //End Effector Current Limits
    public static final double End_Effector_Wrist_Current_Limit = 80.0;
    public static final double End_Effector_Roller_Current_Limit = 80.0;

    //~~~~~End Effector Speeds~~~~~
    //Coral
    public static final double End_Effector_Ground_Intake_Speed = 0.6;
    public static final double End_Effector_Ground_Outake_Speed = -0.4;
    public static final double End_Effector_Coral_Station_Intake_Speed = 0.0;
    public static final double End_Effector_Score_L1_Coral_Speed = -0.27;
    public static final double End_Effector_Score_L2_L3_L4_Speed = 0.7; //0.35

    //Algae
    public static final double End_Effector_Algae_Intake_Speed = 1.0;
    public static final double End_Effector_Algae_Score_Speed = -1.0;

    //~~~~~End Effector Setpoints~~~~~
    //Coral
    public static final double End_Effector_Wrist_Zero_Setpoint = 0.0;
    public static final double End_Effector_Wrist_Coral_Ground_Setpoint = 1.50;
    public static final double End_Effector_Wrist_L1_Score_Setpoint = 30;
    public static final double End_Effector_Wrist_L2_L3_Score_Setpoint = 12.5;
    public static final double End_Effector_Wrist_L4_Score_Setpoint = 15.5; //16
    public static final double End_Effector_Wrist_Coral_Station_Setpoint = 0.0;

    //Algae  
    public static final double End_Effector_Wrist_Algae_Remove_Setpoint = 36.0;
    public static final double End_Effector_Wrist_Algae_Ground_Setpoint = 38.0;
    public static final double End_Effector_Wrist_Algae_Stow_Setpoint = 15.5; //4.0
    public static final double End_Effector_Wrist_Processor_Score_Setpoint = 31.0;
    public static final double End_Effector_Wrist_Net_Score_Setpoint = 21.0;

    public static final double End_Effector_Wrist_Climb_Start_Setpoint = 0.0;
    public static final double End_Effector_Wrist_Climb_End_Setpoint = 30.0;

    //====================Elevator====================
    //Elevator Motion Magic
    public static final double Elevator_kG = -0.3; //-0.3
    public static final double Elevator_kP = 1.9; //2.25
    public static final double Elevator_kI = 0.0; //0.0
    public static final double Elevator_kD = 0.03; //0.03
    public static final double Elevator_Velocity = 150.0; //100
    public static final double Elevator_Acceleration = 300.0; //200

    //Elevator Curent Limit
    public static final double Elevator_Current_Limit = 80.0;

    //~~~~~Elevator Setpoints~~~~~
    //Coral
    public static final double Elevator_Zero_Setpoint = 0.0;
    public static final double Elevator_Ground_Coral_Setpoint = 0.5;
    public static final double Elevator_Coral_Station_Setpoint = 5.6;
    public static final double Elevator_L1_Setpoint = 1.3;
    public static final double Elevator_L2_Setpoint = 4;
    public static final double Elevator_L3_Setpoint = 11.1;
    public static final double Elevator_L4_Setpoint = 24.14;

    //Algae
    public static final double Elevator_Bottom_Algae_Setpoint = 13.0;
    public static final double Elevator_Top_Algae_Setpoint = 19.5;
    public static final double Elevator_Ground_Algae_Setpoint = 0.0;
    public static final double Elevator_Processor_Score_Setpoint = 0.0;
    public static final double Elevator_Net_Score_Setpoint = 24.14;

    public static final double Elevator_Climb_Setpoint = 0.0;

    //====================Climb====================
    //Climb Current Limit
    public static final double Climb_Roller_Current_Limit = 80.0;
    public static final double Climb_Wrist_Current_Limit = 120.0;

    //Climb Speeds
    public static final double Climb_Up_Speed = 1.0;
    public static final double Climb_Down_Speed = -1.0;
}