package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

/**
 * Manages hot-changeable constants through Shuffleboard.
 * Updates Constants class values in real-time during operation.
 */
public class HotChangeManager {
    private static HotChangeManager instance;
    
    // Intake Tab Entries
    private final ShuffleboardTab intakeTab;
    private final GenericEntry intakeGroundDeployEntry;
    
    // EndEffector Tab Entries
    private final ShuffleboardTab endEffectorTab;
    private final GenericEntry endEffectorWristVelocityEntry;
    private final GenericEntry endEffectorWristAccelerationEntry;
    private final GenericEntry endEffectorL2L3SetpointEntry;
    private final GenericEntry endEffectorL4SetpointEntry;
    
    // Elevator Tab Entries
    private final ShuffleboardTab elevatorTab;
    private final GenericEntry elevatorL1SetpointEntry;
    private final GenericEntry elevatorL2SetpointEntry;
    private final GenericEntry elevatorL3SetpointEntry;
    private final GenericEntry elevatorL4SetpointEntry;
    
    private HotChangeManager() {
        // Initialize Intake Tab
        intakeTab = Shuffleboard.getTab("Intake Hot Changes");
        intakeGroundDeployEntry = intakeTab
            .add("Ground Deploy Setpoint", Constants.Intake_Ground_Deploy_Setpoint)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
        
        // Initialize EndEffector Tab
        endEffectorTab = Shuffleboard.getTab("EndEffector Hot Changes");
        endEffectorWristVelocityEntry = endEffectorTab
            .add("Wrist Velocity", Constants.End_Effector_Wrist_Velocity)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
        endEffectorWristAccelerationEntry = endEffectorTab
            .add("Wrist Acceleration", Constants.End_Effector_Wrist_Acceleration)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
        endEffectorL2L3SetpointEntry = endEffectorTab
            .add("L2/L3 Score Setpoint", Constants.End_Effector_Wrist_L2_L3_Score_Setpoint)
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();
        endEffectorL4SetpointEntry = endEffectorTab
            .add("L4 Score Setpoint", Constants.End_Effector_Wrist_L4_Score_Setpoint)
            .withPosition(0, 3)
            .withSize(2, 1)
            .getEntry();
        
        // Initialize Elevator Tab
        elevatorTab = Shuffleboard.getTab("Elevator Hot Changes");
        elevatorL1SetpointEntry = elevatorTab
            .add("L1 Setpoint", Constants.Elevator_L1_Setpoint)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
        elevatorL2SetpointEntry = elevatorTab
            .add("L2 Setpoint", Constants.Elevator_L2_Setpoint)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
        elevatorL3SetpointEntry = elevatorTab
            .add("L3 Setpoint", Constants.Elevator_L3_Setpoint)
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();
        elevatorL4SetpointEntry = elevatorTab
            .add("L4 Setpoint", Constants.Elevator_L4_Setpoint)
            .withPosition(0, 3)
            .withSize(2, 1)
            .getEntry();
        
        System.out.println("====================Hot Change Manager Initialized====================");
    }
    
    public static HotChangeManager getInstance() {
        if (instance == null) {
            instance = new HotChangeManager();
        }
        return instance;
    }
    
    /**
     * Updates all hot-changeable constants from Shuffleboard values.
     * Call this method periodically (e.g., in Robot.robotPeriodic()).
     */
    public void updateConstants() {
        // Update Intake Constants
        Constants.Intake_Ground_Deploy_Setpoint = 
            intakeGroundDeployEntry.getDouble(Constants.Intake_Ground_Deploy_Setpoint);
        
        // Update EndEffector Constants
        Constants.End_Effector_Wrist_Velocity = 
            endEffectorWristVelocityEntry.getDouble(Constants.End_Effector_Wrist_Velocity);
        Constants.End_Effector_Wrist_Acceleration = 
            endEffectorWristAccelerationEntry.getDouble(Constants.End_Effector_Wrist_Acceleration);
        Constants.End_Effector_Wrist_L2_L3_Score_Setpoint = 
            endEffectorL2L3SetpointEntry.getDouble(Constants.End_Effector_Wrist_L2_L3_Score_Setpoint);
        Constants.End_Effector_Wrist_L4_Score_Setpoint = 
            endEffectorL4SetpointEntry.getDouble(Constants.End_Effector_Wrist_L4_Score_Setpoint);
        
        // Update Elevator Constants
        Constants.Elevator_L1_Setpoint = 
            elevatorL1SetpointEntry.getDouble(Constants.Elevator_L1_Setpoint);
        Constants.Elevator_L2_Setpoint = 
            elevatorL2SetpointEntry.getDouble(Constants.Elevator_L2_Setpoint);
        Constants.Elevator_L3_Setpoint = 
            elevatorL3SetpointEntry.getDouble(Constants.Elevator_L3_Setpoint);
        Constants.Elevator_L4_Setpoint = 
            elevatorL4SetpointEntry.getDouble(Constants.Elevator_L4_Setpoint);
    }
    
    public void resetToDefaults() {
        intakeGroundDeployEntry.setDouble(-10.8);
        endEffectorWristVelocityEntry.setDouble(175.0);
        endEffectorWristAccelerationEntry.setDouble(375.0);
        endEffectorL2L3SetpointEntry.setDouble(12.5);
        endEffectorL4SetpointEntry.setDouble(16.0);
        elevatorL1SetpointEntry.setDouble(0.7);
        elevatorL2SetpointEntry.setDouble(4.0);
        elevatorL3SetpointEntry.setDouble(11.1);
        elevatorL4SetpointEntry.setDouble(24.14);
        
        updateConstants();
        System.out.println("Hot Change values reset to defaults");
    }
    
    public void printCurrentValues() {
        System.out.println("========== Current Hot Change Values ==========");
        System.out.println("INTAKE:");
        System.out.println("  Ground Deploy Setpoint: " + Constants.Intake_Ground_Deploy_Setpoint);
        System.out.println("\nEND EFFECTOR:");
        System.out.println("  Wrist Velocity: " + Constants.End_Effector_Wrist_Velocity);
        System.out.println("  Wrist Acceleration: " + Constants.End_Effector_Wrist_Acceleration);
        System.out.println("  L2/L3 Score Setpoint: " + Constants.End_Effector_Wrist_L2_L3_Score_Setpoint);
        System.out.println("  L4 Score Setpoint: " + Constants.End_Effector_Wrist_L4_Score_Setpoint);
        System.out.println("\nELEVATOR:");
        System.out.println("  L1 Setpoint: " + Constants.Elevator_L1_Setpoint);
        System.out.println("  L2 Setpoint: " + Constants.Elevator_L2_Setpoint);
        System.out.println("  L3 Setpoint: " + Constants.Elevator_L3_Setpoint);
        System.out.println("  L4 Setpoint: " + Constants.Elevator_L4_Setpoint);
        System.out.println("===============================================");
    }
}