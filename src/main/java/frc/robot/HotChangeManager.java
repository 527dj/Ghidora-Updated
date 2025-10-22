package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

/**
 * Centralized manager for all hot-changeable constants.
 * Consolidates all HOT CHANGE ENABLED variables into dedicated Shuffleboard tabs.
 * 
 * Benefits:
 * - Single source of truth for tunable values
 * - Organized interface with fire emoji tabs for easy identification
 * - Real-time updates during matches without code redeployment
 * - Clean separation from diagnostic/telemetry data
 */
public class HotChangeManager {
    private static HotChangeManager instance;
    
    // Shuffleboard Tabs
    private final ShuffleboardTab intakeHotTab;
    private final ShuffleboardTab endEffectorHotTab;
    private final ShuffleboardTab elevatorHotTab;
    
    // Intake Entries
    private final GenericEntry intakeGroundDeployEntry;
    
    // EndEffector Entries
    private final GenericEntry endEffectorWristVelocityEntry;
    private final GenericEntry endEffectorWristAccelerationEntry;
    private final GenericEntry endEffectorL2L3SetpointEntry;
    private final GenericEntry endEffectorL4SetpointEntry;
    
    // Elevator Entries
    private final GenericEntry elevatorL1SetpointEntry;
    private final GenericEntry elevatorL2SetpointEntry;
    private final GenericEntry elevatorL3SetpointEntry;
    private final GenericEntry elevatorL4SetpointEntry;
    
    private HotChangeManager() {
        // ==================== 🔥 Intake Hot Changes ====================
        intakeHotTab = Shuffleboard.getTab("🔥 Intake Hot");
        
        intakeGroundDeployEntry = intakeHotTab
            .add("Ground Deploy Setpoint", Constants.Intake_Ground_Deploy_Setpoint)
            .withPosition(0, 0)
            .withSize(3, 2)
            .getEntry();
        
        // ==================== 🔥 EndEffector Hot Changes ====================
        endEffectorHotTab = Shuffleboard.getTab("🔥 EndEffector Hot");
        
        // Motion Magic Parameters (Top Row)
        endEffectorWristVelocityEntry = endEffectorHotTab
            .add("Wrist Velocity", Constants.End_Effector_Wrist_Velocity)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
            
        endEffectorWristAccelerationEntry = endEffectorHotTab
            .add("Wrist Acceleration", Constants.End_Effector_Wrist_Acceleration)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
        
        // Scoring Setpoints (Bottom Row)
        endEffectorL2L3SetpointEntry = endEffectorHotTab
            .add("L2/L3 Score Setpoint", Constants.End_Effector_Wrist_L2_L3_Score_Setpoint)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
            
        endEffectorL4SetpointEntry = endEffectorHotTab
            .add("L4 Score Setpoint", Constants.End_Effector_Wrist_L4_Score_Setpoint)
            .withPosition(2, 1)
            .withSize(2, 1)
            .getEntry();
        
        // ==================== 🔥 Elevator Hot Changes ====================
        elevatorHotTab = Shuffleboard.getTab("🔥 Elevator Hot");
        
        // Level Setpoints in 2x2 Grid
        elevatorL1SetpointEntry = elevatorHotTab
            .add("L1 Setpoint", Constants.Elevator_L1_Setpoint)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
            
        elevatorL2SetpointEntry = elevatorHotTab
            .add("L2 Setpoint", Constants.Elevator_L2_Setpoint)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
            
        elevatorL3SetpointEntry = elevatorHotTab
            .add("L3 Setpoint", Constants.Elevator_L3_Setpoint)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
            
        elevatorL4SetpointEntry = elevatorHotTab
            .add("L4 Setpoint", Constants.Elevator_L4_Setpoint)
            .withPosition(2, 1)
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
     * Updates all hot-changeable constants from Shuffleboard.
     * Called every robot loop in Robot.robotPeriodic().
     */
    public void updateConstants() {
        // Intake
        Constants.Intake_Ground_Deploy_Setpoint = 
            intakeGroundDeployEntry.getDouble(Constants.Intake_Ground_Deploy_Setpoint);
        
        // EndEffector
        Constants.End_Effector_Wrist_Velocity = 
            endEffectorWristVelocityEntry.getDouble(Constants.End_Effector_Wrist_Velocity);
        Constants.End_Effector_Wrist_Acceleration = 
            endEffectorWristAccelerationEntry.getDouble(Constants.End_Effector_Wrist_Acceleration);
        Constants.End_Effector_Wrist_L2_L3_Score_Setpoint = 
            endEffectorL2L3SetpointEntry.getDouble(Constants.End_Effector_Wrist_L2_L3_Score_Setpoint);
        Constants.End_Effector_Wrist_L4_Score_Setpoint = 
            endEffectorL4SetpointEntry.getDouble(Constants.End_Effector_Wrist_L4_Score_Setpoint);
        
        // Elevator
        Constants.Elevator_L1_Setpoint = 
            elevatorL1SetpointEntry.getDouble(Constants.Elevator_L1_Setpoint);
        Constants.Elevator_L2_Setpoint = 
            elevatorL2SetpointEntry.getDouble(Constants.Elevator_L2_Setpoint);
        Constants.Elevator_L3_Setpoint = 
            elevatorL3SetpointEntry.getDouble(Constants.Elevator_L3_Setpoint);
        Constants.Elevator_L4_Setpoint = 
            elevatorL4SetpointEntry.getDouble(Constants.Elevator_L4_Setpoint);
    }
    
    /**
     * Resets all values to defaults from Constants.java.
     */
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
        System.out.println("✓ All hot change values reset to defaults");
    }
    
    /**
     * Prints all current values to console for verification.
     */
    public void printCurrentValues() {
        System.out.println("========== Hot Change Values ==========");
        System.out.println("INTAKE:");
        System.out.println("  Ground Deploy: " + Constants.Intake_Ground_Deploy_Setpoint);
        System.out.println("\nEND EFFECTOR:");
        System.out.println("  Velocity: " + Constants.End_Effector_Wrist_Velocity);
        System.out.println("  Acceleration: " + Constants.End_Effector_Wrist_Acceleration);
        System.out.println("  L2/L3 Setpoint: " + Constants.End_Effector_Wrist_L2_L3_Score_Setpoint);
        System.out.println("  L4 Setpoint: " + Constants.End_Effector_Wrist_L4_Score_Setpoint);
        System.out.println("\nELEVATOR:");
        System.out.println("  L1: " + Constants.Elevator_L1_Setpoint);
        System.out.println("  L2: " + Constants.Elevator_L2_Setpoint);
        System.out.println("  L3: " + Constants.Elevator_L3_Setpoint);
        System.out.println("  L4: " + Constants.Elevator_L4_Setpoint);
        System.out.println("======================================");
    }
    
    /**
     * Returns a compact summary string for logging.
     */
    public String getValuesSummary() {
        return String.format(
            "Intake:%.2f | EE(V:%.0f A:%.0f L23:%.1f L4:%.1f) | Elev(%.1f/%.1f/%.1f/%.2f)",
            Constants.Intake_Ground_Deploy_Setpoint,
            Constants.End_Effector_Wrist_Velocity,
            Constants.End_Effector_Wrist_Acceleration,
            Constants.End_Effector_Wrist_L2_L3_Score_Setpoint,
            Constants.End_Effector_Wrist_L4_Score_Setpoint,
            Constants.Elevator_L1_Setpoint,
            Constants.Elevator_L2_Setpoint,
            Constants.Elevator_L3_Setpoint,
            Constants.Elevator_L4_Setpoint
        );
    }
}