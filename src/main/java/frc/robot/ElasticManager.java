package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import java.util.Map;

/**
 * Manages all Elastic dashboard widgets and telemetry.
 * Provides a centralized interface for displaying robot data.
 */
public class ElasticManager {
    private static ElasticManager instance;
    
    // Field visualization
    private final Field2d field;
    
    // Shuffleboard tabs
    private final ShuffleboardTab mainTab;
    private final ShuffleboardTab visionTab;
    private final ShuffleboardTab tunablesTab;
    private final ShuffleboardTab diagnosticsTab;
    
    // Main Tab Entries
    private GenericEntry matchTimeEntry;
    private GenericEntry batteryVoltageEntry;
    private GenericEntry allianceEntry;
    
    // Vision Tab Entries
    private GenericEntry limelightValidEntry;
    private GenericEntry limelightTxEntry;
    private GenericEntry limelightTyEntry;
    private GenericEntry limelightTaEntry;
    private GenericEntry limelightDistanceEntry;
    private GenericEntry limelightTagCountEntry;
    private GenericEntry limelightPipelineEntry;
    
    // Tunable Entries (Hot Changes)
    private GenericEntry intakeGroundDeployEntry;
    private GenericEntry endEffectorWristVelocityEntry;
    private GenericEntry endEffectorWristAccelerationEntry;
    private GenericEntry endEffectorL2L3SetpointEntry;
    private GenericEntry endEffectorL4SetpointEntry;
    private GenericEntry elevatorL1Entry;
    private GenericEntry elevatorL2Entry;
    private GenericEntry elevatorL3Entry;
    private GenericEntry elevatorL4Entry;
    
    // Diagnostics
    private GenericEntry cpuTempEntry;
    private GenericEntry canUtilizationEntry;
    
    private ElasticManager() {
        System.out.println("====================Elastic Dashboard Initializing====================");
        
        // Initialize field
        field = new Field2d();
        
        // Create tabs with emojis for easy identification
        mainTab = Shuffleboard.getTab("Main");
        visionTab = Shuffleboard.getTab("Vision");
        tunablesTab = Shuffleboard.getTab("Tunables");
        diagnosticsTab = Shuffleboard.getTab("Diagnostics");
        
        setupMainTab();
        setupVisionTab();
        setupTunablesTab();
        setupDiagnosticsTab();
        
        System.out.println("====================Elastic Dashboard Online====================");
    }
    
    public static ElasticManager getInstance() {
        if (instance == null) {
            instance = new ElasticManager();
        }
        return instance;
    }
    
    /**
     * Setup main competition tab with essential match info
     */
    private void setupMainTab() {
        // Field widget - large and prominent
        mainTab.add("Field", field)
            .withPosition(0, 0)
            .withSize(6, 4);
        
        // Match time - large display
        matchTimeEntry = mainTab.add("Match Time", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(6, 0)
            .withSize(2, 1)
            .getEntry();
        
        // Battery voltage with color coding
        batteryVoltageEntry = mainTab.add("Battery", 12.0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withPosition(6, 1)
            .withSize(2, 1)
            .withProperties(Map.of("min", 10.0, "max", 13.0))
            .getEntry();
        
        // Alliance color
        allianceEntry = mainTab.add("Alliance", "Unknown")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(6, 2)
            .withSize(2, 1)
            .getEntry();
        
        // Robot pose components
        mainTab.addDouble("Pose X", () -> field.getRobotPose().getX())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(8, 0)
            .withSize(2, 1);
        
        mainTab.addDouble("Pose Y", () -> field.getRobotPose().getY())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(8, 1)
            .withSize(2, 1);
        
        mainTab.addDouble("Rotation", () -> field.getRobotPose().getRotation().getDegrees())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(8, 2)
            .withSize(2, 1);
    }
    
    /**
     * Setup vision tab with Limelight data
     */
    private void setupVisionTab() {
        // Limelight status
        limelightValidEntry = visionTab.add("Valid Target", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
        
        // Limelight crosshair offsets
        limelightTxEntry = visionTab.add("TX (Horizontal)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -30.0, "max", 30.0))
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
        
        limelightTyEntry = visionTab.add("TY (Vertical)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -30.0, "max", 30.0))
            .withPosition(4, 0)
            .withSize(2, 1)
            .getEntry();
        
        limelightTaEntry = visionTab.add("TA (Area %)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", 0.0, "max", 100.0))
            .withPosition(6, 0)
            .withSize(2, 1)
            .getEntry();
        
        // Distance to target
        limelightDistanceEntry = visionTab.add("Distance (m)", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
        
        // Tag count
        limelightTagCountEntry = visionTab.add("Tag Count", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 1)
            .withSize(2, 1)
            .getEntry();
        
        // Pipeline
        limelightPipelineEntry = visionTab.add("Pipeline", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 1)
            .withSize(2, 1)
            .getEntry();
        
        // Additional vision metrics
        visionTab.addBoolean("MegaTag2 Valid", 
            () -> LimelightHelpers.validPoseEstimate(
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-dihlite")))
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(6, 1)
            .withSize(2, 1);
    }
    
    /**
     * Setup tunables tab for on-the-fly adjustments
     */
    private void setupTunablesTab() {
        // Intake tunables
        tunablesTab.add("INTAKE", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(4, 1);
        
        intakeGroundDeployEntry = tunablesTab.add("Ground Deploy", Constants.Intake_Ground_Deploy_Setpoint)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -15.0, "max", 0.0))
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
        
        // End Effector tunables
        tunablesTab.add("END EFFECTOR", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 2)
            .withSize(4, 1);
        
        endEffectorWristVelocityEntry = tunablesTab.add("Wrist Velocity", Constants.End_Effector_Wrist_Velocity)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 50.0, "max", 200.0))
            .withPosition(0, 3)
            .withSize(2, 1)
            .getEntry();
        
        endEffectorWristAccelerationEntry = tunablesTab.add("Wrist Accel", Constants.End_Effector_Wrist_Acceleration)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 100.0, "max", 400.0))
            .withPosition(2, 3)
            .withSize(2, 1)
            .getEntry();
        
        endEffectorL2L3SetpointEntry = tunablesTab.add("L2/L3 Setpoint", Constants.End_Effector_Wrist_L2_L3_Score_Setpoint)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 30.0))
            .withPosition(0, 4)
            .withSize(2, 1)
            .getEntry();
        
        endEffectorL4SetpointEntry = tunablesTab.add("L4 Setpoint", Constants.End_Effector_Wrist_L4_Score_Setpoint)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 30.0))
            .withPosition(2, 4)
            .withSize(2, 1)
            .getEntry();
        
        // Elevator tunables
        tunablesTab.add("ELEVATOR", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 0)
            .withSize(4, 1);
        
        elevatorL1Entry = tunablesTab.add("L1", Constants.Elevator_L1_Setpoint)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 5.0))
            .withPosition(4, 1)
            .withSize(2, 1)
            .getEntry();
        
        elevatorL2Entry = tunablesTab.add("L2", Constants.Elevator_L2_Setpoint)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 10.0))
            .withPosition(6, 1)
            .withSize(2, 1)
            .getEntry();
        
        elevatorL3Entry = tunablesTab.add("L3", Constants.Elevator_L3_Setpoint)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 15.0))
            .withPosition(4, 2)
            .withSize(2, 1)
            .getEntry();
        
        elevatorL4Entry = tunablesTab.add("L4", Constants.Elevator_L4_Setpoint)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 30.0))
            .withPosition(6, 2)
            .withSize(2, 1)
            .getEntry();
    }
    
    /**
     * Setup diagnostics tab for system health monitoring
     */
    private void setupDiagnosticsTab() {
        // CPU Temperature
        cpuTempEntry = diagnosticsTab.add("CPU Temp (C)", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
        
        // CAN bus utilization
        canUtilizationEntry = diagnosticsTab.add("CAN Usage (%)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", 0.0, "max", 100.0))
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
        
        // Subsystem status indicators
        diagnosticsTab.addBoolean("Drivetrain", () -> true)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 1)
            .withSize(1, 1);
        
        diagnosticsTab.addBoolean("Intake", () -> true)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(1, 1)
            .withSize(1, 1);
        
        diagnosticsTab.addBoolean("EndEffector", () -> true)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 1)
            .withSize(1, 1);
        
        diagnosticsTab.addBoolean("Elevator", () -> true)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 1)
            .withSize(1, 1);
        
        diagnosticsTab.addBoolean("Vision", () -> LimelightHelpers.getTV("limelight-dihlite"))
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 1)
            .withSize(1, 1);
    }
    
    /**
     * Send autonomous chooser to dashboard
     */
    public void sendAutoChooser(SendableChooser<Command> chooser) {
        mainTab.add("Auto Selector", chooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 4)
            .withSize(3, 1);
    }
    
    /**
     * Update robot pose on field
     */
    public void updateRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }
    
    /**
     * Update match time
     */
    public void updateMatchTime(double time) {
        matchTimeEntry.setDouble(time);
    }
    
    /**
     * Update battery voltage
     */
    public void updateBatteryVoltage(double voltage) {
        batteryVoltageEntry.setDouble(voltage);
        
        // Send alert if low
        if (voltage < 11.5) {
            SmartDashboard.putString("Alert", String.format("LOW BATTERY: %.2fV", voltage));
        }
    }
    
    /**
     * Update alliance color
     */
    public void updateAlliance(String alliance) {
        allianceEntry.setString(alliance);
    }
    
    /**
     * Update Limelight vision data
     */
    public void updateLimelightData() {
        String llName = "limelight-dihlite";
        
        // Get latest results
        LimelightResults results = LimelightHelpers.getLatestResults(llName);
        
        if (results != null && results.valid) {
            limelightValidEntry.setBoolean(true);
            
            // Update basic targeting data
            limelightTxEntry.setDouble(LimelightHelpers.getTX(llName));
            limelightTyEntry.setDouble(LimelightHelpers.getTY(llName));
            limelightTaEntry.setDouble(LimelightHelpers.getTA(llName));
            
            // Pipeline
            limelightPipelineEntry.setDouble(LimelightHelpers.getCurrentPipelineIndex(llName));
            
            // If we have fiducial targets, show tag count and distance
            if (results.targets_Fiducials != null && results.targets_Fiducials.length > 0) {
                limelightTagCountEntry.setInteger(results.targets_Fiducials.length);
                
                // Get closest tag distance
                LimelightTarget_Fiducial closestTag = results.targets_Fiducials[0];
                double distance = closestTag.getTargetPose_CameraSpace().getTranslation().getNorm();
                limelightDistanceEntry.setDouble(distance);
            } else {
                limelightTagCountEntry.setInteger(0);
                limelightDistanceEntry.setDouble(0.0);
            }
        } else {
            limelightValidEntry.setBoolean(false);
            limelightTxEntry.setDouble(0.0);
            limelightTyEntry.setDouble(0.0);
            limelightTaEntry.setDouble(0.0);
            limelightDistanceEntry.setDouble(0.0);
            limelightTagCountEntry.setInteger(0);
        }
    }
    
    /**
     * Update all tunable constants from dashboard
     * Call this in Robot.robotPeriodic()
     */
    public void updateTunableConstants() {
        // Intake
        Constants.Intake_Ground_Deploy_Setpoint = 
            intakeGroundDeployEntry.getDouble(Constants.Intake_Ground_Deploy_Setpoint);
        
        // End Effector
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
            elevatorL1Entry.getDouble(Constants.Elevator_L1_Setpoint);
        Constants.Elevator_L2_Setpoint = 
            elevatorL2Entry.getDouble(Constants.Elevator_L2_Setpoint);
        Constants.Elevator_L3_Setpoint = 
            elevatorL3Entry.getDouble(Constants.Elevator_L3_Setpoint);
        Constants.Elevator_L4_Setpoint = 
            elevatorL4Entry.getDouble(Constants.Elevator_L4_Setpoint);
    }
    
    /**
     * Update system diagnostics
     */
    public void updateDiagnostics() {
        // CPU temp (if available - RoboRIO 2 only)
        try {
            // This is a placeholder - actual implementation depends on RoboRIO version
            cpuTempEntry.setDouble(0.0);
        } catch (Exception e) {
            cpuTempEntry.setDouble(-1.0);
        }
        
        // CAN utilization
        canUtilizationEntry.setDouble(RobotController.getCANStatus().percentBusUtilization * 100);
    }
    
    /**
     * Periodic update - call this from Robot.robotPeriodic()
     */
    public void periodic() {
        // Update match time
        updateMatchTime(Timer.getMatchTime());
        
        // Update battery voltage
        updateBatteryVoltage(RobotController.getBatteryVoltage());
        
        // Update alliance
        DriverStation.getAlliance().ifPresent(alliance -> {
            updateAlliance(alliance.name());
        });
        
        // Update vision data
        updateLimelightData();
        
        // Update diagnostics
        updateDiagnostics();
        
        // Update tunable constants
        updateTunableConstants();
    }
    
    /**
     * Get field object for subsystems
     */
    public Field2d getField() {
        return field;
    }
    
    /**
     * Reset all tunables to default values
     */
    public void resetTunablesToDefaults() {
        intakeGroundDeployEntry.setDouble(-10.8);
        endEffectorWristVelocityEntry.setDouble(100.0);
        endEffectorWristAccelerationEntry.setDouble(200.0);
        endEffectorL2L3SetpointEntry.setDouble(12.5);
        endEffectorL4SetpointEntry.setDouble(16.0);
        elevatorL1Entry.setDouble(0.7);
        elevatorL2Entry.setDouble(4.0);
        elevatorL3Entry.setDouble(11.1);
        elevatorL4Entry.setDouble(24.14);
        
        updateTunableConstants();
        System.out.println("All tunable values reset to defaults");
    }
    
    /**
     * Print summary of current values
     */
    public void printValuesSummary() {
        System.out.println("========== Current Tunable Values ==========");
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
        System.out.println("============================================");
    }
}