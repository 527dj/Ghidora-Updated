package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
// import org.goldenlions.elastic.Elastic;
// import org.goldenlions.elastic.ElasticNotification;
// import org.goldenlions.elastic.ElasticNotification.NotificationLevel;
/**
 * Manages the Elastic dashboard configuration and widgets.
 * Replaces Shuffleboard with a modern web-based interface.
 */
public class ElasticManager {
    private static ElasticManager instance;
    
    private final Field2d field;
    
    private ElasticManager() {
        System.out.println("====================Elastic Dashboard Initializing====================");
        
        // Create field visualization
        field = new Field2d();
        
        // Configure Elastic
        setupElasticDashboard();
        
        System.out.println("====================Elastic Dashboard Online====================");
    }
    
    public static ElasticManager getInstance() {
        if (instance == null) {
            instance = new ElasticManager();
        }
        return instance;
    }
    
    /**
     * Set up the Elastic dashboard with all necessary widgets
     */
    private void setupElasticDashboard() {
        // Send the field to NetworkTables for visualization
        Elastic.sendSendable("Field", field);
        
        // Configure camera streams
        setupCameraStreams();
        
        // Setup tunable numbers for hot changes
        setupTunableNumbers();
        
        // Setup subsystem telemetry
        setupSubsystemTelemetry();
        
        // Send notification that Elastic is ready
        ElasticNotification.sendNotification(
            "Elastic Dashboard Ready",
            "All widgets configured successfully",
            NotificationLevel.INFO
        );
    }
    
    /**
     * Configure camera streams for Elastic
     */
    private void setupCameraStreams() {
        // Limelight camera stream
        Elastic.sendCameraStream("Limelight", "http://limelight-dihlite.local:5800/stream.mjpg");
        
        // If you have USB cameras:
        // Elastic.sendCameraStream("Driver Camera", "camera_server:1181");
    }
    
    /**
     * Setup tunable numbers that can be changed on-the-fly
     * These will appear as editable fields in Elastic
     */
    private void setupTunableNumbers() {
        // Intake tunables
        Elastic.sendDouble("Tunable/Intake/Ground Deploy", 
            () -> Constants.Intake_Ground_Deploy_Setpoint,
            (value) -> Constants.Intake_Ground_Deploy_Setpoint = value);
        
        // EndEffector tunables
        Elastic.sendDouble("Tunable/EndEffector/Wrist Velocity", 
            () -> Constants.End_Effector_Wrist_Velocity,
            (value) -> Constants.End_Effector_Wrist_Velocity = value);
            
        Elastic.sendDouble("Tunable/EndEffector/Wrist Acceleration", 
            () -> Constants.End_Effector_Wrist_Acceleration,
            (value) -> Constants.End_Effector_Wrist_Acceleration = value);
            
        Elastic.sendDouble("Tunable/EndEffector/L2L3 Setpoint", 
            () -> Constants.End_Effector_Wrist_L2_L3_Score_Setpoint,
            (value) -> Constants.End_Effector_Wrist_L2_L3_Score_Setpoint = value);
            
        Elastic.sendDouble("Tunable/EndEffector/L4 Setpoint", 
            () -> Constants.End_Effector_Wrist_L4_Score_Setpoint,
            (value) -> Constants.End_Effector_Wrist_L4_Score_Setpoint = value);
        
        // Elevator tunables
        Elastic.sendDouble("Tunable/Elevator/L1", 
            () -> Constants.Elevator_L1_Setpoint,
            (value) -> Constants.Elevator_L1_Setpoint = value);
            
        Elastic.sendDouble("Tunable/Elevator/L2", 
            () -> Constants.Elevator_L2_Setpoint,
            (value) -> Constants.Elevator_L2_Setpoint = value);
            
        Elastic.sendDouble("Tunable/Elevator/L3", 
            () -> Constants.Elevator_L3_Setpoint,
            (value) -> Constants.Elevator_L3_Setpoint = value);
            
        Elastic.sendDouble("Tunable/Elevator/L4", 
            () -> Constants.Elevator_L4_Setpoint,
            (value) -> Constants.Elevator_L4_Setpoint = value);
    }
    
    /**
     * Setup subsystem telemetry displays
     */
    private void setupSubsystemTelemetry() {
        // This will be populated by periodic() calls in each subsystem
        // Just creating the structure here for organization
        
        // Drivetrain telemetry structure
        Elastic.sendDouble("Telemetry/Drivetrain/Pose X", () -> 0.0);
        Elastic.sendDouble("Telemetry/Drivetrain/Pose Y", () -> 0.0);
        Elastic.sendDouble("Telemetry/Drivetrain/Rotation", () -> 0.0);
        
        // Vision telemetry structure
        Elastic.sendBoolean("Telemetry/Vision/Valid", () -> false);
        Elastic.sendDouble("Telemetry/Vision/Tag Count", () -> 0.0);
    }
    
    /**
     * Send autonomous chooser to Elastic
     */
    public void sendAutoChooser(SendableChooser<Command> chooser) {
        Elastic.sendSendable("Auto Chooser", chooser);
    }
    
    /**
     * Update robot pose on the field visualization
     */
    public void updateRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
        
        // Also send individual components for numeric displays
        Elastic.sendDouble("Telemetry/Drivetrain/Pose X", () -> pose.getX());
        Elastic.sendDouble("Telemetry/Drivetrain/Pose Y", () -> pose.getY());
        Elastic.sendDouble("Telemetry/Drivetrain/Rotation", () -> pose.getRotation().getDegrees());
    }
    
    /**
     * Get the field object for other classes to use
     */
    public Field2d getField() {
        return field;
    }
    
    /**
     * Send a notification to the Elastic dashboard
     */
    public static void sendNotification(String title, String message, NotificationLevel level) {
        ElasticNotification.sendNotification(title, message, level);
    }
    
    /**
     * Send match time to dashboard
     */
    public void updateMatchTime(double time) {
        Elastic.sendDouble("Match/Time Remaining", () -> time);
    }
    
    /**
     * Send battery voltage to dashboard
     */
    public void updateBatteryVoltage(double voltage) {
        Elastic.sendDouble("Match/Battery Voltage", () -> voltage);
        
        // Send alert if battery is low
        if (voltage < 11.5) {
            sendNotification("Low Battery", 
                String.format("Battery at %.2fV", voltage), 
                NotificationLevel.WARNING);
        }
    }
}