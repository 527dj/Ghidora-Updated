package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Devices;
import frc.robot.Utility.AbsoluteEncoder;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final TalonFX Intake_Wrist_Motor = new TalonFX(Devices.INTAKE_WRIST_MOTOR);
    private final TalonFX Intake_Roller_Motor = new TalonFX(Devices.INTAKE_ROLLER_MOTOR);
    private final TalonFX Intake_Indexer_Master_Motor = new TalonFX(Devices.INDEXER_MOTOR);
    private final AbsoluteEncoder encoder = new AbsoluteEncoder(Devices.INTAKE_WRIST_THROUGH_BORE_PORT);
    private final GenericEntry IntakeGroundDeploy = Shuffleboard.getTab("Intake").add("Ground Deploy", Constants.Intake_Ground_Deploy_Setpoint).getEntry();

    private double setpoint;
    
    public static Intake getInstance() {
        return instance;
    }

    private static Intake instance = new Intake();

    public Intake() {
        System.out.println("====================Intake Subsystem Online====================");
        //HotRegreshIntakeConfig
        // SmartDashboard.putNumber("Intake kG", 0.0);
        // SmartDashboard.putNumber("Intake kP", 0.0);
        // SmartDashboard.putNumber("Intake kI", 0.0);
        // SmartDashboard.putNumber("Intake kD", 0.0);
        // SmartDashboard.putNumber("Intake kVelo", 0.0);
        // SmartDashboard.putNumber("Intake kAccel", 0.0);

        //====================Intake Wrist====================
        var intakeWristMotorConfigs = new TalonFXConfiguration();

        Intake_Wrist_Motor.setPosition(Constants.Absolute_Zero);

        //encoder.setInverted(false);
        //encoder.setOffset(Rotation2d.fromRotations(Constants.Intake_Wrist_Through_Bore_Offset));

        intakeWristMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        //General Configurations
        var generalSlotConfigs = intakeWristMotorConfigs.Slot0;
        generalSlotConfigs.kP = Constants.Intake_Wrist_kP;
        generalSlotConfigs.kI = Constants.Intake_Wrist_kI;
        generalSlotConfigs.kD = Constants.Intake_Wrist_kD;

        //Motion Magic
        var motionMagicConfigs = intakeWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Intake_Wrist_Velocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Intake_Wrist_Acceleration;

        //Current Limits
        var intakeWristLimitConfigs = intakeWristMotorConfigs.CurrentLimits;
        intakeWristLimitConfigs.StatorCurrentLimit = Constants.Intake_Wrist_Current_Limit;
        intakeWristLimitConfigs.StatorCurrentLimitEnable = true;

        //Applies Configs
        Intake_Wrist_Motor.getConfigurator().apply(intakeWristMotorConfigs);

        //====================Intake Rollers====================
        var intakeRollersMotorConfigs = new TalonFXConfiguration();

        //Current Limits
        var intakeRollerLimitConfigs = intakeRollersMotorConfigs.CurrentLimits;
        intakeRollerLimitConfigs.StatorCurrentLimit = Constants.Intake_Roller_Current_Limit;
        intakeRollerLimitConfigs.StatorCurrentLimitEnable = true;

        //Applies Configs
        Intake_Roller_Motor.getConfigurator().apply(intakeRollersMotorConfigs);
        Intake_Indexer_Master_Motor.getConfigurator().apply(intakeRollersMotorConfigs);
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Intake Through Bore Rotations", getIntakeWristThroughBoreWithOffset());
        SmartDashboard.putNumber("Intake Wrist Encoder", getIntakeWristEncoder());
        SmartDashboard.putNumber("Intake Wrist Velocity", getIntakeWristVelocity());
    }
    
    //====================Intake Wrist Methods====================
    public void zeroIntakeWrist() {
        Intake_Wrist_Motor.setPosition(Constants.Absolute_Zero);
    }

    public double getIntakeWristEncoder() {
        return Intake_Wrist_Motor.getPosition().getValueAsDouble();
    }

    public double getIntakeWristVelocity() {
        return Intake_Wrist_Motor.getVelocity().getValueAsDouble();
    }

    public double getRollerCurrent(){
        return Intake_Roller_Motor.getStatorCurrent().getValueAsDouble();
    }

    // public void zeroIntakeWristWithAbsolute() {
    //     Intake_Wrist_Motor.setPosition(getIntakeWristThroughBoreWithOffset());
    // }

    // public double getIntakeWristThroughBoreWithOffset() {
    //     return encoder.getPosition().getRotations() * Constants.Intake_Wrist_Through_Bore_Gear_Ratio; //getPosition OR getRawPosition
    // }

    public void setIntakeWristSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void goToIntakeWristSetpoint(double setpoint) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(setpoint).withEnableFOC(true);
        Intake_Wrist_Motor.setControl(m_request);
        SmartDashboard.putNumber("Wrist Setpoint", setpoint);

    }

    public void setIntakeWristSpeed(double speed) {
        Intake_Wrist_Motor.set(-speed);
    }

    //====================Intake Roller Methods====================
    public void setIntakeRollerMotorSpeed(double speed) {
        Intake_Roller_Motor.set(-1 * speed);
    }

    //====================Indexer Methods====================
    public void setIndexerMotorSpeed(double speed) {
        Intake_Indexer_Master_Motor.set(speed);
    }
    
    // public void HotRegreshIntakeConfig() {
    //     //General Configurations
    //     var generalSlotConfigs = new Slot0Configs();
    //     generalSlotConfigs.kG = SmartDashboard.getNumber("Intake kG", 0.0);
    //     generalSlotConfigs.kP = SmartDashboard.getNumber("Intake kP", 0.0);
    //     generalSlotConfigs.kI = SmartDashboard.getNumber("Intake kI", 0.0);
    //     generalSlotConfigs.kD = SmartDashboard.getNumber("Intake kD", 0.0);

    //     //Motion Magic
    //     var motionMagicConfigs = new MotionMagicConfigs();
    //     motionMagicConfigs.MotionMagicCruiseVelocity = SmartDashboard.getNumber("Intake kVelo", 0.0);
    //     motionMagicConfigs.MotionMagicAcceleration = SmartDashboard.getNumber("Intake kAccel", 0.0);

    //     //Applies Configs
    //     Intake_Wrist_Motor.getConfigurator().apply(generalSlotConfigs);
    //     Intake_Wrist_Motor.getConfigurator().apply(motionMagicConfigs);

    //     System.out.println("HotRegreshIntakeConfig Complete");
    // }
}