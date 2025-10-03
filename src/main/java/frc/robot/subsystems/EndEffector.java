package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Devices;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
    private final TalonFX End_Effector_Wrist_Motor = new TalonFX(Devices.END_EFFECTOR_WRIST_MOTOR);
    private final TalonFX End_Effector_Top_Roller_Motor = new TalonFX(Devices.END_EFFECTOR_ROLLER_MOTOR);
    private final DigitalInput End_Effector_Front_Photoelectric = new DigitalInput(Devices.END_EFFECTOR_PHOTOELECTRIC_FRONT_PORT);
    private final DigitalInput End_Effector_Back_Photoelectric = new DigitalInput(Devices.END_EFFECTOR_PHOTOELECTRIC_BACK_PORT);

    private double setpoint;

    public static EndEffector getInstance() {
        return instance;
    }

    private static EndEffector instance = new EndEffector();

    public EndEffector() {
        System.out.println("====================EndEffector Subsystem Online====================");
        //HotRefreshEndEffectorConfig
        // SmartDashboard.putNumber("End Effector kG", 0.0);
        // SmartDashboard.putNumber("End Effector kP", 0.0);
        // SmartDashboard.putNumber("End Effector kI", 0.0);
        // SmartDashboard.putNumber("End Effector kD", 0.0);
        // SmartDashboard.putNumber("End Effector kVelo", 0.0);
        // SmartDashboard.putNumber("End Effector kAccel", 0.0);

        //====================End Effector Wrist====================
        var endEffectorWristMotorConfigs = new TalonFXConfiguration();

        End_Effector_Wrist_Motor.setPosition(Constants.Absolute_Zero);

        //Brake Mode
        endEffectorWristMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //General Configurations
        var generalSlotConfigs = endEffectorWristMotorConfigs.Slot0;
        generalSlotConfigs.kP = Constants.End_Effector_Wrist_kP;
        generalSlotConfigs.kI = Constants.End_Effector_Wrist_kI;
        generalSlotConfigs.kD = Constants.End_Effector_Wrist_kD;

        //Motion Magic
        var motionMagicConfigs = endEffectorWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.End_Effector_Wrist_Velocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.End_Effector_Wrist_Acceleration;

        //Current Limits
        var endEffectorWristLimitConfigs = endEffectorWristMotorConfigs.CurrentLimits;
        endEffectorWristLimitConfigs.StatorCurrentLimit = Constants.End_Effector_Wrist_Current_Limit;
        endEffectorWristLimitConfigs.StatorCurrentLimitEnable = true;

        //Applies Configs
        End_Effector_Wrist_Motor.getConfigurator().apply(endEffectorWristMotorConfigs);

        //====================End Effector Rollers====================
        var endEffectorRollerMotorsConfigs = new TalonFXConfiguration();

        //Current Limits
        var endEffectorRollerLimitConfigs = endEffectorRollerMotorsConfigs.CurrentLimits;
        endEffectorRollerLimitConfigs.StatorCurrentLimit = Constants.End_Effector_Roller_Current_Limit;
        endEffectorRollerLimitConfigs.StatorCurrentLimitEnable = true; 

        //Applies Configs
        End_Effector_Top_Roller_Motor.getConfigurator().apply(endEffectorRollerLimitConfigs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("End Effector Wrist Encoder", getEndEffectorWristEncoder());
        SmartDashboard.putNumber("End Effector Wrist Velocity", getEndEffectorWristVelocity());

        SmartDashboard.putBoolean("End Effector Front Photoelectric Reading", getEndEffectorFrontPhotoElectricReading());
        SmartDashboard.putBoolean("End Effector Back Photoelectric Reading", getEndEffectorBackPhotoElectricReading());
    }
    
    //====================End Effector Wrist Methods====================
    public double getEndEffectorWristEncoder() {
        return End_Effector_Wrist_Motor.getPosition().getValueAsDouble();
    }

    public double getEndEffectorWristVelocity() {
        return End_Effector_Wrist_Motor.getVelocity().getValueAsDouble();
    }


    public void setEndEffectorWristSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void changeMotionMagic(MotionMagicConfigs configs) {
        End_Effector_Wrist_Motor.getConfigurator().apply(configs);
    }

    public void goToEndEffectorWristSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(Constants.Absolute_Zero).withEnableFOC(true);
        End_Effector_Wrist_Motor.setControl(m_request.withPosition(-1 * this.setpoint));
    }

    public void setEndEffectorWristSpeed(double speed) {
        End_Effector_Wrist_Motor.set(-1 * speed);
    }

    public void zeroEndEffectorWrist() {
        End_Effector_Wrist_Motor.setPosition(Constants.Absolute_Zero);
    }

    //====================End Effector Roller Methods====================
    public void setEndEffectorRollerMotorSpeed(double speed) {    
        End_Effector_Top_Roller_Motor.set(speed);
    }

    public boolean getEndEffectorFrontPhotoElectricReading() {
        return !End_Effector_Front_Photoelectric.get();
    }

    public boolean getEndEffectorBackPhotoElectricReading() {
        return !End_Effector_Back_Photoelectric.get();
    }

    // public void HotRefreshEndEffectorConfig() {
    //     //General Configurations
    //     var generalSlotConfigs = new Slot0Configs();
    //     generalSlotConfigs.kG = SmartDashboard.getNumber("End Effector kG", 0.0);
    //     generalSlotConfigs.kP = SmartDashboard.getNumber("End Effector kP", 0.0);
    //     generalSlotConfigs.kI = SmartDashboard.getNumber("End Effector kI", 0.0);
    //     generalSlotConfigs.kD = SmartDashboard.getNumber("End Effector kD", 0.0);

    //     //Motion Magic
    //     var motionMagicConfigs = new MotionMagicConfigs();
    //     motionMagicConfigs.MotionMagicCruiseVelocity = SmartDashboard.getNumber("End Effector kVelo", 0.0);
    //     motionMagicConfigs.MotionMagicAcceleration = SmartDashboard.getNumber("End Effector kAccel", 0.0);

    //     //Applies Configs
    //     End_Effector_Wrist_Motor.getConfigurator().apply(generalSlotConfigs);
    //     End_Effector_Wrist_Motor.getConfigurator().apply(motionMagicConfigs);

    //     System.out.println("HotRefreshEndEffectorConfig Complete");
    //}
}