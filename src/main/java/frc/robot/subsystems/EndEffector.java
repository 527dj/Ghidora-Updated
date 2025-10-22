package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Devices;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

    //Motors
    private final TalonFX End_Effector_Wrist_Motor = new TalonFX(Devices.END_EFFECTOR_WRIST_MOTOR);
    private final TalonFX End_Effector_Top_Roller_Motor = new TalonFX(Devices.END_EFFECTOR_ROLLER_MOTOR);
    private final DigitalInput End_Effector_Front_Photoelectric = new DigitalInput(Devices.END_EFFECTOR_PHOTOELECTRIC_FRONT_PORT);
    private final DigitalInput End_Effector_Back_Photoelectric = new DigitalInput(Devices.END_EFFECTOR_PHOTOELECTRIC_BACK_PORT);

    private double wristVelocity = Constants.End_Effector_Wrist_Velocity;
    private double wristAcceleration = Constants.End_Effector_Wrist_Acceleration;

    private double setpoint;

    private static final EndEffector instance = new EndEffector();
    public static EndEffector getInstance() { return instance; }

    public EndEffector() {
        System.out.println("====================EndEffector Subsystem Online====================");

        var endEffectorWristMotorConfigs = new TalonFXConfiguration();
        End_Effector_Wrist_Motor.setPosition(Constants.Absolute_Zero);

        // Brake Mode
        endEffectorWristMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // PID and Motion Magic Config
        var generalSlotConfigs = endEffectorWristMotorConfigs.Slot0;
        generalSlotConfigs.kP = Constants.End_Effector_Wrist_kP;
        generalSlotConfigs.kI = Constants.End_Effector_Wrist_kI;
        generalSlotConfigs.kD = Constants.End_Effector_Wrist_kD;

        var motionMagicConfigs = endEffectorWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = wristVelocity;
        motionMagicConfigs.MotionMagicAcceleration = wristAcceleration;

        // Current Limits
        var endEffectorWristLimitConfigs = endEffectorWristMotorConfigs.CurrentLimits;
        endEffectorWristLimitConfigs.StatorCurrentLimit = Constants.End_Effector_Wrist_Current_Limit;
        endEffectorWristLimitConfigs.StatorCurrentLimitEnable = true;

        End_Effector_Wrist_Motor.getConfigurator().apply(endEffectorWristMotorConfigs);

        // Roller motor setup
        var rollerConfigs = new TalonFXConfiguration();
        rollerConfigs.CurrentLimits.StatorCurrentLimit = Constants.End_Effector_Roller_Current_Limit;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        End_Effector_Top_Roller_Motor.getConfigurator().apply(rollerConfigs);
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

    public void changeMotionMagic(MotionMagicConfigs configs) {
         End_Effector_Wrist_Motor.getConfigurator().apply(configs); 
    }

    public void applyMotionMagicFromShuffleboard() {
        MotionMagicConfigs configs = new MotionMagicConfigs();
        configs.MotionMagicCruiseVelocity = wristVelocity;
        configs.MotionMagicAcceleration = wristAcceleration;
        End_Effector_Wrist_Motor.getConfigurator().apply(configs);
        System.out.println("[EndEffector] MotionMagic updated from Shuffleboard.");
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
}
