package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Devices;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private final TalonFX Climb_Wrist_Motor = new TalonFX(Devices.CLIMB_WRIST_MOTOR);
    private final TalonFX Climb_Roller_Motor = new TalonFX(Devices.CLIMB_ROLLER_MOTOR);

    public static Climb getInstance() {
        return instance;
    }

    private static Climb instance = new Climb();

    public Climb() {
        System.out.println("====================Climb Subsystem Online====================");

        //====================Climb Subsystem====================
        var climbRollerMotorConfigs = new TalonFXConfiguration();

        climbRollerMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //Current Limits
        var climbRollerLimitConfigs = climbRollerMotorConfigs.CurrentLimits;
        climbRollerLimitConfigs.StatorCurrentLimit = Constants.Climb_Roller_Current_Limit;
        climbRollerLimitConfigs.StatorCurrentLimitEnable = true; 

        //Applies Configs
        Climb_Roller_Motor.getConfigurator().apply(climbRollerMotorConfigs);

        //========================================
        var climbWristMotorConfigs = new TalonFXConfiguration();

        climbWristMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //Current Limits
        var climbWristLimitConfigs = climbWristMotorConfigs.CurrentLimits;
        climbWristLimitConfigs.StatorCurrentLimit = Constants.Climb_Wrist_Current_Limit;
        climbWristLimitConfigs.StatorCurrentLimitEnable = true; 

        //Applies Configs
        Climb_Wrist_Motor.getConfigurator().apply(climbWristMotorConfigs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Wrist Motor Stator Current", getClimbWristStatorCurrent());   
    }

    //====================Climb Methods====================
    public void setClimbWristMotorSpeed(double speed) {
        Climb_Wrist_Motor.set(speed);
    }

    public void setClimbRollerSpeed(double speed) {
        Climb_Roller_Motor.set(speed);
    }

    public double getClimbWristStatorCurrent() {
        return Climb_Wrist_Motor.getStatorCurrent().getValueAsDouble();
    }
}