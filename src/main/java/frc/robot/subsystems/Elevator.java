package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Devices;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final TalonFX Elevator_Master_Motor = new TalonFX(Devices.ELEVATOR_MASTER_MOTOR);
    private final TalonFX Elevator_Slave_Motor = new TalonFX(Devices.ELEVATOR_SLAVE_MOTOR);

    private double setpoint;

    public static Elevator getInstance() {
        return instance;
    }

    private static Elevator instance = new Elevator();

    public Elevator() {
        System.out.println("====================Elevator Subsystem Online====================");

        //HotRefreshElevatorConfig
        // SmartDashboard.putNumber("Elevator kG", 0.0);
        // SmartDashboard.putNumber("Elevator kP", 0.0);
        // SmartDashboard.putNumber("Elevator kI", 0.0);
        // SmartDashboard.putNumber("Elevator kD", 0.0);
        // SmartDashboard.putNumber("Elevator kVelo", 0.0);
        // SmartDashboard.putNumber("Elevator kAccel", 0.0);

        //====================Elevator Subsystem====================
        var elevatorMotorConfigs = new TalonFXConfiguration();

        Elevator_Master_Motor.setPosition(Constants.Absolute_Zero);
        Elevator_Slave_Motor.setPosition(Constants.Absolute_Zero);

        //Brake Mode
        elevatorMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //General Configurations
        var generalSlotConfigs = elevatorMotorConfigs.Slot0;
        generalSlotConfigs.kG = Constants.Elevator_kG;
        generalSlotConfigs.kP = Constants.Elevator_kP;
        generalSlotConfigs.kI = Constants.Elevator_kI;
        generalSlotConfigs.kD = Constants.Elevator_kD;

        //Motion Magic
        var motionMagicConfigs = elevatorMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Elevator_Velocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Elevator_Acceleration;

        //Current limits
        var limitConfigs = elevatorMotorConfigs.CurrentLimits;
        limitConfigs.StatorCurrentLimit = Constants.Elevator_Current_Limit;
        limitConfigs.StatorCurrentLimitEnable = true;

        //Applies Configs
        Elevator_Master_Motor.getConfigurator().apply(elevatorMotorConfigs);
        Elevator_Slave_Motor.getConfigurator().apply(elevatorMotorConfigs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Master Encoder", getElevatorMasterEncoder());
        SmartDashboard.putNumber("Elevator Master Velocity", getElevatorMasterVelocity());

        SmartDashboard.putNumber("Elevator Slave Encoder", getElevatorSlaveEncoder());
        SmartDashboard.putNumber("Elevator Slave Velocity", getElevatorSlaveVelocity());
    }

    //====================Elevator Methods====================
    public void setElevatorMotorSpeed(double speed) {
        Elevator_Master_Motor.set(speed);
        Elevator_Slave_Motor.set(speed);
    }

    public double getElevatorMasterEncoder() {
        return Elevator_Master_Motor.getPosition().getValueAsDouble();
    }

    public double getElevatorMasterVelocity() {
        return Elevator_Master_Motor.getVelocity().getValueAsDouble();
    }
 
    public double getElevatorSlaveEncoder() {
        return Elevator_Slave_Motor.getPosition().getValueAsDouble();
    }

    public double getElevatorSlaveVelocity() {
        return Elevator_Slave_Motor.getVelocity().getValueAsDouble();
    }

    public void setElevatorSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void goToElevatorSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(Constants.Absolute_Zero).withEnableFOC(true);
        Elevator_Master_Motor.setControl(m_request.withPosition(-1 * this.setpoint));
        Elevator_Slave_Motor.setControl(m_request.withPosition(-1 * this.setpoint));
    }

    public void zeroElevator() {
        Elevator_Master_Motor.setPosition(Constants.Absolute_Zero);
        Elevator_Slave_Motor.setPosition(Constants.Absolute_Zero);
    }

    // public void HotRefreshElevatorConfig() {
    //     //General Configurations
    //     var generalSlotConfigs = new Slot0Configs();
    //     generalSlotConfigs.kG = SmartDashboard.getNumber("Elevator kG", 0.0);
    //     generalSlotConfigs.kP = SmartDashboard.getNumber("Elevator kP", 0.0);
    //     generalSlotConfigs.kI = SmartDashboard.getNumber("Elevator kI", 0.0);
    //     generalSlotConfigs.kD = SmartDashboard.getNumber("Elevator kD", 0.0);

    //     //Motion Magic
    //     var motionMagicConfigs = new MotionMagicConfigs();
    //     motionMagicConfigs.MotionMagicCruiseVelocity = SmartDashboard.getNumber("Elevator kVelo", 0.0);
    //     motionMagicConfigs.MotionMagicAcceleration = SmartDashboard.getNumber("Elevator kAccel", 0.0);

    //     //Applies Configs
    //     Elevator_Master_Motor.getConfigurator().apply(generalSlotConfigs);
    //     Elevator_Master_Motor.getConfigurator().apply(motionMagicConfigs);

    //     Elevator_Slave_Motor.getConfigurator().apply(generalSlotConfigs);
    //     Elevator_Slave_Motor.getConfigurator().apply(motionMagicConfigs);

    //     System.out.println("HotRefreshElevatorConfig Complete");
    // }
}