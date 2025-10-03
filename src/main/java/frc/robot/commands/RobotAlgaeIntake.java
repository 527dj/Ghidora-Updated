package frc.robot.commands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class RobotAlgaeIntake extends Command {
    private final EndEffector endEffector;
    private final double endEffectorSetpoint;
    private final double endEffectorSpeed;

    private final Elevator elevator;
    private final double elevatorSetpoint;

    private final Drivetrain drivetrain;
    private final double speedMultiplier;
    private final double turnMultiplier;
    private final GenericHID controller;

    public RobotAlgaeIntake(EndEffector endEffector, double endEffectorSetpoint, double endEffectorSpeed, Elevator elevator, double elevatorSetpoint, Drivetrain drivetrain, double speedMultiplier, double turnMultiplier, GenericHID controller) {
        this.endEffector = EndEffector.getInstance();
        this.endEffectorSetpoint = endEffectorSetpoint;
        this.endEffectorSpeed = endEffectorSpeed; // Fixed: was using endEffectorSetpoint

        this.elevator = Elevator.getInstance();
        this.elevatorSetpoint = elevatorSetpoint;

        this.drivetrain = drivetrain;
        this.speedMultiplier = speedMultiplier;
        this.turnMultiplier = turnMultiplier;
        this.controller = controller;
        addRequirements(elevator);
        addRequirements(endEffector);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        endEffector.setEndEffectorWristSetpoint(endEffectorSetpoint);
        elevator.setElevatorSetpoint(elevatorSetpoint);
        System.out.println("RobotAlgaeIntake Online");
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.End_Effector_Wrist_Velocity_Slow;
        motionMagicConfigs.MotionMagicAcceleration = Constants.End_Effector_Wrist_Acceleration_Slow;
        endEffector.changeMotionMagic(motionMagicConfigs);
    }

    @Override
    public void execute() {
        endEffector.goToEndEffectorWristSetpoint();
        elevator.goToElevatorSetpoint();
        drivetrain.slowDrivetrain(controller, speedMultiplier, turnMultiplier);
        endEffector.setEndEffectorRollerMotorSpeed(endEffectorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("RobotAlgaeIntake Offline");
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.End_Effector_Wrist_Velocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.End_Effector_Wrist_Acceleration;
        endEffector.changeMotionMagic(motionMagicConfigs);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
