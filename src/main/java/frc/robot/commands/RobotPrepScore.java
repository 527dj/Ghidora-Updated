package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class RobotPrepScore extends Command {
    private final EndEffector endEffector;
    private final double endEffectorSetpoint;

    private final Elevator elevator;
    private final double elevatorSetpoint;

    private final Drivetrain drivetrain;
    private final double speedMultiplier;
    private final double turnMultiplier;
    private final XboxController controller;

    public RobotPrepScore(EndEffector endEffector, double endEffectorSetpoint, Elevator elevator, double elevatorSetpoint, Drivetrain drivetrain, double speedMultiplier, double turnMultiplier, XboxController controller) {
        this.endEffector = EndEffector.getInstance();
        this.endEffectorSetpoint = endEffectorSetpoint;

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
        System.out.println("RobotPrepScore Online");
    }

    @Override
    public void execute() {
        endEffector.goToEndEffectorWristSetpoint();
        elevator.goToElevatorSetpoint();
        drivetrain.slowDrivetrain(controller, speedMultiplier, turnMultiplier);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("RobotPrepScore Offline");
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.End_Effector_Wrist_Velocity_Slow;
        motionMagicConfigs.MotionMagicAcceleration = Constants.End_Effector_Wrist_Acceleration_Slow;
        endEffector.changeMotionMagic(motionMagicConfigs);
        endEffector.setEndEffectorWristSetpoint(Constants.Absolute_Zero);
        elevator.setElevatorSetpoint(Constants.Absolute_Zero);
        endEffector.goToEndEffectorWristSetpoint();
        elevator.goToElevatorSetpoint();
        MotionMagicConfigs motionMagicConfigs2 = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.End_Effector_Wrist_Velocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.End_Effector_Wrist_Acceleration;
        endEffector.changeMotionMagic(motionMagicConfigs2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}