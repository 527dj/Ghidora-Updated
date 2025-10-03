package frc.robot.commands;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class RobotHome extends Command {
    private final EndEffector endEffector;
    private final double endEffectorSetpoint;

    private final Elevator elevator;
    private final double elevatorSetpoint;

    public RobotHome(EndEffector endEffector, double endEffectorSetpoint, Elevator elevator, double elevatorSetpoint) {
        this.endEffector = EndEffector.getInstance();
        this.endEffectorSetpoint = endEffectorSetpoint;

        this.elevator = Elevator.getInstance();
        this.elevatorSetpoint = elevatorSetpoint;
        
        addRequirements(elevator);
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        endEffector.setEndEffectorWristSetpoint(endEffectorSetpoint);
        elevator.setElevatorSetpoint(elevatorSetpoint);
        System.out.println("RobotHome Online");
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.End_Effector_Wrist_Velocity_Slow;
        motionMagicConfigs.MotionMagicAcceleration = Constants.End_Effector_Wrist_Acceleration_Slow;
        endEffector.changeMotionMagic(motionMagicConfigs);
    }

    @Override
    public void execute() {
        endEffector.goToEndEffectorWristSetpoint();
        elevator.goToElevatorSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("RobotHome Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}