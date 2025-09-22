package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
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