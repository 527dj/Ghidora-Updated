package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class ScoreL1 extends Command {
    private final EndEffector endEffector;
    private final double setpoint;
    private final Elevator elevator;

    public ScoreL1(EndEffector endEffector, double setpoint, Elevator elevator) {
        this.setpoint = setpoint;
        this.endEffector = EndEffector.getInstance(); 
        this.elevator = Elevator.getInstance();
        addRequirements(endEffector);
        addRequirements(elevator); 
    }

    @Override
    public void initialize() {
        System.out.println("L1 Wrist Online");
    }

    @Override
    public void execute() {
        endEffector.setEndEffectorWristSetpoint(setpoint);
        endEffector.goToEndEffectorWristSetpoint();
        elevator.setElevatorSetpoint(Constants.Elevator_L1_Setpoint);
        elevator.goToElevatorSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("L1 Wrist Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}