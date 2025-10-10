package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class shootL1 extends Command {
    private final EndEffector endEffector;
    private final Elevator elevator;
    public shootL1(EndEffector endEffector, Elevator elevator) {
        this.endEffector = EndEffector.getInstance(); 
        this.elevator = Elevator.getInstance();
        addRequirements(endEffector); 
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("L1 Score Online");
    }

    @Override
    public void execute() {
        endEffector.setEndEffectorRollerMotorSpeed(Constants.End_Effector_Score_L1_Coral_Speed);
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setEndEffectorRollerMotorSpeed(Constants.Absolute_Zero);
        System.out.println("L1 Score Offline");
        elevator.setElevatorSetpoint(Constants.Absolute_Zero);
        elevator.goToElevatorSetpoint();
        endEffector.setEndEffectorWristSetpoint(Constants.Absolute_Zero);
        endEffector.goToEndEffectorWristSetpoint();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}