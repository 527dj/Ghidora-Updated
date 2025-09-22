package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends Command {
    private final Elevator elevator;

    public ZeroElevator(Elevator elevator) {
        this.elevator = Elevator.getInstance(); 
        addRequirements(elevator); 
    }

    @Override
    public void initialize() {
        System.out.println("ZeroElevator Online");
    }

    @Override
    public void execute() {
        elevator.zeroElevator();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ZeroElevator Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}