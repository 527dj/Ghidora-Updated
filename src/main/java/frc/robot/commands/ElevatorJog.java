package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class ElevatorJog extends Command {
    private final Elevator elevator;
    private final DoubleSupplier speed; 

    public ElevatorJog(Elevator elevator, DoubleSupplier speed) {
        this.speed = speed;
        this.elevator = Elevator.getInstance(); 
        addRequirements(elevator); 
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorJog Online");
    }

    @Override
    public void execute() {
        double motorSpeed = speed.getAsDouble();
        elevator.setElevatorMotorSpeed(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorMotorSpeed(Constants.Absolute_Zero);
        System.out.println("ElevatorJog Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}