package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class EndEffectorWristJog extends Command {
    private final EndEffector endEffector;
    private final DoubleSupplier speed; 

    public EndEffectorWristJog(EndEffector endEffector, DoubleSupplier speed) {
        this.speed = speed;
        this.endEffector = EndEffector.getInstance(); 
        addRequirements(endEffector); 
    }

    @Override
    public void initialize() {
        System.out.println("EndEffectorWristJog Online");
    }

    @Override
    public void execute() {
        double motorSpeed = speed.getAsDouble();
        endEffector.setEndEffectorWristSpeed(-1 * motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setEndEffectorWristSpeed(Constants.Absolute_Zero);
        System.out.println("EndEffectorWristJog Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}