package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class ZeroEndEffectorWrist extends Command {
    private final EndEffector endEffector;

    public ZeroEndEffectorWrist(EndEffector endEffector) {
        this.endEffector = EndEffector.getInstance(); 
        addRequirements(endEffector); 
    }

    @Override
    public void initialize() {
        System.out.println("ZeroEndEffectorWrist Online");
    }

    @Override
    public void execute() {
        endEffector.zeroEndEffectorWrist();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ZeroEndEffectorWrist Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}