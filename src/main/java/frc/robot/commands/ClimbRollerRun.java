package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ClimbRollerRun extends Command {
    private final double speed;
    private final Climb climb;

    public ClimbRollerRun(Climb climb, double speed) {
        this.speed = speed;
        this.climb = Climb.getInstance();
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        System.out.println("ClimbRollerRun Online");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;
        climb.setClimbRollerSpeed(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        climb.setClimbRollerSpeed(Constants.Absolute_Zero);
        System.out.println("ClimbRollerRun Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}