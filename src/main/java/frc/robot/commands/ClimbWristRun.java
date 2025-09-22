package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ClimbWristRun extends Command {
    private final double speed;
    private final Climb climb;

    public ClimbWristRun(Climb climb, double speed) {
        this.speed = speed;
        this.climb = Climb.getInstance();
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        System.out.println("ClimbWristRun Online");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;

        climb.setClimbWristMotorSpeed(motorSpeed);
        climb.setClimbRollerSpeed(-Math.abs(motorSpeed));
    }
    

    @Override
    public void end(boolean interrupted) {
        climb.setClimbWristMotorSpeed(Constants.Absolute_Zero);
        climb.setClimbRollerSpeed(Constants.Absolute_Zero);
        System.out.println("ClimbWristRun Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}