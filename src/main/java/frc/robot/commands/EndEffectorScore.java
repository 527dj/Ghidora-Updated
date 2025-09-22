package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.Constants;

public class EndEffectorScore extends Command {
    private final double speed;
    private final EndEffector endEffector;

    public EndEffectorScore(EndEffector endEffector, double speed) {
        this.speed = speed;
        this.endEffector = EndEffector.getInstance();
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        System.out.println("EndEffectorScore Online");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;
        endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setEndEffectorRollerMotorSpeed(Constants.Absolute_Zero);
        System.out.println("EndEffectorScore Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}