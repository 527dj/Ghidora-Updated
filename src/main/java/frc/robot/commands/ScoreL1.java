package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.Constants;

public class ScoreL1 extends Command {
    private final EndEffector endEffector;
    private final double setpoint; 

    public ScoreL1(EndEffector endEffector, double setpoint) {
        this.setpoint = setpoint;
        this.endEffector = EndEffector.getInstance(); 
        addRequirements(endEffector); 
    }

    @Override
    public void initialize() {
        System.out.println("L1 Wrist Online");
    }

    @Override
    public void execute() {
        endEffector.setEndEffectorWristSetpoint(setpoint);
        endEffector.goToEndEffectorWristSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setEndEffectorWristSpeed(Constants.Absolute_Zero);
        System.out.println("L1 Wrist Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}