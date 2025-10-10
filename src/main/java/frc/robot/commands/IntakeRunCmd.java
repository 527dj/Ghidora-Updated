package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

public class IntakeRunCmd extends Command {
    private final double speed;
    private final Intake intake;

    public IntakeRunCmd(Intake intake, double speed) {
        this.speed = speed;
        this.intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("IntakeRun Online");
        intake.setIntakeRollerMotorSpeed(speed);
        intake.setIndexerMotorSpeed(speed);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeRollerMotorSpeed(Constants.Absolute_Zero);
        intake.setIndexerMotorSpeed(Constants.Absolute_Zero);
        System.out.println("IntakeRun Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}