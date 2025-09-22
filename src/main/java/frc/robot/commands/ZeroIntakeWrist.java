package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ZeroIntakeWrist extends Command {
    private final Intake intake;

    public ZeroIntakeWrist(Intake intake) {
        this.intake = Intake.getInstance(); 
        addRequirements(intake); 
    }

    @Override
    public void initialize() {
        System.out.println("ZeroIntakeWrist Online");
    }

    @Override
    public void execute() {
        intake.zeroIntakeWrist();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ZeroIntakeWrist Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}