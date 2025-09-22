package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class IntakeWristJog extends Command {
    private final Intake intake;
    private final DoubleSupplier speed; 

    public IntakeWristJog(Intake intake, DoubleSupplier speed) {
        this.speed = speed;
        this.intake = Intake.getInstance(); 
        addRequirements(intake); 
    }

    @Override
    public void initialize() {
        System.out.println("IntakeWristJog Online");
    }

    @Override
    public void execute() {
        double motorSpeed = speed.getAsDouble();
        intake.setIntakeWristSpeed(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeWristSpeed(Constants.Absolute_Zero);
        System.out.println("IntakeWristJog Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}