package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

public class IntakeWristSetpoint extends Command {
    private final Intake intake;
    private final double setpoint;

    public IntakeWristSetpoint(Intake intake, double setpoint) {
        this.setpoint = setpoint;
        this.intake = Intake.getInstance(); 
        addRequirements(intake); 
    }

    @Override
    public void initialize() {
        System.out.println("Wrist move Online");
    }

    @Override
    public void execute() {
        intake.setIntakeWristSetpoint(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeWristSpeed(Constants.Absolute_Zero);
        System.out.println("Wrist Move Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}