package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

public class SuperIntake extends Command {
    private final Intake intake;
    private final double intakeSetpoint;
    private final double intakeSpeed;

    public SuperIntake(Intake intake, double intakeSetpoint, double intakeSpeed) {
        this.intake = Intake.getInstance();
        this.intakeSetpoint = intakeSetpoint;
        this.intakeSpeed = intakeSpeed;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeWristSetpoint(intakeSetpoint);
        System.out.println("SuperIntake Online");
    }

    @Override
    public void execute() {
        //Roller Control
        intake.setIntakeRollerMotorSpeed(intakeSpeed);
        intake.setIndexerMotorSpeed(intakeSpeed);

        //PID Control
        //UNDO 24 LATER
        intake.goToIntakeWristSetpoint(24);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeRollerMotorSpeed(Constants.Absolute_Zero);
        intake.setIndexerMotorSpeed(Constants.Absolute_Zero);
        System.out.println("SuperIntake Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}