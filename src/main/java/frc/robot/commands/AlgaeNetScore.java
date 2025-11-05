package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EndEffector;
import frc.robot.Constants;

public class AlgaeNetScore extends Command {
    private final double speed;
    private final EndEffector endEffector;

    private final Drivetrain drivetrain;
    private final double speedMultiplier;
    private final double turnMultiplier;
    private final XboxController controller;

    public AlgaeNetScore(EndEffector endEffector, double speed, Drivetrain drivetrain, double speedMultiplier, double turnMultiplier, XboxController controller) {
        this.speed = speed;
        this.endEffector = EndEffector.getInstance();

        this.drivetrain = drivetrain;
        this.speedMultiplier = speedMultiplier;
        this.turnMultiplier = turnMultiplier;
        this.controller = controller;

        addRequirements(endEffector);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("AlgaeNetScore Online");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;
        drivetrain.slowDrivetrain(controller, speedMultiplier, turnMultiplier);
        endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setEndEffectorRollerMotorSpeed(Constants.Absolute_Zero);
        System.out.println("AlgaeNetScore Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}