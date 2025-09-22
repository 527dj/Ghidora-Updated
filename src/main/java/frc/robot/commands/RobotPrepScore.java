package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.Constants;

public class RobotPrepScore extends Command {
    private final EndEffector endEffector;
    private final double endEffectorSetpoint;

    private final Elevator elevator;
    private final double elevatorSetpoint;

    private final Drivetrain drivetrain;
    private final double speedMultiplier;
    private final double turnMultiplier;
    private final XboxController controller;

    public RobotPrepScore(EndEffector endEffector, double endEffectorSetpoint, Elevator elevator, double elevatorSetpoint, Drivetrain drivetrain, double speedMultiplier, double turnMultiplier, XboxController controller) {
        this.endEffector = endEffector;
        this.endEffectorSetpoint = endEffectorSetpoint;

        this.elevator = elevator;
        this.elevatorSetpoint = elevatorSetpoint;

        this.drivetrain = drivetrain;
        this.speedMultiplier = speedMultiplier;
        this.turnMultiplier = turnMultiplier;
        this.controller = controller;

        addRequirements(endEffector, elevator, drivetrain);
    }

    @Override
    public void initialize() {
        endEffector.setEndEffectorWristSetpoint(endEffectorSetpoint);
        elevator.setElevatorSetpoint(elevatorSetpoint);
        System.out.println("RobotPrepScore Online");
    }

    @Override
    public void execute() {
        endEffector.goToEndEffectorWristSetpoint();
        elevator.goToElevatorSetpoint();
        drivetrain.slowDrivetrain(controller, speedMultiplier, turnMultiplier);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("RobotPrepScore Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}