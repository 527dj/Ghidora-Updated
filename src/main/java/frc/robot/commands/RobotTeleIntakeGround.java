package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.Devices;

public class RobotTeleIntakeGround extends Command {
    private final double speed;
    private final EndEffector endEffector;
    private final double setpoint;

    private final Intake intake;
    private double intakeSetpoint;
    private final double intakeSpeed;

    private final Elevator elevator;
    private final double elevatorSetpoint;

    private final XboxController controller;
    private final XboxController opController;

    // Debounced trigger for detecting a game piece
    private final Trigger gamePieceDetected;
    private final Trigger inIntake;

    public RobotTeleIntakeGround(EndEffector endEffector, double speed, double setpoint,
            Intake intake, double intakeSetpoint, double intakeSpeed,
            Elevator elevator, double elevatorSetpoint,
            XboxController controller, XboxController opController) {

        this.speed = speed;
        this.endEffector = EndEffector.getInstance();
        this.setpoint = setpoint;

        this.intake = Intake.getInstance();
        this.intakeSetpoint = intakeSetpoint;
        this.intakeSpeed = intakeSpeed;

        this.elevator = Elevator.getInstance();
        this.elevatorSetpoint = elevatorSetpoint;

        this.controller = controller;
        this.opController = opController;

        BooleanSupplier isIntakeIn = () -> this.intake.getRollerCurrent()>60;
        BooleanSupplier hasGamePiece = () -> this.endEffector.getEndEffectorFrontPhotoElectricReading();
        this.gamePieceDetected = new Trigger(hasGamePiece).debounce(0.05);
        this.inIntake = new Trigger(isIntakeIn).debounce(0.001);
        addRequirements(this.elevator, this.endEffector, this.intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeWristSetpoint(intakeSetpoint);
        endEffector.setEndEffectorWristSetpoint(setpoint);
        elevator.setElevatorSetpoint(elevatorSetpoint);
        System.out.println("RobotTeleIntakeGround Online");
    }

    @Override
    public void execute() {
        // Roller Control
        intake.setIntakeRollerMotorSpeed(intakeSpeed);
        intake.setIndexerMotorSpeed(-intakeSpeed);

        double motorSpeed = speed;

        intakeSetpoint = inIntake.getAsBoolean() ? Constants.Intake_Between_Setpoint : Constants.Intake_Ground_Deploy_Setpoint;

        // Debounced photoelectric logic
        if (gamePieceDetected.getAsBoolean()) {
            endEffector.setEndEffectorRollerMotorSpeed(Constants.Absolute_Zero);

            controller.setRumble(XboxController.RumbleType.kLeftRumble, Devices.CONTROLLER_RUMBLE);
            controller.setRumble(XboxController.RumbleType.kRightRumble, Devices.CONTROLLER_RUMBLE);
            opController.setRumble(XboxController.RumbleType.kLeftRumble, Devices.CONTROLLER_RUMBLE);
            opController.setRumble(XboxController.RumbleType.kRightRumble, Devices.CONTROLLER_RUMBLE);
        } else {
            endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);

            controller.setRumble(XboxController.RumbleType.kLeftRumble, Constants.Absolute_Zero);
            controller.setRumble(XboxController.RumbleType.kRightRumble, Constants.Absolute_Zero);
            opController.setRumble(XboxController.RumbleType.kLeftRumble, Constants.Absolute_Zero);
            opController.setRumble(XboxController.RumbleType.kRightRumble, Constants.Absolute_Zero);
        }

        SmartDashboard.putNumber("Current Intake Wrist Position: ", intake.getIntakeWristEncoder());

        // PID Control
        intake.goToIntakeWristSetpoint(intakeSetpoint);
        endEffector.goToEndEffectorWristSetpoint();
        elevator.goToElevatorSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeRollerMotorSpeed(Constants.Absolute_Zero);
        intake.setIndexerMotorSpeed(Constants.Absolute_Zero);
        endEffector.setEndEffectorRollerMotorSpeed(Constants.Absolute_Zero);
        intake.goToIntakeWristSetpoint(Constants.Intake_Stow_Setpoint);

        controller.setRumble(XboxController.RumbleType.kLeftRumble, Constants.Absolute_Zero);
        controller.setRumble(XboxController.RumbleType.kRightRumble, Constants.Absolute_Zero);
        opController.setRumble(XboxController.RumbleType.kRightRumble, Constants.Absolute_Zero);
        opController.setRumble(XboxController.RumbleType.kLeftRumble, Constants.Absolute_Zero);

        System.out.println("RobotTeleIntakeGround Offline");
    }

    @Override
    public boolean isFinished() {
        return speed == Constants.Absolute_Zero;
    }
}
