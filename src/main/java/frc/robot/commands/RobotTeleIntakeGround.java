package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    private final double intakeSetpoint;
    private final double intakeSpeed;

    private final Elevator elevator;
    private final double elevatorSetpoint;

    private final XboxController controller;
    private final XboxController opController;

    public RobotTeleIntakeGround(EndEffector endEffector, double speed, double setpoint, Intake intake,
            double intakeSetpoint, double intakeSpeed, Elevator elevator, double elevatorSetpoint,
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

        addRequirements(elevator);
        addRequirements(endEffector);
        addRequirements(intake);
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
        //intake.goToIntakeWristSetpoint(setpoint);
        double motorSpeed = speed;

        endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);

        // if (endEffector.getEndEffectorFrontPhotoElectricReading() == true && endEffector.getEndEffectorBackPhotoElectricReading() == true) {
        //     endEffector.setEndEffectorRollerMotorSpeed(Constants.Absolute_Zero);
        //     controller.setRumble(XboxController.RumbleType.kLeftRumble, Devices.CONTROLLER_RUMBLE);
        //     controller.setRumble(XboxController.RumbleType.kRightRumble, Devices.CONTROLLER_RUMBLE);
        // } else if (endEffector.getEndEffectorFrontPhotoElectricReading() == false && endEffector.getEndEffectorBackPhotoElectricReading() == true) {
        //     endEffector.setEndEffectorRollerMotorSpeed(0.5 * motorSpeed);
        //     controller.setRumble(XboxController.RumbleType.kLeftRumble, Constants.Absolute_Zero);
        //     controller.setRumble(XboxController.RumbleType.kRightRumble, Constants.Absolute_Zero);
        // } else {
        //     endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);
        //     controller.setRumble(XboxController.RumbleType.kLeftRumble, Constants.Absolute_Zero);
        //     controller.setRumble(XboxController.RumbleType.kRightRumble, Constants.Absolute_Zero);
        // }

        // OLD
        if (endEffector.getEndEffectorFrontPhotoElectricReading() == true) {
            endEffector.setEndEffectorRollerMotorSpeed(Constants.Absolute_Zero);

            controller.setRumble(XboxController.RumbleType.kLeftRumble,
            Devices.CONTROLLER_RUMBLE);
            controller.setRumble(XboxController.RumbleType.kRightRumble,
            Devices.CONTROLLER_RUMBLE);
            opController.setRumble(XboxController.RumbleType.kLeftRumble,
            Devices.CONTROLLER_RUMBLE);
            opController.setRumble(XboxController.RumbleType.kRightRumble,
            Devices.CONTROLLER_RUMBLE);
        } 
        else {
            endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);

            controller.setRumble(XboxController.RumbleType.kLeftRumble,
            Constants.Absolute_Zero);
            controller.setRumble(XboxController.RumbleType.kRightRumble,
            Constants.Absolute_Zero);
            opController.setRumble(XboxController.RumbleType.kLeftRumble,
            Constants.Absolute_Zero);
            opController.setRumble(XboxController.RumbleType.kRightRumble,
            Constants.Absolute_Zero);
        }
        SmartDashboard.putNumber("Current Intake Wrist Position: ",Intake.getInstance().getIntakeWristEncoder());
        //Error Checking
        // if(intakeSetpoint == Constants.Intake_Between_Setpoint)
        //     System.out.println("INTAKE BETWEEN: "+ Intake.getInstance().getRollerCurrent());
        // else if(intakeSetpoint == Constants.Intake_Ground_Deploy_Setpoint)
        //     System.out.println("INTAKE GROUND DEPLOY"+ Intake.getInstance().getRollerCurrent());
        // else if(intakeSetpoint == Constants.Intake_Stow_Setpoint)
        //     System.out.println("INTAKE STOWED"+ Intake.getInstance().getRollerCurrent());
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