package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.Devices;

public class RobotIntakeGround extends Command {
    private final double speed;
    private final EndEffector endEffector;
    private final double setpoint;

    private final Intake intake;
    private final double intakeSetpoint;
    private final double intakeSpeed;

    private final Elevator elevator;
    private final double elevatorSetpoint;

    public RobotIntakeGround(EndEffector endEffector, double speed, double setpoint, Intake intake, double intakeSetpoint, double intakeSpeed, Elevator elevator, double elevatorSetpoint) {
        this.speed = speed;
        this.endEffector = EndEffector.getInstance();
        this.setpoint = setpoint;

        this.intake = Intake.getInstance();
        this.intakeSetpoint = intakeSetpoint;
        this.intakeSpeed = intakeSpeed;

        this.elevator = Elevator.getInstance();
        this.elevatorSetpoint = elevatorSetpoint;

        addRequirements(intake);
        addRequirements(endEffector);
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        intake.setIntakeWristSetpoint(intakeSetpoint);
        endEffector.setEndEffectorWristSetpoint(setpoint);
        elevator.setElevatorSetpoint(elevatorSetpoint);
        System.out.println("RobotIntakeGround Online");
    }

    @Override
    public void execute() {
        //Roller Control
        intake.setIntakeRollerMotorSpeed(intakeSpeed);
        intake.setIndexerMotorSpeed(intakeSpeed);

        double motorSpeed = speed;

        endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);

        // if (endEffector.getEndEffectorFrontPhotoElectricReading() == true && endEffector.getEndEffectorBackPhotoElectricReading() == true) {
        //     endEffector.setEndEffectorRollerMotorSpeed(Constants.Absolute_Zero);
        // } else if (endEffector.getEndEffectorFrontPhotoElectricReading() == false && endEffector.getEndEffectorBackPhotoElectricReading() == true) {
        //     endEffector.setEndEffectorRollerMotorSpeed(0.5 * motorSpeed);
        // } else {
        //     endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);
        // }

        //OLD
        if (endEffector.getEndEffectorFrontPhotoElectricReading() == true) {
            endEffector.setEndEffectorRollerMotorSpeed(Constants.Absolute_Zero);
        } else {
            endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);
        }

        //PID Control
        intake.goToIntakeWristSetpoint(23);
        endEffector.goToEndEffectorWristSetpoint();
        elevator.goToElevatorSetpoint();        

    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeRollerMotorSpeed(Constants.Absolute_Zero);
        intake.setIndexerMotorSpeed(Constants.Absolute_Zero);
        endEffector.setEndEffectorRollerMotorSpeed(Constants.Absolute_Zero);
        System.out.println("RobotIntakeGround Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}