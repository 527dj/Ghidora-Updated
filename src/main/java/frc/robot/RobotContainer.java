package frc.robot;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionManager;
import frc.robot.commands.RobotHome;
import frc.robot.commands.IntakeRunCmd;
import frc.robot.commands.IntakeWristJog;
import frc.robot.commands.IntakeWristSetpoint;
import frc.robot.commands.RobotAlgaeIntake;
import frc.robot.commands.RobotAutoPrepScore;
import frc.robot.subsystems.EndEffector;
import frc.robot.commands.EndEffectorWristJog;
import frc.robot.commands.RobotIntakeGround;
import frc.robot.commands.RobotPrepScore;
import frc.robot.commands.RobotStationIntake;
import frc.robot.commands.RobotTeleIntakeGround;
import frc.robot.commands.ScoreL1;
import frc.robot.commands.SuperIntake;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.ZeroEndEffectorWrist;
import frc.robot.commands.ZeroIntakeWrist;
import frc.robot.commands.DrivetrainRightAlign.ALIGN_STATES;
import frc.robot.commands.EndEffectorScore;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ElevatorJog;
import frc.robot.subsystems.Climb;
import frc.robot.commands.AlgaeNetScore;
import frc.robot.commands.ClimbRollerRun;
import frc.robot.commands.ClimbWristRun;
//Limelight Imports
import frc.robot.commands.DrivetrainRightAlign;
import frc.robot.commands.DrivetrainReefAutoAlignProfiled;

public class RobotContainer {
    //====================GENERAL SETUP====================
    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController driverController = new CommandXboxController(Devices.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(Devices.OPERATOR_CONTROLLER);

    //====================SWERVE SETUP====================
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.DrivetrainMaxSpeed * 0.1).withRotationalDeadband(Constants.DrivetrainMaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static final Drivetrain drivetrain = TunerConstants.createDrivetrain();    

    public RobotContainer() {
        //====================AUTONOMOUS SETUP====================
        //====================Alignment Commands====================
        NamedCommands.registerCommand("DrivetrainRightAlign", new DrivetrainRightAlign(drivetrain, VisionManager.getInstance(), ALIGN_STATES.RIGHT));
        NamedCommands.registerCommand("DrivetrainMiddleAlign", new DrivetrainRightAlign(drivetrain, VisionManager.getInstance(),ALIGN_STATES.MIDDLE));
        //====================Actions====================
        NamedCommands.registerCommand("RobotAutoPrepScoreL4", new RobotAutoPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L4_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L4_Setpoint));
        NamedCommands.registerCommand("EndEffectorScore", new EndEffectorScore(EndEffector.getInstance(), Constants.End_Effector_Score_L2_L3_L4_Speed));
        NamedCommands.registerCommand("GroundIntake", new RobotIntakeGround(EndEffector.getInstance(), Constants.End_Effector_Ground_Intake_Speed, Constants.End_Effector_Wrist_Coral_Ground_Setpoint, Intake.getInstance(), Constants.Intake_Ground_Deploy_Setpoint, Constants.Intake_Ground_Run_Speed, Elevator.getInstance(), Constants.Elevator_Ground_Coral_Setpoint));
        NamedCommands.registerCommand("BottomAlgaeRemoval", new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Bottom_Algae_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        //====================Zeroing====================
        NamedCommands.registerCommand("RobotHome", new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));
        NamedCommands.registerCommand("EndEffectorStop", new EndEffectorScore(EndEffector.getInstance(), Constants.Absolute_Zero));
        NamedCommands.registerCommand("GroundIntakeStop", new RobotIntakeGround(EndEffector.getInstance(), Constants.Absolute_Zero, Constants.Absolute_Zero, Intake.getInstance(), Constants.Intake_Zero_Setpoint, Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("DavisProcessor");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        //====================DRIVER CONTROLLER BINDINGS====================
            drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive
            .withVelocityX(-1 * MathUtil.applyDeadband(driverController.getLeftY(), Devices.JOYSTICK_DEADZONE_TOLERANCE) * Constants.Drivetrain_Speed_Multiplier * Constants.DrivetrainMaxSpeed)
            .withVelocityY(-1 * MathUtil.applyDeadband(driverController.getLeftX(), Devices.JOYSTICK_DEADZONE_TOLERANCE) * Constants.Drivetrain_Speed_Multiplier * Constants.DrivetrainMaxSpeed)
            .withRotationalRate(-1 * MathUtil.applyDeadband(driverController.getRightX(), Devices.JOYSTICK_DEADZONE_TOLERANCE) * Constants.Drivetrain_Turn_Multiplier * Constants.DrivetrainMaxAngularRate)
            )
        );

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //====================Swerve Heading Reset====================
        driverController.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //====================Align Left====================
        driverController.leftBumper().whileTrue(new DrivetrainRightAlign(drivetrain, VisionManager.getInstance(),ALIGN_STATES.LEFT));
        //====================Align Right====================
        driverController.rightBumper().whileTrue(new DrivetrainRightAlign(drivetrain, VisionManager.getInstance(), ALIGN_STATES.RIGHT));
        //====================Ground Intake====================
        BooleanSupplier hasGamePiece = () -> Intake.getInstance().getRollerCurrent() > Constants.INTAKE_CURRENT_SPIKE;
        Trigger intakeSpiked = new Trigger(hasGamePiece);
        DoubleSupplier intakePos =() -> intakeSpiked.debounce(0.01).getAsBoolean() ? Constants.Intake_Between_Setpoint : Constants.Intake_Ground_Deploy_Setpoint;
        driverController.leftTrigger().whileTrue(new RobotTeleIntakeGround(EndEffector.getInstance(), Constants.End_Effector_Ground_Intake_Speed, Constants.End_Effector_Wrist_Coral_Ground_Setpoint, Intake.getInstance(), intakePos.getAsDouble(), Constants.Intake_Ground_Run_Speed, Elevator.getInstance(), Constants.Elevator_Ground_Coral_Setpoint, driverController.getHID(), operatorController.getHID()));
        driverController.leftTrigger().onFalse(new RobotTeleIntakeGround(EndEffector.getInstance(), Constants.Absolute_Zero, Constants.Absolute_Zero, Intake.getInstance(), Constants.Intake_Stow_Setpoint, Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero, driverController.getHID(),operatorController.getHID()));
        //====================Ground Outtake====================
        driverController.povUp().whileTrue(
                Commands.parallel(    
                new IntakeRunCmd(Intake.getInstance(), Constants.Outake_Ground_Run_Speed),
                new EndEffectorScore(EndEffector.getInstance(), Constants.End_Effector_Ground_Outake_Speed)
                )
        );
    
        driverController.povUp().onFalse(
                Commands.parallel(
                new IntakeRunCmd(Intake.getInstance(), Constants.Absolute_Zero),
                new EndEffectorScore(EndEffector.getInstance(), Constants.Absolute_Zero)
                )
        );
        //====================End Effector Run====================
        driverController.rightTrigger().whileTrue(new EndEffectorScore(EndEffector.getInstance(), Constants.End_Effector_Score_L2_L3_L4_Speed));
        driverController.rightTrigger().and(operatorController.povDown()).negate().onTrue(new EndEffectorScore(EndEffector.getInstance(), Constants.Absolute_Zero));

        //======================Algae Intake=======================
        driverController.a().whileTrue(new AlgaeNetScore(EndEffector.getInstance(), Constants.End_Effector_Algae_Score_Speed, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.a().onFalse(new AlgaeNetScore(EndEffector.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverController.getHID()));

        //====================Level 2 Coral Score====================
        driverController.b().whileTrue(new RobotPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L2_L3_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L2_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.b().onFalse(new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Level 3 Coral Score====================
        driverController.x().whileTrue(new RobotPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L2_L3_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L3_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.x().onFalse(new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Level 4 Coral Score====================
        driverController.y().whileTrue(new RobotPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L4_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L4_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.y().onFalse(new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Coral Gullet Intake====================
        driverController.button(8).whileTrue(new RobotStationIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Coral_Station_Setpoint, Constants.End_Effector_Coral_Station_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Coral_Station_Setpoint));
        driverController.button(8).onFalse(new RobotStationIntake(EndEffector.getInstance(), Constants.Absolute_Zero, Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Ground Algae Intake====================
        driverController.rightStick().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Ground_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Ground_Algae_Setpoint, drivetrain, 0.3, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.rightStick().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverController.getHID()));

        //====================Bottom Algae Removal====================
        driverController.povLeft().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Bottom_Algae_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.povLeft().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverController.getHID()));

        //====================Top Algae Removal====================
        driverController.povRight().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Top_Algae_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.povRight().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverController.getHID()));

        //====================Net Algae Score====================
        driverController.leftStick().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Net_Score_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Net_Score_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.leftStick().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverController.getHID()));

        //====================End Effector Hot PID Refresh====================
        //driverController.leftBumper().onTrue(new InstantCommand(EndEffector.getInstance()::HotRefreshEndEffectorConfig));

        //====================Elevator Hot PID Refresh====================
        //driverController.leftBumper().onTrue(new InstantCommand(Elevator.getInstance()::HotRefreshElevatorConfig));

        //====================Intake Hot PID Refresh====================
        //driverController.leftBumper().onTrue(new InstantCommand(Intake.getInstance()::HotRegreshIntakeConfig));

        //====================OPERATOR CONTROLLER BINDINGS====================
        //====================Climb Wrist Up=====================
        operatorController.rightTrigger().whileTrue(new ClimbWristRun(Climb.getInstance(), Constants.Climb_Up_Speed));
        operatorController.rightTrigger().onFalse(new ClimbWristRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Climb Wrist Down=====================
        operatorController.rightBumper().whileTrue(new ClimbWristRun(Climb.getInstance(), Constants.Climb_Down_Speed));
        operatorController.rightBumper().onFalse(new ClimbWristRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Climb Roller Clockwise=====================
        operatorController.leftTrigger().whileTrue(new ClimbRollerRun(Climb.getInstance(), Constants.Climb_Up_Speed));
        operatorController.leftTrigger().onFalse(new ClimbRollerRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Climb Roller CounterClockwise=====================
        operatorController.leftBumper().whileTrue(new ClimbRollerRun(Climb.getInstance(), Constants.Climb_Down_Speed));
        operatorController.leftBumper().onFalse(new ClimbRollerRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Elevator Climb + End Effector=====================
         operatorController.a().whileTrue(new RobotHome(EndEffector.getInstance(), Constants.End_Effector_Wrist_Climb_Start_Setpoint, Elevator.getInstance(), Constants.Elevator_Climb_Setpoint));
         operatorController.a().onFalse(new RobotHome(EndEffector.getInstance(), Constants.End_Effector_Wrist_Climb_End_Setpoint, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Processor=====================
        //operatorController.x().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Processor_Score_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Processor_Score_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        //operatorController.x().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverController.getHID()));

        //====================Elevator Jog=====================
        operatorController.povUp().whileTrue(new ElevatorJog(Elevator.getInstance(), () -> operatorController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER));

        //====================Elevator Manual Zero=====================
        operatorController.y().onTrue(new ZeroElevator(Elevator.getInstance()));

        /// //====================End Effector Wrist Jog=====================
        // operatorController.povRight().whileTrue(new EndEffectorWristJog(EndEffector.getInstance(), () -> operatorController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER));

        // //====================End Effector Wrist Manual Zero=====================
        // operatorController.b().onTrue(new ZeroEndEffectorWrist(EndEffector.getInstance()));

        // //====================Intake Wrist Jog=====================
        // operatorController.povLeft().whileTrue(new IntakeWristJog(Intake.getInstance(), () -> operatorController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER));
        // //====================Intake Wrist Manual Zero=====================
        // operatorController.x().onTrue(new ZeroIntakeWrist(Intake.getInstance()));

        //====================Super Intake=====================
        // operatorController.a().whileTrue(new SuperIntake(Intake.getInstance(), Constants.Intake_Ground_Deploy_Setpoint, 0.5 * Constants.Intake_Ground_Run_Speed));
        // operatorController.a().onFalse(new SuperIntake(Intake.getInstance(), Constants.Intake_Zero_Setpoint, Constants.Absolute_Zero));

        //====================Spit L1=====================
        operatorController.povDown().whileTrue(new ScoreL1(EndEffector.getInstance(), Constants.End_Effector_Wrist_L1_Score_Setpoint));
        //MOVE INTAKE TO HIGHER SETPOINT (OPERATOR)
        //operatorController.povDown().whileTrue(new IntakeWristSetpoint(Intake.getInstance(), Constants.IntakeHighStow));
    }

        public Command getAutonomousCommand() {
            return autoChooser.getSelected();
        }
    }
    