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
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
=======
import edu.wpi.first.wpilibj2.command.button.Trigger;
>>>>>>> events-chezy
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
    //====================CONTROLLER SELECTION SETUP====================
    public enum ControllerType {
        PS5,
        XBOX
    }
    
    private final SendableChooser<ControllerType> driverControllerChooser;
    private final SendableChooser<ControllerType> operatorControllerChooser;
    
    //====================GENERAL SETUP====================
<<<<<<< HEAD
    //private final SendableChooser<Command> autoChooser;
    private final CommandPS5Controller driverPS5Controller = new CommandPS5Controller(Devices.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController driverXboxController = new CommandXboxController(Devices.DRIVER_CONTROLLER_PORT);
    private final CommandPS5Controller operatorPS5Controller = new CommandPS5Controller(Devices.OPERATOR_CONTROLLER);
    private final CommandXboxController operatorXboxController = new CommandXboxController(Devices.OPERATOR_CONTROLLER);
=======
    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController driverController = new CommandXboxController(Devices.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(Devices.OPERATOR_CONTROLLER);
>>>>>>> events-chezy

    //====================SWERVE SETUP====================
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.DrivetrainMaxSpeed * 0.1).withRotationalDeadband(Constants.DrivetrainMaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static final Drivetrain drivetrain = TunerConstants.createDrivetrain();    

    public RobotContainer() {
        //====================CONTROLLER SELECTION SETUP====================
        driverControllerChooser = new SendableChooser<>();
        driverControllerChooser.setDefaultOption("PS5 Controller", ControllerType.PS5);
        driverControllerChooser.addOption("Xbox Controller", ControllerType.XBOX);
        SmartDashboard.putData("Driver Controller Type", driverControllerChooser);
        
        operatorControllerChooser = new SendableChooser<>();
        operatorControllerChooser.setDefaultOption("PS5 Controller", ControllerType.PS5);
        operatorControllerChooser.addOption("Xbox Controller", ControllerType.XBOX);
        SmartDashboard.putData("Operator Controller Type", operatorControllerChooser);

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
        // Configure bindings based on selected controller type
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        // Set up drivetrain default command that works for both controller types
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
            ControllerType selectedType = driverControllerChooser.getSelected();
            if (selectedType == ControllerType.PS5) {
                return drive
                    .withVelocityX(-1 * MathUtil.applyDeadband(driverPS5Controller.getLeftY(), Devices.JOYSTICK_DEADZONE_TOLERANCE) * Constants.Drivetrain_Speed_Multiplier * Constants.DrivetrainMaxSpeed)
                    .withVelocityY(-1 * MathUtil.applyDeadband(driverPS5Controller.getLeftX(), Devices.JOYSTICK_DEADZONE_TOLERANCE) * Constants.Drivetrain_Speed_Multiplier * Constants.DrivetrainMaxSpeed)
                    .withRotationalRate(-1 * MathUtil.applyDeadband(driverPS5Controller.getRightX(), Devices.JOYSTICK_DEADZONE_TOLERANCE) * Constants.Drivetrain_Turn_Multiplier * Constants.DrivetrainMaxAngularRate);
            } else {
                return drive
                    .withVelocityX(-1 * MathUtil.applyDeadband(driverXboxController.getLeftY(), Devices.JOYSTICK_DEADZONE_TOLERANCE) * Constants.Drivetrain_Speed_Multiplier * Constants.DrivetrainMaxSpeed)
                    .withVelocityY(-1 * MathUtil.applyDeadband(driverXboxController.getLeftX(), Devices.JOYSTICK_DEADZONE_TOLERANCE) * Constants.Drivetrain_Speed_Multiplier * Constants.DrivetrainMaxSpeed)
                    .withRotationalRate(-1 * MathUtil.applyDeadband(driverXboxController.getRightX(), Devices.JOYSTICK_DEADZONE_TOLERANCE) * Constants.Drivetrain_Turn_Multiplier * Constants.DrivetrainMaxAngularRate);
            }
        }));

        //====================PS5 DRIVER CONTROLLER BINDINGS====================
        configurePS5DriverBindings();
        
        //====================XBOX DRIVER CONTROLLER BINDINGS====================
        configureXboxDriverBindings();
    }

    private void configurePS5DriverBindings() {
        //====================SysId Commands====================
        // driverPS5Controller.share().and(driverPS5Controller.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverPS5Controller.share().and(driverPS5Controller.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverPS5Controller.options().and(driverPS5Controller.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverPS5Controller.options().and(driverPS5Controller.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //====================Swerve Heading Reset====================
<<<<<<< HEAD
        driverPS5Controller.pov(180).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //====================Align Right====================
        driverPS5Controller.R1().whileTrue(new DrivetrainRightAlign(drivetrain, VisionManager.getInstance()));

        //====================Ground Intake====================
        driverPS5Controller.L2().whileTrue(new RobotTeleIntakeGround(EndEffector.getInstance(), Constants.End_Effector_Ground_Intake_Speed, Constants.End_Effector_Wrist_Coral_Ground_Setpoint, Intake.getInstance(), Constants.Intake_Ground_Deploy_Setpoint, Constants.Intake_Ground_Run_Speed, Elevator.getInstance(), Constants.Elevator_Ground_Coral_Setpoint, driverPS5Controller.getHID()));
        driverPS5Controller.L2().onFalse(new RobotTeleIntakeGround(EndEffector.getInstance(), Constants.Absolute_Zero, Constants.Absolute_Zero, Intake.getInstance(), Constants.Intake_Zero_Setpoint, Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero, driverPS5Controller.getHID()));

=======
        driverController.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //====================Align Left====================
        driverController.leftBumper().whileTrue(new DrivetrainRightAlign(drivetrain, VisionManager.getInstance(),ALIGN_STATES.LEFT));
        //====================Align Right====================
        driverController.rightBumper().whileTrue(new DrivetrainRightAlign(drivetrain, VisionManager.getInstance(), ALIGN_STATES.RIGHT));
        //====================Ground Intake====================
        driverController.leftTrigger().whileTrue(new RobotTeleIntakeGround(EndEffector.getInstance(), Constants.End_Effector_Ground_Intake_Speed, Constants.End_Effector_Wrist_Coral_Ground_Setpoint, Intake.getInstance(), Constants.Intake_Ground_Deploy_Setpoint, Constants.Intake_Ground_Run_Speed, Elevator.getInstance(), Constants.Elevator_Ground_Coral_Setpoint, driverController.getHID(), operatorController.getHID()));
        driverController.leftTrigger().onFalse(new RobotTeleIntakeGround(EndEffector.getInstance(), Constants.Absolute_Zero, Constants.Absolute_Zero, Intake.getInstance(), Constants.Intake_Stow_Setpoint, Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero, driverController.getHID(),operatorController.getHID()));
>>>>>>> events-chezy
        //====================Ground Outtake====================
        driverPS5Controller.pov(0).whileTrue(
                Commands.parallel(    
                new IntakeRunCmd(Intake.getInstance(), Constants.Outake_Ground_Run_Speed),
                new EndEffectorScore(EndEffector.getInstance(), Constants.End_Effector_Ground_Outake_Speed)
                )
        );
    
        driverPS5Controller.pov(0).onFalse(
                Commands.parallel(
                new IntakeRunCmd(Intake.getInstance(), Constants.Absolute_Zero),
                new EndEffectorScore(EndEffector.getInstance(), Constants.Absolute_Zero)
                )
        );
<<<<<<< HEAD

        //====================End Effector Run====================
        driverPS5Controller.R2().whileTrue(new EndEffectorScore(EndEffector.getInstance(), Constants.End_Effector_Score_L2_L3_L4_Speed));
        driverPS5Controller.R2().onFalse(new EndEffectorScore(EndEffector.getInstance(), Constants.Absolute_Zero));

        //====================Algae Net Score====================
        driverPS5Controller.cross().whileTrue(new AlgaeNetScore(EndEffector.getInstance(), Constants.End_Effector_Algae_Score_Speed, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverPS5Controller.getHID()));
        driverPS5Controller.cross().onFalse(new AlgaeNetScore(EndEffector.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverPS5Controller.getHID()));

        //====================Level 2 Coral Score====================
        driverPS5Controller.circle().whileTrue(new RobotPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L2_L3_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L2_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverPS5Controller.getHID()));
        driverPS5Controller.circle().onFalse(new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Level 3 Coral Score====================
        driverPS5Controller.square().whileTrue(new RobotPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L2_L3_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L3_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverPS5Controller.getHID()));
        driverPS5Controller.square().onFalse(new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Level 4 Coral Score====================
        driverPS5Controller.triangle().whileTrue(new RobotPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L4_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L4_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverPS5Controller.getHID()));
        driverPS5Controller.triangle().onFalse(new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));
=======
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
>>>>>>> events-chezy

        //====================Coral Gullet Intake====================
        driverPS5Controller.touchpad().whileTrue(new RobotStationIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Coral_Station_Setpoint, Constants.End_Effector_Coral_Station_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Coral_Station_Setpoint));
        driverPS5Controller.touchpad().onFalse(new RobotStationIntake(EndEffector.getInstance(), Constants.Absolute_Zero, Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Ground Algae Intake====================
        driverPS5Controller.R3().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Ground_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Ground_Algae_Setpoint, drivetrain, 0.3, Constants.Drivetrain_Elevator_Turn_Multiplier, driverPS5Controller.getHID()));
        driverPS5Controller.R3().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverPS5Controller.getHID()));

        //====================Bottom Algae Removal====================
        driverPS5Controller.pov(270).whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Bottom_Algae_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverPS5Controller.getHID()));
        driverPS5Controller.pov(270).onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverPS5Controller.getHID()));

        //====================Top Algae Removal====================
        driverPS5Controller.pov(90).whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Top_Algae_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverPS5Controller.getHID()));
        driverPS5Controller.pov(90).onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverPS5Controller.getHID()));

        //====================Net Algae Score====================
        driverPS5Controller.L3().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Net_Score_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Net_Score_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverPS5Controller.getHID()));
        driverPS5Controller.L3().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverPS5Controller.getHID()));
    }

    private void configureXboxDriverBindings() {
        //====================SysId Commands====================
        driverXboxController.back().and(driverXboxController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverXboxController.back().and(driverXboxController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverXboxController.start().and(driverXboxController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverXboxController.start().and(driverXboxController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //====================Swerve Heading Reset====================
        driverXboxController.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //====================Align Right====================
        driverXboxController.rightBumper().whileTrue(new DrivetrainRightAlign(drivetrain, VisionManager.getInstance()));

        //====================Ground Intake====================
        driverXboxController.leftTrigger().whileTrue(new RobotTeleIntakeGround(EndEffector.getInstance(), Constants.End_Effector_Ground_Intake_Speed, Constants.End_Effector_Wrist_Coral_Ground_Setpoint, Intake.getInstance(), Constants.Intake_Ground_Deploy_Setpoint, Constants.Intake_Ground_Run_Speed, Elevator.getInstance(), Constants.Elevator_Ground_Coral_Setpoint, driverXboxController.getHID()));
        driverXboxController.leftTrigger().onFalse(new RobotTeleIntakeGround(EndEffector.getInstance(), Constants.Absolute_Zero, Constants.Absolute_Zero, Intake.getInstance(), Constants.Intake_Zero_Setpoint, Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero, driverXboxController.getHID()));

        //====================Ground Outtake====================
        driverXboxController.povUp().whileTrue(
                Commands.parallel(    
                new IntakeRunCmd(Intake.getInstance(), Constants.Outake_Ground_Run_Speed),
                new EndEffectorScore(EndEffector.getInstance(), Constants.End_Effector_Ground_Outake_Speed)
                )
        );
    
        driverXboxController.povUp().onFalse(
                Commands.parallel(
                new IntakeRunCmd(Intake.getInstance(), Constants.Absolute_Zero),
                new EndEffectorScore(EndEffector.getInstance(), Constants.Absolute_Zero)
                )
        );

        //====================End Effector Run====================
        driverXboxController.rightTrigger().whileTrue(new EndEffectorScore(EndEffector.getInstance(), Constants.End_Effector_Score_L2_L3_L4_Speed));
        driverXboxController.rightTrigger().onFalse(new EndEffectorScore(EndEffector.getInstance(), Constants.Absolute_Zero));

        //====================Algae Net Score====================
        driverXboxController.a().whileTrue(new AlgaeNetScore(EndEffector.getInstance(), Constants.End_Effector_Algae_Score_Speed, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverXboxController.getHID()));
        driverXboxController.a().onFalse(new AlgaeNetScore(EndEffector.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverXboxController.getHID()));

        //====================Level 2 Coral Score====================
        driverXboxController.b().whileTrue(new RobotPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L2_L3_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L2_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverXboxController.getHID()));
        driverXboxController.b().onFalse(new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Level 3 Coral Score====================
        driverXboxController.x().whileTrue(new RobotPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L2_L3_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L3_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverXboxController.getHID()));
        driverXboxController.x().onFalse(new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Level 4 Coral Score====================
        driverXboxController.y().whileTrue(new RobotPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L4_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L4_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverXboxController.getHID()));
        driverXboxController.y().onFalse(new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Coral Gullet Intake (using left bumper for Xbox)====================
        driverXboxController.leftBumper().whileTrue(new RobotStationIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Coral_Station_Setpoint, Constants.End_Effector_Coral_Station_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Coral_Station_Setpoint));
        driverXboxController.leftBumper().onFalse(new RobotStationIntake(EndEffector.getInstance(), Constants.Absolute_Zero, Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Ground Algae Intake====================
        driverXboxController.rightStick().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Ground_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Ground_Algae_Setpoint, drivetrain, 0.3, Constants.Drivetrain_Elevator_Turn_Multiplier, driverXboxController.getHID()));
        driverXboxController.rightStick().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverXboxController.getHID()));

        //====================Bottom Algae Removal====================
        driverXboxController.povLeft().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Bottom_Algae_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverXboxController.getHID()));
        driverXboxController.povLeft().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverXboxController.getHID()));

        //====================Top Algae Removal====================
        driverXboxController.povRight().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Top_Algae_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverXboxController.getHID()));
        driverXboxController.povRight().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverXboxController.getHID()));

        //====================Net Algae Score====================
        driverXboxController.leftStick().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Net_Score_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Net_Score_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverXboxController.getHID()));
        driverXboxController.leftStick().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverXboxController.getHID()));
    }

    private void configureOperatorBindings() {
        configurePS5OperatorBindings();
        configureXboxOperatorBindings();
    }

    private void configurePS5OperatorBindings() {
        //====================Climb Wrist Up=====================
        operatorPS5Controller.R2().whileTrue(new ClimbWristRun(Climb.getInstance(), Constants.Climb_Up_Speed));
        operatorPS5Controller.R2().onFalse(new ClimbWristRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Climb Wrist Down=====================
        operatorPS5Controller.R1().whileTrue(new ClimbWristRun(Climb.getInstance(), Constants.Climb_Down_Speed));
        operatorPS5Controller.R1().onFalse(new ClimbWristRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Climb Roller Clockwise=====================
        operatorPS5Controller.L2().whileTrue(new ClimbRollerRun(Climb.getInstance(), Constants.Climb_Up_Speed));
        operatorPS5Controller.L2().onFalse(new ClimbRollerRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Climb Roller CounterClockwise=====================
        operatorPS5Controller.L1().whileTrue(new ClimbRollerRun(Climb.getInstance(), Constants.Climb_Down_Speed));
        operatorPS5Controller.L1().onFalse(new ClimbRollerRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Elevator Climb + End Effector=====================
<<<<<<< HEAD
        operatorPS5Controller.cross().whileTrue(new RobotHome(EndEffector.getInstance(), Constants.End_Effector_Wrist_Climb_Start_Setpoint, Elevator.getInstance(), Constants.Elevator_Climb_Setpoint));
        operatorPS5Controller.cross().onFalse(new RobotHome(EndEffector.getInstance(), Constants.End_Effector_Wrist_Climb_End_Setpoint, Elevator.getInstance(), Constants.Absolute_Zero));
=======
         operatorController.a().whileTrue(new RobotHome(EndEffector.getInstance(), Constants.End_Effector_Wrist_Climb_Start_Setpoint, Elevator.getInstance(), Constants.Elevator_Climb_Setpoint));
         operatorController.a().onFalse(new RobotHome(EndEffector.getInstance(), Constants.End_Effector_Wrist_Climb_End_Setpoint, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Processor=====================
        //operatorController.x().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Processor_Score_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Processor_Score_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        //operatorController.x().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Stow_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverController.getHID()));
>>>>>>> events-chezy

        //====================Elevator Jog=====================
        operatorPS5Controller.pov(0).whileTrue(new ElevatorJog(Elevator.getInstance(), () -> {
            ControllerType selectedType = operatorControllerChooser.getSelected();
            return selectedType == ControllerType.PS5 ? 
                operatorPS5Controller.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER :
                operatorXboxController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER;
        }));

        //====================Elevator Manual Zero=====================
<<<<<<< HEAD
        operatorPS5Controller.triangle().onTrue(new ZeroElevator(Elevator.getInstance()));

        //====================End Effector Wrist Jog=====================
        operatorPS5Controller.pov(90).whileTrue(new EndEffectorWristJog(EndEffector.getInstance(), () -> {
            ControllerType selectedType = operatorControllerChooser.getSelected();
            return selectedType == ControllerType.PS5 ? 
                operatorPS5Controller.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER :
                operatorXboxController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER;
        }));

        //====================End Effector Wrist Manual Zero=====================
        operatorPS5Controller.circle().onTrue(new ZeroEndEffectorWrist(EndEffector.getInstance()));

        //====================Intake Wrist Jog=====================
        operatorPS5Controller.pov(270).whileTrue(new IntakeWristJog(Intake.getInstance(), () -> {
            ControllerType selectedType = operatorControllerChooser.getSelected();
            return selectedType == ControllerType.PS5 ? 
                operatorPS5Controller.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER :
                operatorXboxController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER;
        }));
        
        //====================Intake Wrist Manual Zero=====================
        //operatorPS5Controller.square().onTrue(new ZeroIntakeWrist(Intake.getInstance()));
    }
=======
        operatorController.y().onTrue(new ZeroElevator(Elevator.getInstance()));

        /// //====================End Effector Wrist Jog=====================
        // operatorController.povRight().whileTrue(new EndEffectorWristJog(EndEffector.getInstance(), () -> operatorController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER));

        // //====================End Effector Wrist Manual Zero=====================
        // operatorController.b().onTrue(new ZeroEndEffectorWrist(EndEffector.getInstance()));

        // //====================Intake Wrist Jog=====================
        // operatorController.povLeft().whileTrue(new IntakeWristJog(Intake.getInstance(), () -> operatorController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER));
        // //====================Intake Wrist Manual Zero=====================
        // operatorController.x().onTrue(new ZeroIntakeWrist(Intake.getInstance()));
>>>>>>> events-chezy

    private void configureXboxOperatorBindings() {
        //====================Climb Wrist Up=====================
        operatorXboxController.rightTrigger().whileTrue(new ClimbWristRun(Climb.getInstance(), Constants.Climb_Up_Speed));
        operatorXboxController.rightTrigger().onFalse(new ClimbWristRun(Climb.getInstance(), Constants.Absolute_Zero));

<<<<<<< HEAD
        //====================Climb Wrist Down=====================
        operatorXboxController.rightBumper().whileTrue(new ClimbWristRun(Climb.getInstance(), Constants.Climb_Down_Speed));
        operatorXboxController.rightBumper().onFalse(new ClimbWristRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Climb Roller Clockwise=====================
        operatorXboxController.leftTrigger().whileTrue(new ClimbRollerRun(Climb.getInstance(), Constants.Climb_Up_Speed));
        operatorXboxController.leftTrigger().onFalse(new ClimbRollerRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Climb Roller CounterClockwise=====================
        operatorXboxController.leftBumper().whileTrue(new ClimbRollerRun(Climb.getInstance(), Constants.Climb_Down_Speed));
        operatorXboxController.leftBumper().onFalse(new ClimbRollerRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Elevator Climb + End Effector=====================
        operatorXboxController.a().whileTrue(new RobotHome(EndEffector.getInstance(), Constants.End_Effector_Wrist_Climb_Start_Setpoint, Elevator.getInstance(), Constants.Elevator_Climb_Setpoint));
        operatorXboxController.a().onFalse(new RobotHome(EndEffector.getInstance(), Constants.End_Effector_Wrist_Climb_End_Setpoint, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Elevator Jog=====================
        operatorXboxController.povUp().whileTrue(new ElevatorJog(Elevator.getInstance(), () -> {
            ControllerType selectedType = operatorControllerChooser.getSelected();
            return selectedType == ControllerType.PS5 ? 
                operatorPS5Controller.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER :
                operatorXboxController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER;
        }));

        //====================Elevator Manual Zero=====================
        operatorXboxController.y().onTrue(new ZeroElevator(Elevator.getInstance()));

        //====================End Effector Wrist Jog=====================
        operatorXboxController.povRight().whileTrue(new EndEffectorWristJog(EndEffector.getInstance(), () -> {
            ControllerType selectedType = operatorControllerChooser.getSelected();
            return selectedType == ControllerType.PS5 ? 
                operatorPS5Controller.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER :
                operatorXboxController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER;
        }));

        //====================End Effector Wrist Manual Zero=====================
        operatorXboxController.b().onTrue(new ZeroEndEffectorWrist(EndEffector.getInstance()));

        //====================Intake Wrist Jog=====================
        operatorXboxController.povLeft().whileTrue(new IntakeWristJog(Intake.getInstance(), () -> {
            ControllerType selectedType = operatorControllerChooser.getSelected();
            return selectedType == ControllerType.PS5 ? 
                operatorPS5Controller.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER :
                operatorXboxController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER;
        }));
        
        //====================Intake Wrist Manual Zero=====================
        //operatorXboxController.x().onTrue(new ZeroIntakeWrist(Intake.getInstance()));
    }

        // public Command getAutonomousCommand() {
        //     return autoChooser.getSelected();
        // }
=======
        //====================Spit L1=====================
        operatorController.povDown().whileTrue(new ScoreL1(EndEffector.getInstance(), Constants.End_Effector_Wrist_L1_Score_Setpoint));
        //MOVE INTAKE TO HIGHER SETPOINT (OPERATOR)
        //operatorController.povDown().whileTrue(new IntakeWristSetpoint(Intake.getInstance(), Constants.IntakeHighStow));
    }

        public Command getAutonomousCommand() {
            return autoChooser.getSelected();
        }
>>>>>>> events-chezy
    }