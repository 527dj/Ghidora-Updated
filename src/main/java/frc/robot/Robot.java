package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.PlayMusicCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.VisionManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final ShuffleboardManager shuffleboardManager;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Initialize robot container (subsystems, commands, button bindings)
    m_robotContainer = new RobotContainer();
    
    // Initialize Shuffleboard dashboard manager
    shuffleboardManager = ShuffleboardManager.getInstance();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotInit() {
    super.robotInit();
    
    // Initialize vision manager
    VisionManager.getInstance();
    
    // Play startup music on climb motors
    new PlayMusicCommand(
        Climb.getInstance().getClimbWristMotor(),
        Climb.getInstance().getClimbRollerMotor()
    ).schedule();
    
    System.out.println("====================Robot Initialization Complete====================");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want to run during disabled, autonomous, teleoperated and test.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    // Update Shuffleboard dashboard with current robot state
    // This single call handles ALL dashboard updates:
    // - Match time, battery voltage, alliance color
    // - Robot pose on field
    // - Limelight/vision data
    // - Tunable constants (reads from sliders)
    // - System diagnostics
    shuffleboardManager.periodic(
        Timer.getMatchTime(),                      // Current match time
        RobotController.getBatteryVoltage(),       // Battery voltage
        RobotContainer.drivetrain.getState().Pose  // Current robot pose
    );
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.out.println("====================Robot Disabled====================");
    
    // Print summary of all tunable values to console
    shuffleboardManager.printValuesSummary();
  }

  @Override
  public void disabledPeriodic() {
    // Nothing needed here - shuffleboardManager.periodic() runs in robotPeriodic()
  }

  /** This autonomous runs the autonomous command selected by your RobotContainer class. */
  @Override
  public void disabledExit() {
    System.out.println("====================Exiting Disabled Mode====================");
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
    System.out.println("====================Autonomous Mode Started====================");
    
    // Get the selected autonomous command from the dashboard chooser
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      System.out.println("Auto command scheduled: " + m_autonomousCommand.getName());
    } else {
      System.err.println("WARNING: No autonomous command selected!");
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Command scheduler runs in robotPeriodic()
    // No additional code needed here
  }

  @Override
  public void autonomousExit() {
    System.out.println("====================Autonomous Mode Ended====================");
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("====================Teleop Mode Started====================");
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      System.out.println("Autonomous command cancelled");
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Command scheduler runs in robotPeriodic()
    // Driver controls are handled by button bindings in RobotContainer
  }

  @Override
  public void teleopExit() {
    System.out.println("====================Teleop Mode Ended====================");
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    System.out.println("====================Test Mode Started====================");
    
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Test mode functionality can be added here
  }

  @Override
  public void testExit() {
    System.out.println("====================Test Mode Ended====================");
  }

  /** This function is called periodically during simulation. */
  @Override
  public void simulationPeriodic() {
    // Simulation-specific updates can be added here
  }
}