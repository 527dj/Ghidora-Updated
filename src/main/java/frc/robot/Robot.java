package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.PlayMusicCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.VisionManager;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final ElasticManager elasticManager;

  public Robot() {
    m_robotContainer = new RobotContainer();
    elasticManager = ElasticManager.getInstance();
  }

  @Override
  public void robotInit() {
    super.robotInit();
    
    // Initialize subsystems
    VisionManager.getInstance();
    
    // Play startup music
    new PlayMusicCommand(
        Climb.getInstance().getClimbWristMotor(),
        Climb.getInstance().getClimbRollerMotor()
    ).schedule();
    
    System.out.println("====================Robot Initialization Complete====================");
  }

  @Override
  public void robotPeriodic() {
    // Run command scheduler
    CommandScheduler.getInstance().run();
    
    // Update Elastic dashboard (this handles all telemetry)
    elasticManager.periodic();
  }

  @Override
  public void disabledInit() {
    System.out.println("====================Robot Disabled====================");
    elasticManager.printValuesSummary();
  }

  @Override
  public void disabledPeriodic() {
    // Nothing needed here - elasticManager.periodic() handles updates
  }

  @Override
  public void disabledExit() {
    System.out.println("====================Exiting Disabled Mode====================");
  }

  @Override
  public void autonomousInit() {
    System.out.println("====================Autonomous Mode Started====================");
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      System.out.println("Auto command: " + m_autonomousCommand.getName());
    } else {
      System.err.println("WARNING: No autonomous command selected!");
    }
  }

  @Override
  public void autonomousPeriodic() {
    // Command scheduler runs in robotPeriodic()
  }

  @Override
  public void autonomousExit() {
    System.out.println("====================Autonomous Mode Ended====================");
  }

  @Override
  public void teleopInit() {
    System.out.println("====================Teleop Mode Started====================");
    
    // Cancel autonomous if still running
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Command scheduler runs in robotPeriodic()
  }

  @Override
  public void teleopExit() {
    System.out.println("====================Teleop Mode Ended====================");
  }

  @Override
  public void testInit() {
    System.out.println("====================Test Mode Started====================");
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // Test mode functionality
  }

  @Override
  public void testExit() {
    System.out.println("====================Test Mode Ended====================");
  }

  @Override
  public void simulationPeriodic() {
    // Simulation-specific updates
  }
}