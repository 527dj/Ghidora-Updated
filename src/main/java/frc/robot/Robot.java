package frc.robot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.PlayMusicCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionManager;
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final boolean kUseLimelight = true;
  private HotChangeManager hotChangeManager;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
      super.robotInit();
      hotChangeManager = HotChangeManager.getInstance();
      //Intake.getInstance().zeroIntakeWristWithAbsolute();
      VisionManager.getInstance();
          new PlayMusicCommand(
        Climb.getInstance().getClimbWristMotor(),
        Climb.getInstance().getClimbRollerMotor()
    ).schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //Match Time Log
    double matchTime = Timer.getMatchTime();
    SmartDashboard.putNumber("Match Time", matchTime);
   
    //Battery Voltage Log
    double batteryVoltage = RobotController.getBatteryVoltage();
    SmartDashboard.putNumber("Battery Voltage", batteryVoltage);

    hotChangeManager.updateConstants();
  }

  @Override
  public void disabledInit() {
    hotChangeManager.printCurrentValues();
    // Intake.getInstance().coastInit();
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // Intake.getInstance().brakeInit();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // Intake.getInstance().brakeInit();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}