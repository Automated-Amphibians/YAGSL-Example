// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;
  private SnapTo snapTo = new SnapTo();

  private double last_set = 0;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    PathPlannerServer.startServer(5811);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
  }

  private double snapAngle;
  private String lastLookie = ""; 

  @Override
  public void testInit() {
    // Cancels any currently running commands.
    CommandScheduler.getInstance().cancelAll();    
    
    // give snapto a way to measure with the gyroscope
    snapTo.gyroMeasurement = () -> {return m_robotContainer.drivebase.swerveDrive.getYaw().getDegrees();};

    // give snapto a way to turn
    snapTo.turnMethod =  (turnPower) -> {
      Rotation2d yaw = m_robotContainer.drivebase.swerveDrive.getYaw();
      m_robotContainer.drivebase.swerveDrive.driveFieldOriented(DrivetrainFun.angleAndSpeedToChassisSpeeds(0, 0, turnPower, yaw));
    };
    
    // put the robot in an easier to view location on the field
    DrivetrainFun.estimateRobotPositionOnField(3, 3, 0, m_robotContainer.drivebase.swerveDrive);

    m_robotContainer.setMotorBrake(true);
  }


  @Override
  public void testPeriodic() {
    
    // every 2 seconds we're going to change the snap to angle to a different right angle
    int currTime = (int)Timer.getFPGATimestamp();
    if (currTime % 2 == 0) {
      if (last_set != currTime) {
        snapAngle = (snapAngle + (int)((Math.random()*2)+1)*90) % 360;
        snapTo.setSnapTo(snapAngle);        
        last_set = currTime;
      }
    }    
    
    // perform one cycle of the snapping action
    double output = snapTo.doSnapToCycle(0.1, 10);

    // show the amount of power utilized during the snap to action
    String lookie = String.format("%.3f, %.3f, %.3f", m_robotContainer.drivebase.swerveDrive.getYaw().getDegrees(), output, snapTo.timeToSnap);
    if (!lastLookie.equals(lookie)) {            
      System.out.println(lookie);
      lastLookie = lookie;      
    }
      
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
