// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDrive extends CommandBase {
  private final SwerveSubsystem  swerve;
  private final DoubleSupplier   vX;
  private final DoubleSupplier   vY;
  private final DoubleSupplier   omega;
  private final BooleanSupplier  driveMode;  
  private final SwerveController controller;
  private final Timer            timer    = new Timer();
  private final boolean          headingCorrection;
  private       double           angle    = 0;
  private       double           lastTime = 0;


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
                     BooleanSupplier driveMode, boolean isOpenLoop, boolean headingCorrection) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;    
    this.controller = swerve.getSwerveController();
    this.headingCorrection = headingCorrection;
    if (headingCorrection) {
      timer.start();
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (headingCorrection) {
      lastTime = timer.get();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity   = vX.getAsDouble();
    double yVelocity   = vY.getAsDouble();
    double angVelocity = omega.getAsDouble();
    SmartDashboard.putNumber("vX", xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);
    if (headingCorrection) {
      // Estimate the desired angle in radians.
      angle += (angVelocity * (timer.get() - lastTime)) * controller.config.maxAngularVelocity;
      // Get the desired ChassisSpeeds given the desired angle and current angle.
      ChassisSpeeds correctedChassisSpeeds = controller.getTargetSpeeds(xVelocity, yVelocity, angle, swerve.getHeading().getRadians());
      // Drive using given data points.
      swerve.drive(SwerveController.getTranslation2d(correctedChassisSpeeds), correctedChassisSpeeds.omegaRadiansPerSecond, driveMode.getAsBoolean());
      lastTime = timer.get();
    } else {
      // Drive using raw values.
      swerve.drive(new Translation2d(xVelocity * controller.config.maxSpeed, yVelocity * controller.config.maxSpeed),
                   angVelocity * controller.config.maxAngularVelocity,
                   driveMode.getAsBoolean());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}