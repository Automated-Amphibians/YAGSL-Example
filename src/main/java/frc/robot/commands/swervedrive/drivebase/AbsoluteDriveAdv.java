// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * A more advanced Swerve Control System that has 4 buttons for which direction to face
 */
public class AbsoluteDriveAdv extends Command
{

  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY;
  private final DoubleSupplier  rotationalVelocity;  
  private Rotation2d targetHeading = null;
  private boolean isKeptTarget = false;
  private boolean fieldOriented = true;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. Heading Adjust changes the current heading after being
   * multipied by a constant. The look booleans are shortcuts to get the robot to face a certian direction. Based off of
   * ideas in https://www.chiefdelphi.com/t/experiments-with-a-swerve-steering-knob/446172
   *
   * @param swerve        The swerve drivebase subsystem.
   * @param vX            DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY            DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive Y is towards the left wall when looking through
   *                      the driver station glass.
   * @param rotationalVelocity DoubleSupplier that supplies the component of the robot's heading angle that should be
   *                      adjusted. Should range from -1 to 1 with deadband already accounted for.

   */
  public AbsoluteDriveAdv(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rotationalVelocity) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.rotationalVelocity = rotationalVelocity;
    this.targetHeading = null;
    this.isKeptTarget = false;
    this.fieldOriented = true;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    
    //ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), Rotation2d.fromDegrees(90));
    
    Rotation2d heading = swerve.getHeading();
    Double requestedRotationalVel = rotationalVelocity.getAsDouble();

    // if controller is requesting rotation.... 
    if (Math.abs(requestedRotationalVel) > 0) {      
       // we're going to provide a target to rotate to relative to where we currently are
      this.targetHeading = Rotation2d.fromDegrees(heading.getDegrees() + (requestedRotationalVel * Constants.MAX_ANGULAR_DEGREES));      
      this.isKeptTarget = false;      
    } else {      
      // if the controller is NOT requesting any rotation and we didn't use a snap-to last, then we can clear the target heading
      if (!this.isKeptTarget) {      
        this.targetHeading = null;
      }
    }
      
    ChassisSpeeds desiredSpeeds;  
    desiredSpeeds = swerve.getSwerveController().getTargetSpeeds(
                      vX.getAsDouble(),
                      vY.getAsDouble(),
                      this.targetHeading == null ? heading.getRadians() : this.targetHeading.getRadians(),
                      heading.getRadians(),                      
                      Constants.MAX_SPEED);    

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);

    translation = SwerveMath.limitVelocity(translation, 
                                           swerve.getFieldVelocity(), swerve.getPose(),
                                           Constants.LOOP_TIME, 
                                           Constants.ROBOT_MASS, 
                                           List.of(Constants.CHASSIS),
                                           swerve.getSwerveDriveConfiguration());

    //SlewRateLimiter srl = new SlewRateLimiter(headingY);

    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    if (Math.abs(rotationalVelocity.getAsDouble()) > 0) {      
      desiredSpeeds.omegaRadiansPerSecond = Constants.OperatorConstants.TURN_CONSTANT * -rotationalVelocity.getAsDouble();
    } 
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, this.fieldOriented);
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

  public void setTargetHeading(Rotation2d heading) {
    this.targetHeading = heading;
    this.isKeptTarget = true;
  }

  public void setFieldOriented(boolean b) {
    this.fieldOriented = b;
  }


}
