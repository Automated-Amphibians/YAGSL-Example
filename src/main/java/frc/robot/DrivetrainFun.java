package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;

public class DrivetrainFun {
    public double angle = 0;
    private double change = 2.5;
    private SwerveDrive swerveDrive;
  
  
    public DrivetrainFun(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;        
    }

    /**
     * Tells the pose estimator where we are positioned on the field.
     *      
     * @param x - The location along the X axis. The x-axis is forward (positive) to backward (negative) on the field, presuming the driver is standing on the blue side.
     * @param y - The location along the Y axis. The y-axis is left (positive) to right (negative) on the field, presuming the driver is standing on the blue side.
     * @param rotationInDegrees - Rotation of the robot in CW+, (rotation to the right is positive)
     */
    public void estimateRobotPositionOnField(double x, double y, double rotationInDegrees) {
      Pose2d pose2d = new Pose2d(x, y, Rotation2d.fromDegrees(rotationInDegrees));      
      swerveDrive.swerveDrivePoseEstimator.resetPosition(
          Rotation2d.fromDegrees(0), 
          swerveDrive.getModulePositions(), 
          pose2d);
    }

    static public void estimateRobotPositionOnField(double x, double y, double rotationInDegrees, SwerveDrive swerveDrive) {
        Pose2d pose2d = new Pose2d(x, y, Rotation2d.fromDegrees(rotationInDegrees));      
        swerveDrive.swerveDrivePoseEstimator.resetPosition(
            Rotation2d.fromDegrees(0), 
            swerveDrive.getModulePositions(), 
            pose2d);
      }
  
  
    public void setAngleForModule(SwerveModule sm, double angle) {
      SwerveModuleState sms = new SwerveModuleState();
      sms.angle = Rotation2d.fromDegrees(angle);
      sms.speedMetersPerSecond = 1;
      sm.setDesiredState(sms, false, true);    
      SmartDashboard.putNumber("angle", angle);
    }
  
    public ChassisSpeeds angleAndSpeedToChassisSpeeds(double angleInDegrees, double speed) {    
        return angleAndSpeedToChassisSpeeds(angleInDegrees, speed, 0);
    }

    public ChassisSpeeds angleAndSpeedToChassisSpeeds(double angleInDegrees, double speed, double turnSpeed) {    
      // Convert angle to radians
      double angleRadians = Math.toRadians(angleInDegrees);
  
      // Calculate X and Y components
      double xSpeed = Math.cos(angleRadians) * speed;
      double ySpeed = Math.sin(angleRadians) * speed;      
  
      // Now xSpeed and ySpeed hold the X and Y components of the speed
      return ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveDrive.getYaw());
    } 

    static public ChassisSpeeds angleAndSpeedToChassisSpeeds(double angleInDegrees, double speed, double turnSpeed, Rotation2d currentYaw) {    
        // Convert angle to radians
        double angleRadians = Math.toRadians(angleInDegrees);
    
        // Calculate X and Y components
        double xSpeed = Math.cos(angleRadians) * speed;
        double ySpeed = Math.sin(angleRadians) * speed;      
    
        // Now xSpeed and ySpeed hold the X and Y components of the speed
        return ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, currentYaw);
      } 
    
    public void incrementAngle() {
        angle = angle + change;
    }
     
}
