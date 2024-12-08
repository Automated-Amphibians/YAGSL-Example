package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriverControl {
    private double flip = 1.0;  
    private AbsoluteDriveAdv closedAbsoluteDriveAdv;
    private SwerveSubsystem drivebase;
    private CommandXboxController driverXbox;
    
    public Command getDriveCommand(SwerveSubsystem drivebase, CommandXboxController driverXbox) {
       this.drivebase = drivebase;
       this.driverXbox = driverXbox;
       closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,     
            () -> {
              int pov = driverXbox.getHID().getPOV();
              if (pov > -1) {
                return Rotation2d.fromDegrees(pov).getCos() * flip;
              } else {
                return -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) * flip;
              }
            },
            () -> {
              int pov = driverXbox.getHID().getPOV();
              if (pov > -1) {
                return Rotation2d.fromDegrees(pov - 180).getSin() * flip;
              } else {
                return -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) * flip;
              }
            },
            () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND) * flip
       );      
      return closedAbsoluteDriveAdv;
    }

    private Command getSetTargetHeadingCmd(int heading) {
    return Commands.runOnce(() -> {
        if (flip == 1) {
          closedAbsoluteDriveAdv.setTargetHeading(Rotation2d.fromDegrees(heading));
        } else {
          closedAbsoluteDriveAdv.setTargetHeading(Rotation2d.fromDegrees(heading - 180));
        }
    });
  }

  public void configureDriverBindings() {    
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.back().onTrue(Commands.runOnce(() -> {flip = flip * -1;}));
    /*
    driverXbox.leftBumper().onTrue(Commands.runOnce(() -> {
      flip = flip < 0.0 ? flip + 0.33 : flip - 0.33;
      if (Math.abs(flip) < 0.3) {
        flip = flip < 0.0 ? -1 : 1;
      }
    }));
    */
    driverXbox.rightTrigger()
       .onTrue(Commands.runOnce(() -> {
         flip = flip < 0 ? -0.3 : 0.3;
       }))
       .onFalse(Commands.runOnce(() -> {
        flip = flip < 0 ? -1 : 1;
       }));
    driverXbox.rightBumper()
       .onTrue(Commands.runOnce(() -> {
         closedAbsoluteDriveAdv.setFieldOriented(false);
       }))
       .onFalse(Commands.runOnce(() -> {
        closedAbsoluteDriveAdv.setFieldOriented(true);
       }));
    driverXbox.y().onTrue(getSetTargetHeadingCmd(0));
    driverXbox.x().onTrue(getSetTargetHeadingCmd(90));
    driverXbox.a().onTrue(getSetTargetHeadingCmd(180));
    driverXbox.b().onTrue(getSetTargetHeadingCmd(270));    
  }  
}
