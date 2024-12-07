// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {  
  final CommandXboxController driverXbox = new CommandXboxController(0);  
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ruby"));
  private double flip = 1.0;
  
  private AbsoluteDriveAdv closedAbsoluteDriveAdv;

  public RobotContainer() {
    
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
    configureDriverBindings();
    drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
    
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

  private void configureDriverBindings() {    
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.back().onTrue(Commands.runOnce(() -> {flip = flip * -1;}));
    driverXbox.leftBumper().onTrue(Commands.runOnce(() -> {
      flip = flip < 0.0 ? flip + 0.25 : flip - 0.25;
      if (Math.abs(flip) < 0.5) {
        flip = flip < 0.0 ? -1 : 1;
      }
    }));
    driverXbox.rightTrigger()
       .onTrue(Commands.runOnce(() -> {
         flip = flip < 0 ? -0.5 : 0.5;
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
    //driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
  }  

}
