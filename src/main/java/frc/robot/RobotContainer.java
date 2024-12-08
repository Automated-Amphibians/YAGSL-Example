// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swervedrive.drivebase.DriverControl;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {  
  final CommandXboxController driverXbox = new CommandXboxController(0);  
  //public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/sonic"));
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ruby"));
  private OurShuffleboard shuffleboard;
  private DriverControl driverControl = new DriverControl();

  public RobotContainer() {    
    Command driverCommand = driverControl.getDriveCommand(drivebase, driverXbox);
    drivebase.setDefaultCommand(driverCommand);
    driverControl.configureDriverBindings();
    
    this.shuffleboard = new OurShuffleboard(this); 
  }

}
