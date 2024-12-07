package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import swervelib.SwerveModule;
import swervelib.motors.SwerveMotor;

public class OurShuffleboard {
    private RobotContainer rc;
    private ShuffleboardLayout motorLayout;    
    public ShuffleboardTab sonicConfigTab;

    public OurShuffleboard(RobotContainer robotContainer) {        
        this.rc = robotContainer;
        this.sonicConfigTab = Shuffleboard.getTab("SwerveConfig");
        refreshSwerveCheck();
        //addJoystickCheck();
    }

    public void addCanCheck(SwerveMotor mc, String name) {        
        //SmartDashboard.putNumber(name, mc.getPosition());        
        motorLayout.addDouble(name, mc::getPosition);
    }

    public void addAbsoluteEncoderCheck(SwerveModule mc, String name) {        
        //SmartDashboard.putNumber(name, mc.getAbsolutePosition());
        motorLayout.addDouble(name, mc::getAbsolutePosition);
    }


    public void refreshSwerveCheck() {        
        motorLayout = sonicConfigTab.getLayout("Swerve", BuiltInLayouts.kList).withPosition(1, 0).withSize(2, 4);
        for(int x=0;x<rc.drivebase.swerveDrive.getModules().length;x++) {
            addCanCheck(rc.drivebase.swerveDrive.getModules()[x].getAngleMotor(), "Swerve/Angle "+x);
            addCanCheck(rc.drivebase.swerveDrive.getModules()[x].getDriveMotor(), "Swerve/Drive "+x);
        }
        for(int x=0;x<rc.drivebase.swerveDrive.getModules().length;x++) {
            addAbsoluteEncoderCheck(rc.drivebase.swerveDrive.getModules()[x], "Swerve/AbsEnc "+x);
        }
    }


}
