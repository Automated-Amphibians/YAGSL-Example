package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class Utils {
 
    static boolean amRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;        
    }
}
