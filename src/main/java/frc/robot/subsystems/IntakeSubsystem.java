package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorPort, MotorType.kBrushless);

    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
    }

    public void setPosition(double intakeSpeed) {
        intakeMotor.set(intakeSpeed);
        // if (open) {
        //     intakeMotor.set(IntakeConstants.kOpenSpeed);
        // } else {
        //     intakeMotor.set(IntakeConstants.kCloseSpeed);
        // }
    }
}