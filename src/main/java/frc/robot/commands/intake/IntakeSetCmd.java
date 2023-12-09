package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetCmd extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    //private final boolean open;
    private  double speed = 0.0;

    public IntakeSetCmd(IntakeSubsystem intakeSubsystem, double speed) {
        //this.open = open;
        this.speed = speed;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("IntakeSetCmd started!");
    }

    @Override
    public void execute() {
        intakeSubsystem.setPosition(speed);
        //System.out.print("Command is executing" + open);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("IntakeSetCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}