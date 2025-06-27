package frc.robot.commands.algea.EXO;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NickClimbingSubsystem;
import frc.robot.subsystems.OzzyGrabberSubsystem;

public class OzDown extends Command{
    private OzzyGrabberSubsystem climbing;

    public OzDown(OzzyGrabberSubsystem subsystem) {
        // Initialization code here
        climbing = subsystem;
        addRequirements(climbing);
    }

    boolean isFinished;
    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void execute() {
        // Code to move the elevator
        climbing.down();
    }

    @Override
    public void end(boolean interrupted) {
        climbing.end();
    }
}
