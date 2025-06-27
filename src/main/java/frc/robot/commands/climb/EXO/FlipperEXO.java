package frc.robot.commands.climb.EXO;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NickClimbingSubsystem;

public class FlipperEXO extends Command {
    private final NickClimbingSubsystem nc;

    public FlipperEXO(NickClimbingSubsystem nc) {
        this.nc = nc;
        addRequirements(nc);
    }

    boolean isFinished;

    @Override
    public void initialize() {
        timer.start();
        isFinished = false;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        nc.Flipper(0);
        timer.stop();
        timer.reset();
    }

    Timer timer = new Timer();

    @Override
    public void execute() {
        // Code to move the elevator
        if (timer.get() < 5) {
            nc.Flipper(1);
        }
    }
}
