package frc.robot.commands.climb.EXO;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NickClimbingSubsystem;

public class Climb1and2EXO extends Command {
    private final NickClimbingSubsystem nc;

    public Climb1and2EXO(NickClimbingSubsystem nc) {
        this.nc = nc;
        addRequirements(nc);
    }

    boolean isFinished;

    @Override
    public void initialize() {
        nc.Climb();
        isFinished = false;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void execute() {
        // Code to move the elevator
        isFinished = nc.Climb();
    }

}
