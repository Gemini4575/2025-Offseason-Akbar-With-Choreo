package frc.robot.commands.coral.lili;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiliCoralSubystem;

public class EXOCloseGateSlow extends Command {
    LiliCoralSubystem c;

    public EXOCloseGateSlow(LiliCoralSubystem c) {
        this.c = c;
        addRequirements(c);
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
        if (c.CloseGateSlow()) {
            isFinished = true;
        }
    }
}
