package frc.robot.commands.coral.lili;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiliCoralSubystem;

public class AUTOCoral extends Command{
    boolean isFinished;
    LiliCoralSubystem c;
    public AUTOCoral(LiliCoralSubystem s) {
        c = s;
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        if(c.Coral().getAsBoolean()) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
