package frc.robot.commands.coral.lili;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiliCoralSubystem;

public class EXODropGate extends Command{
    LiliCoralSubystem c;
    public EXODropGate(LiliCoralSubystem cc){
        this.c = cc;
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
        if(c.DropGate()) {
            isFinished = true;
        }
    }
}
