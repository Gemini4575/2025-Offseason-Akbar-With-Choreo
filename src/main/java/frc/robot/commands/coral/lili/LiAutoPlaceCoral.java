package frc.robot.commands.coral.lili;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LiliCoralSubystem;

public class LiAutoPlaceCoral extends SequentialCommandGroup{
    public LiAutoPlaceCoral(LiliCoralSubystem c) {
        addCommands(
            new EXODropGate(c).withTimeout(3),
            new WaitCommand(1.25)
        );
    }
}
