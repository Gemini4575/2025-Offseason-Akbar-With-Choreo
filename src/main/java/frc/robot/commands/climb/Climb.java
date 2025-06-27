package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.EXO.Climb1and2EXO;
import frc.robot.commands.climb.EXO.FlipperEXO;
import frc.robot.subsystems.NickClimbingSubsystem;

public class Climb extends SequentialCommandGroup {

    public Climb(NickClimbingSubsystem nc) {
        addCommands(
                new FlipperEXO(nc).withTimeout(5),
                new Climb1and2EXO(nc));
    }

}
