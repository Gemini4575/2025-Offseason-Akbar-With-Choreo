package frc.robot.commands.driving;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;

public class Stop extends Command {
    Drivetrain d;
    boolean isFinished;
    Timer timer;

    public Stop(Drivetrain d) {
        this.d = d;
        addRequirements(d);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        isFinished = false;
        timer.start();
    }

    @Override
    public void execute() {
        d.drive(0, .0, 0, false);
        if (timer.advanceIfElapsed(1.5)) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
