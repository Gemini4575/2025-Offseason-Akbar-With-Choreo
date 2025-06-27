package frc.robot.commands.driving;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;

public class Spin180 extends Command {
    Drivetrain d;
    boolean isFinished;

    public Spin180(Drivetrain d) {
        this.d = d;
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        isFinished = d.rotate(new Rotation2d().fromDegrees(180));
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean ds) {
        if (ds) {
            d.end();
        }
    }
}
