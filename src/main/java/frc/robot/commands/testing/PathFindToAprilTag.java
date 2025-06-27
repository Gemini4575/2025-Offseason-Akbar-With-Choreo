package frc.robot.commands.testing;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drivetrain;

public class PathFindToAprilTag extends Command {
    private final Vision vision;
    private final Drivetrain driveSubsystem;
    private Command cmd;

    public PathFindToAprilTag(Vision vision, Drivetrain driveSubsystem) {
        this.vision = vision;
        this.driveSubsystem = driveSubsystem;
        addRequirements(vision, driveSubsystem);
    }

    @Override
    public void initialize() {
        cmd = new PathFindToPose(driveSubsystem, () -> vision.getAprilTagTarget());
        CommandScheduler.getInstance().schedule(cmd);
    }

    @Override
    public boolean isFinished() {
        return cmd == null || cmd.isFinished();
    }
}
