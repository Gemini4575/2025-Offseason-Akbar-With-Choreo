package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.driving.AlineWheels;
import frc.robot.commands.driving.Stop;
import frc.robot.subsystems.drive.Drivetrain;

public class AutoChoreoTesting extends Command {
    Drivetrain drivetrain;

    public AutoChoreoTesting(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        // Add your commands here, e.g.:
        // addCommands(new ExampleCommand());
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(
                Commands.sequence(
                        new AlineWheels(drivetrain),
                        drivetrain.autoFactory.resetOdometry("1 Meter Testing"),
                        drivetrain.autoFactory.trajectoryCmd("1 Meter Testing"),
                        new Stop(drivetrain)));
    }

    @Override
    public void execute() {
        // This method is called repeatedly while the command is scheduled.
    }

    @Override
    public boolean isFinished() {
        // This method returns true when the command should end.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // This method is called once the command ends or is interrupted.
    }
}
