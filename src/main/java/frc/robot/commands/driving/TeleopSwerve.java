package frc.robot.commands.driving;

import frc.robot.subsystems.drive.Drivetrain;

import java.net.http.HttpResponse.BodySubscriber;
import java.security.PublicKey;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private final Drivetrain m_drivetrain;
    private final DoubleSupplier m_xSpeedSupplier;
    private final DoubleSupplier m_ySpeedSupplier;
    private final DoubleSupplier m_rotSupplier;
    private final BooleanSupplier m_SlowMode;
    private final IntSupplier dpad;

    public TeleopSwerve(Drivetrain drivetrain, DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier rotSupplier, BooleanSupplier SlowMode, IntSupplier dpad) {
        m_drivetrain = drivetrain;
        m_xSpeedSupplier = xSpeedSupplier;
        m_ySpeedSupplier = ySpeedSupplier;
        m_rotSupplier = rotSupplier;
        m_SlowMode = SlowMode;
        this.dpad = dpad;

        addRequirements(m_drivetrain);
    }

    public ChassisSpeeds dpadCalculator() {
        switch (dpad.getAsInt()) {
            case 0: // Up
                return new ChassisSpeeds(0.5, 0.0, 0.0);
            case 45: // Up-Right
                return new ChassisSpeeds(0.5, 0.5, 0.0);
            case 90: // Right
                return new ChassisSpeeds(0.0, 0.5, 0.0);
            case 135: // Down-Right
                return new ChassisSpeeds(-0.5, 0.5, 0.0);
            case 180: // Down
                return new ChassisSpeeds(-0.5, 0.0, 0.0);
            case 225: // Down-Left
                return new ChassisSpeeds(-0.5, -0.5, 0.0);
            case 270: // Left
                return new ChassisSpeeds(0.0, -0.5, 0.0);
            case 315: // Up-Left
                return new ChassisSpeeds(0.5, -0.5, 0.0);
            default: // D-pad not pressed
                return new ChassisSpeeds(0.0, 0.0, 0.0);
        }
    }

    @Override
    public void execute() {
        if (dpad.getAsInt() == -1) {
            if (m_SlowMode.getAsBoolean()) {
                double xSpeed = MathUtil.applyDeadband(m_xSpeedSupplier.getAsDouble(), 0.1) * .5;
                double ySpeed = -MathUtil.applyDeadband(m_ySpeedSupplier.getAsDouble(), 0.1) * .5;
                double rot = MathUtil.applyDeadband(m_rotSupplier.getAsDouble(), 0.1) * .5;
                m_drivetrain.drive(xSpeed, ySpeed, rot, true);
            } else {
                double xSpeed = MathUtil.applyDeadband(m_xSpeedSupplier.getAsDouble(), 0.1);
                double ySpeed = -MathUtil.applyDeadband(m_ySpeedSupplier.getAsDouble(), 0.1);
                double rot = MathUtil.applyDeadband(m_rotSupplier.getAsDouble(), 0.1);
                m_drivetrain.drive(xSpeed * 2, ySpeed * 2, rot * 2, true);
            }
        } else {
            m_drivetrain.driveRobotRelative(dpadCalculator());
        }
    }
}
