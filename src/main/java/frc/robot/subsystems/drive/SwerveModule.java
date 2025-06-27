package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {

    private ProfiledPIDController turningPidController = new ProfiledPIDController(
            16, // Proportional gain
            0.0, // Integral gain
            0.0,
            new TrapezoidProfile.Constraints(
                    SwerveConstants.kModuleMaxAngularVelocity,
                    SwerveConstants.kModuleMaxAngularAcceleration));

    private ProfiledPIDController drivingPidController = new ProfiledPIDController(
            1.0, // Proportional gain
            0.0, // Integral gain
            0.0,
            new TrapezoidProfile.Constraints(
                    SwerveConstants.MaxMetersPersecond,
                    SwerveConstants.kMaxAceceration));

    private SparkMax driveMotor;
    private SparkMax angleMotor;

    private AnalogInput Encoder;
    private RelativeEncoder m_driveEncoder;

    private double angleOffset;

    private int moduleNumber;

    public SwerveModule(SwerveModuleConstants s) {
        driveMotor = new SparkMax(s.driveMotorID, MotorType.kBrushless);
        angleMotor = new SparkMax(s.angleMotorID, MotorType.kBrushless);
        SparkBaseConfig driveMotorConfig = new SparkMaxConfig();
        SparkBaseConfig angleMotorConfig = new SparkMaxConfig();
        driveMotorConfig.smartCurrentLimit(40, 40);
        driveMotorConfig.disableFollowerMode();
        driveMotorConfig.inverted(true);
        driveMotorConfig.idleMode(IdleMode.kBrake);
        angleMotorConfig.apply(driveMotorConfig);

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        Encoder = new AnalogInput(s.cancoderID);

        m_driveEncoder = driveMotor.getEncoder();

        angleOffset = -s.angleOffset;

        moduleNumber = s.cancoderID; // Assuming module number is based on drive motor ID for simplicity

        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        if (RobotState.isTest()) {
            SmartDashboard.putNumber("[Swerve]encoder raw " + moduleNumber, getRawAngle());
        }

    }

    private double encoderValue() {
        var retVal = getRawAngle();
        // SmartDashboard.putNumber("[Swerve]module " + moduleNumber, retVal);
        if (RobotState.isTest()) {
            SmartDashboard.putNumber("[Swerve]encoder raw " + moduleNumber, retVal);
        }

        SmartDashboard.putNumber("[Swerve]encoder " + moduleNumber, (retVal * 1000) / 1000.0);
        SmartDashboard.putNumber("[Swerve]encoder degrees " + moduleNumber, (retVal * (180 / Math.PI) * 1000) / 1000.0);

        retVal = (retVal + angleOffset) % (2.0 * Math.PI); // apply offset for this encoder and map it back onto [0,
                                                           // 2pi]
        // might need this so we're in the same range as the pid controller is
        // expecting.
        // retVal = retVal - Math.PI;
        if (RobotState.isTest()) {
            SmartDashboard.putNumber("[Swerve]encoder adjusted " + moduleNumber, retVal);
        }

        SmartDashboard.putNumber("[Swerve]EncoderValue() " + moduleNumber, retVal);
        return (retVal);
    }

    private double getRawAngle() {
        var retVal = Encoder.getVoltage() / RobotController.getVoltage5V(); // convert voltage to %
        retVal = 2.0 * Math.PI * retVal; // get % of circle encoder is reading
        return retVal;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        var s = getConvertedVelocity();
        return new SwerveModuleState(
                s, new Rotation2d(encoderValue()));
    }

    private double getConvertedVelocity() {
        return (m_driveEncoder.getVelocity() / (60.0 * SwerveConstants.gearboxRatio))
                * ((SwerveConstants.kWheelRadius * 2) * Math.PI);
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // encode is % rotations
        var retVal = 1
                * ((m_driveEncoder.getPosition() / SwerveConstants.gearboxRatio) * (SwerveConstants.kWheelRadius * 2)
                        * Math.PI); // distance
        // in
        // whatever
        // units
        // the
        // wheel
        // diameter
        // is
        // KB ^^^^ This is from 1 meter testing dont move/change
        return new SwerveModulePosition(retVal, new Rotation2d(encoderValue()));
    }

    public void SetDesiredState(SwerveModuleState desiredState) {
        SmartDashboard.putNumber("[Swerve]Pre Optimize angle target degrees " + moduleNumber,
                desiredState.angle.getDegrees());
        // Optimize the reference state to avoid spinning further than 90 degrees
        SmartDashboard.putNumber("[Swerve]turn encoder" + moduleNumber, encoderValue());

        @SuppressWarnings("deprecation")
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(encoderValue()));

        SmartDashboard.putNumber("[Swerve]After Optimize angle target degrees " + moduleNumber,
                state.angle.getDegrees());

        final double driveOutput = drivingPidController.calculate(m_driveEncoder.getVelocity(),
                state.speedMetersPerSecond);

        final double turnOutput = turningPidController.calculate(encoderValue(), state.angle.getRadians());
        // final double turnOutput = Math.min (Math.max
        // (turningPidController.calculate(encoderValue(), state.angle.getRadians());
        // SmartDashboard.putNumber("[Swerve]pid " + moduleNumber, turnOutput);

        SmartDashboard.putNumber("[Swerve]Setpoint velocity", turningPidController.getSetpoint().velocity);

        driveMotor.set(state.speedMetersPerSecond);

        angleMotor.set((turnOutput / SwerveConstants.kModuleMaxAngularVelocity));

        SmartDashboard.putNumber("[Swerve]m_driveMotor set " + moduleNumber,
                state.speedMetersPerSecond / SwerveConstants.MaxMetersPersecond);
        SmartDashboard.putNumber("[Swerve]m_turningMotor set " + moduleNumber,
                turnOutput / SwerveConstants.kModuleMaxAngularVelocity);

        SmartDashboard.putNumber("[Swerve]m_driveMotor actual" + moduleNumber, getConvertedVelocity());
        SmartDashboard.putNumber("[Swerve]m_turningMotor actual" + moduleNumber, angleMotor.get());

        SmartDashboard.putNumber("[Swerve]drive encoder" + moduleNumber, angleMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("[Swerve]turn encoder" + moduleNumber, encoderValue());

        if (RobotState.isTest()) {
            SmartDashboard.putNumber("[Swerve]turnOutput", turnOutput);
            // SmartDashboard.putNumber("[Swerve]Drive", ((driveOutput + driveFeedforward)
            // /2.1) /2);
            // SmartDashboard.putNumber("[Swerve]Turning stuff", Math.max(turnOutput,
            // turnFeedforward));
            // SmartDashboard.putNumber("[Swerve]Turning stuff", turnOutput +
            // turnFeedforward);
            SmartDashboard.putNumber("[Swerve]target " + moduleNumber, state.angle.getRadians());
        }
    }
}
