// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.algea.EXO.OzDown;
import frc.robot.commands.algea.EXO.OzUp;
import frc.robot.commands.climb.Climb;
import frc.robot.commands.coral.lili.AUTOCoral;
import frc.robot.commands.coral.lili.AUTOCoralFalse;
import frc.robot.commands.coral.lili.EXOCloseGate;
import frc.robot.commands.coral.lili.LIPlaceCoral;
import frc.robot.commands.coral.lili.LIPlaceCoralSlow;
import frc.robot.commands.coral.lili.LiAutoPlaceCoral;
import frc.robot.commands.driving.AlineWheels;
import frc.robot.commands.driving.Spin180;
import frc.robot.commands.driving.Stop;
import frc.robot.commands.driving.TeleopSwerve;
import frc.robot.commands.testing.AutoChoreoTesting;
import frc.robot.commands.testing.PathFindToAprilTag;
import frc.robot.subsystems.LiliCoralSubystem;
import frc.robot.subsystems.NickClimbingSubsystem;
import frc.robot.subsystems.OzzyGrabberSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.Vision;

import static frc.robot.Constants.JoystickConstants.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.Choreo;
import choreo.auto.AutoFactory;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  Field2d visionPoseEstimate = new Field2d();

  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick rotator = new Joystick(1);
  private final Joystick operator = new Joystick(2);
  private final Joystick climber = new Joystick(3);
  private final Joystick testing = new Joystick(4);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(testing, BACK_BUTTON);
  private final Trigger Slow = new Trigger(new JoystickButton(driver, 7)
      .and(new JoystickButton(driver, 12)))
      .or(new JoystickButton(operator, START_BUTTON));

  /* Choreo Stuff */

  /* Subsystems */
  private final Drivetrain D = new Drivetrain();
  private final LiliCoralSubystem c = new LiliCoralSubystem();
  private final NickClimbingSubsystem nc = new NickClimbingSubsystem();
  private final OzzyGrabberSubsystem g = new OzzyGrabberSubsystem();
  private final Vision V = new Vision();
  private final VisionSubsystem VS = new VisionSubsystem(V);

  private final Field2d autoRobotPose = new Field2d();
  private final Field2d autoTargetPose = new Field2d();
  private final Field2d autoPath = new Field2d();

  public RobotContainer() {

    configureLogging();

    SmartDashboard.putData("[Robot]Vision Pose Estimate", visionPoseEstimate);
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    /* Driver Controls */
    zeroGyro.onTrue(new InstantCommand(() -> D.ResetGyro()));
    D.setDefaultCommand(
        new TeleopSwerve(
            D,
            () -> -testing.getRawAxis(LEFT_X_AXIS),
            () -> testing.getRawAxis(LEFT_Y_AXIS),
            () -> -testing.getRawAxis(RIGHT_Y_AXIS),
            Slow,
            () -> testing.getPOV()));
    new JoystickButton(testing, RED_BUTTON)
        .onTrue(new Spin180(D).asProxy());
    /* Operator Controls */
    new JoystickButton(operator, JoystickConstants.BLUE_BUTTON)
        .onTrue(new LIPlaceCoral(c));
    new JoystickButton(operator, JoystickConstants.GREEN_BUTTON)
        .whileTrue(new OzDown(g));
    new JoystickButton(operator, YELLOW_BUTTON)
        .whileTrue(new OzUp(g));

    /* Testing */

    System.out.println("Ended configureBindings()");
  }

  public void Periodic() {
    updateVisionEst();
  }

  public void teleopPeriodic() {
    if (operator.getRawButton(LEFT_BUMPER)) {
      g.intakePulse();
    } else if (operator.getRawButton(RIGHT_BUMPER)) {
      g.outake();
    } else {
      g.stop();
    }
    // c.JoyControll(operator.getRawAxis(JoystickConstants.LEFT_Y_AXIS));
    g.joy(MathUtil.applyDeadband(operator.getRawAxis(JoystickConstants.LEFT_Y_AXIS), 0.5) * 1);
    // g.joy1(MathUtil.applyDeadband(climber.getRawAxis(JoystickConstants.LEFT_Y_AXIS),
    // 0.2));
    if (climber.getRawButton(GREEN_BUTTON)) {
      nc.JoyClimb1(-1, false);
      nc.JoyClimb2(-1, false);
    } else {
      nc.JoyClimb1(MathUtil.applyDeadband(climber.getRawAxis(JoystickConstants.RIGHT_Y_AXIS), 0.5),
          climber.getRawButton(JoystickConstants.START_BUTTON));
      nc.JoyClimb2(MathUtil.applyDeadband(climber.getRawAxis(JoystickConstants.LEFT_Y_AXIS), 0.5),
          climber.getRawButton(JoystickConstants.BACK_BUTTON));
    }

    if (climber.getPOV() == 0) {
      nc.Flipper(-1);
    } else if (climber.getPOV() == 180) {
      nc.Flipper(1);
    } else {
      nc.Flipper(0);
    }

  }

  private void configureLogging() {
    SmartDashboard.putData("[Robot]Auto Robot Pose", autoRobotPose);
    SmartDashboard.putData("[Robot]Auto Target Pose", autoTargetPose);
    SmartDashboard.putData("[Robot]Auto Path", autoPath);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      autoRobotPose.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      autoTargetPose.setRobotPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      autoPath.getObject("Path").setPoses(poses);
    });
  }

  private void updateVisionEst() {
    var visionEst = V.getEstimatedGlobalPose();
    updateLocationWithVision(visionEst);
    var visionEstColor = V.getEstimatedGlobalPoseColor();
    updateLocationWithVision(visionEstColor);
  }

  private void updateLocationWithVision(Optional<EstimatedRobotPose> visionEst) {
    visionEst.ifPresent(
        est -> {
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = V.getEstimationStdDevs();

          D.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

          visionPoseEstimate.setRobotPose(est.estimatedPose.toPose2d());
        });
  }

  /**
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new AutoChoreoTesting(D);
  }
}
