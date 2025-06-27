package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.OzzyGrabberConstants.*;

public class OzzyGrabberSubsystem extends SubsystemBase {

    SparkMax intake;
    SparkMax Pose;
    SparkMax kick;

    Timer timer;

    DigitalInput Top;
    DigitalInput bottom;
    DigitalInput kicker;

    public OzzyGrabberSubsystem() {
        intake = new SparkMax(GrabberMotor, MotorType.kBrushless);
        Pose = new SparkMax(PosetionMotor, MotorType.kBrushed);
        kick = new SparkMax(15, MotorType.kBrushed);

        timer = new Timer();

        Top = new DigitalInput(top);
        bottom = new DigitalInput(Bottom);
    }

    private boolean top() {
        return Top.get();
    }

    private boolean bottom() {
        return bottom.get();
    }

    private int IntakePulseCount = 0;

    public void intake() {
        intake.set(IntakeSpeed);
    }

    public void intakePulse() {
        if (IntakePulseCount < 5 && IntakePulseCount >= 0) {
            intake.set(IntakeSpeed);
        } else if (IntakePulseCount >= 5) {
            intake.stopMotor();
            IntakePulseCount = -5;
        } else if (IntakePulseCount < 0) {
            intake.stopMotor();
        }
        IntakePulseCount++;
    }

    public void outake() {
        intake.set(OutakeSpeed);
    }

    public void stop() {
        intake.stopMotor();
    }

    // private boolean Up() {
    // if(!top()) {
    // Pose.set(UpSpeed);
    // return false;
    // } else {
    // Pose.stopMotor();
    // return true;
    // }
    // }

    // private boolean down() {
    // intake.set(IntakeSpeed);
    // if(!bottom()) {
    // Pose.set(DownSpeed);
    // return false;
    // }else {
    // timer.start();
    // Pose.stopMotor();
    // if(!timer.advanceIfElapsed(2)) {
    // return false;
    // } else {
    // intake.stopMotor();
    // timer.reset();
    // return true;
    // }
    // }
    // }

    public void end() {
        intake.stopMotor();
        Pose.stopMotor();
    }

    public void Up() {
        // if(!top()){
        Pose.set(UpSpeed);
        // }
    }

    public void down() {
        // if(!bottom()){
        Pose.set(DownSpeed);
        // }
    }

    public boolean Kick() {
        timer.start();
        if (timer.advanceIfElapsed(.9)) {
            // kick.stopMotor();
            timer.reset();
            timer.restart();
            return true;
        } else {
            // kick.set(0.3);
            return false;
        }
    }

    public void joy(double joy) {
        kick.set(Math.abs(joy));
    }

    public void joy1(double joy) {
        intake.set(joy);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Top", top());
        SmartDashboard.putBoolean("Bottom", bottom());
        SmartDashboard.putNumber("Pose", Pose.get());
    }
}

// package frc.robot.subsystems;

// import java.util.function.BooleanSupplier;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.OzzyGrabberConstants;

// public class OzzyGrabberSubsystem extends SubsystemBase{
// SparkMax GrabberMotor;
// SparkMax PoseMotor;
// DigitalInput AlgeaSensor;
// private long algaeLastSeenTime;
// private AnalogInput analogSensor;

// public OzzyGrabberSubsystem() {
// // Initialization code here
// GrabberMotor = new SparkMax(OzzyGrabberConstants.GrabberMotor,
// MotorType.kBrushless);
// //GrabberMotor = new Motor(OzzyGrabberConstants.GrabberMotor,
// MotorType.kBrushless);
// PoseMotor = new SparkMax(OzzyGrabberConstants.PosetionMotor,
// MotorType.kBrushless);
// AlgeaSensor = new DigitalInput(OzzyGrabberConstants.BeamBreak);
// analogSensor = new AnalogInput(4); //this works but we have no more analog
// ports
// }

// public boolean Grab() {
// // Code to move the elevator
// if(PoseMotor.getEncoder().getPosition() < OzzyGrabberConstants.MiddleLength)
// {
// PoseMotor.set(OzzyGrabberConstants.DownSpeed);
// } else {
// PoseMotor.set(0);
// }
// return PoseMotor.getEncoder().getPosition() ==
// OzzyGrabberConstants.MiddleLength;
// }

// public boolean intake() {
// if(!AlgeaSensor.get()) {
// GrabberMotor.set(OzzyGrabberConstants.IntakeSpeed);
// } else {
// GrabberMotor.stopMotor();
// }
// return AlgeaSensor.get();
// }

// public boolean outake() {
// if(AlgeaSensor.get()) {
// GrabberMotor.set(OzzyGrabberConstants.OutakeSpeed);
// algaeLastSeenTime = System.currentTimeMillis();
// } else {
// if (System.currentTimeMillis() > algaeLastSeenTime + 500) {
// GrabberMotor.stopMotor();
// return true;
// }
// }
// return false;
// }

// public boolean down() {
// if(PoseMotor.getEncoder().getPosition() < OzzyGrabberConstants.MovmentLength)
// {
// PoseMotor.set(OzzyGrabberConstants.DownSpeed);
// return false;
// } else {
// PoseMotor.stopMotor();
// }
// return true;
// }

// public boolean up() {
// if(PoseMotor.getEncoder().getPosition() > 0) {
// PoseMotor.set(OzzyGrabberConstants.UpSpeed);
// return false;
// } else {
// PoseMotor.stopMotor();
// }
// return true;
// }

// public boolean isHangingLoose() {
// return PoseMotor.getEncoder().getPosition() > 0.15 *
// OzzyGrabberConstants.MovmentLength;
// }

// public BooleanSupplier BeamBreak() {
// return () -> AlgeaSensor.get();
// }

// public BooleanSupplier FalseBeamnBreak() {
// return () -> !AlgeaSensor.get();
// }

// @Override
// public void periodic() {
// SmartDashboard.putNumber("Algae Grabber position",
// GrabberMotor.getEncoder().getPosition());
// SmartDashboard.putNumber("Algae Pose position",
// PoseMotor.getEncoder().getPosition());
// SmartDashboard.putBoolean("Algae sensor", AlgeaSensor.get());
// SmartDashboard.putNumber("Algae sensor1", analogSensor.getValue());
// SmartDashboard.putNumber("Algae sensor2", analogSensor.getVoltage());
// SmartDashboard.putNumber("Algae sensor3", analogSensor.getAverageValue());
// }
// }
