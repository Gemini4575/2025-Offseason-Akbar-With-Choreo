package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.epramotor.Motor;
import static frc.robot.Constants.NickClimbingConstanst.*;
import static frc.robot.Constants.OzzyGrabberConstants.top;

public class NickClimbingSubsystem extends SubsystemBase {

    public Motor ClimbingMotor1;
    public Motor ClimbingMotor2;
    private Motor Flipper;

    public NickClimbingSubsystem() {
        // Initialization code here
        ClimbingMotor1 = new Motor(ClimbingMotor1_CANID, MotorType.kBrushless);
        ClimbingMotor2 = new Motor(ClimbingMotor2_CANID, MotorType.kBrushless);
        Flipper = new Motor(Flipper_CANID, MotorType.kBrushed);

    }

    private boolean Climb1() {
        // Code to move the elevator
        return ClimbingMotor1.SetIfBoolean(
                Math.abs(ClimbingMotor1.getPosition()) < (ClimbingMotorPoseition),
                ClimbingSpeed);
    }

    private boolean Climb2() {
        // Code to move the elevator
        return ClimbingMotor2.SetIfBoolean(
                Math.abs(ClimbingMotor2.getPosition()) < (ClimbingMotorPoseition),
                ClimbingSpeed);
    }

    public boolean init1() {
        // Code to move the elevator
        if (ClimbingMotor1.getPosition() < Math.round(0)) {
            ClimbingMotor1.set(ClimbingSpeed);
        } else {
            Stop();
            return true;
        }
        return false;
    }

    public boolean init2() {
        // Code to move the elevator
        if (ClimbingMotor2.getPosition() < Math.round(ClimbingMotorPoseition)) {
            ClimbingMotor2.set(ClimbingSpeed);
        } else {
            Stop();
            return true;
        }
        return false;
    }

    public boolean Climb() {
        Climb1();
        Climb2();
        return Climb1() && Climb2();
    }

    public void End() {
        ClimbingMotor1.stop();
        ClimbingMotor2.stop();
    }

    /**
     * Curently this is what we are using but after I get mesurments this should not
     * be used 1/28
     * 
     * @param Joy The joysitck that you are using
     */
    @Deprecated
    public void JoyClimb1(double Joy, boolean btuon) {
        ClimbingMotor1.set(Joy);
        if (btuon) {
            ClimbingMotor1.getEncoder().setPosition(0);
        }
    }

    @Deprecated
    public void JoyClimb2(double Joy, boolean btuon) {
        ClimbingMotor2.set(Joy);
        if (btuon) {
            ClimbingMotor2.getEncoder().setPosition(0);
        }
    }

    Timer timer = new Timer();

    public void Flipper(double set) {
        Flipper.set(set);
    }

    public void Stop() {
        // Code to stop the elevator
        ClimbingMotor1.stop();
        ClimbingMotor2.stop();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("CimbEndcoder 1", ClimbingMotor1.getPosition());
        SmartDashboard.putNumber("CimbEndcoder 2", ClimbingMotor2.getPosition());

    }

}
