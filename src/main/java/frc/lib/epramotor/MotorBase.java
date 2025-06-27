package frc.lib.epramotor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class MotorBase {
    public class MySparkMax extends SparkMax{
        SparkMaxConfig sc;
        boolean follower = false;
        public MySparkMax(int CANID, MotorType a) {
            super(CANID, a);
            sc = new SparkMaxConfig();
        }
        /**
         * This is rounded use {@code getEncoder().getPosition()} to get the actual poseition
         * @return The ROUNDED postion of the motor
         */
        public double getPosition() {
            return Math.round(super.getEncoder().getPosition());
        }

        @Override
        public void set(double value) {
            if(!super.isFollower()) {
                super.pauseFollowerMode();
                super.set(value);
            }
            super.set(value);
        }

        public void stop() {
            super.set(0);
        }
        
        public void setInverted() {
            sc.inverted(true);
            super.configure(sc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        public void resetEncoder(){
            super.getEncoder().setPosition(0);
        }

        @Override
        public void setVoltage(double value) {
            super.setVoltage(value);
        }

        public RelativeEncoder getEncoder() {
            return super.getEncoder();
        }

        public void TraceDump() {
            
        }
    
    }

    public class MySpark extends Spark {
            
        public MySpark(int PWM) {
            super(PWM);
        }

        public void stop() {
            super.set(0);
        }
        
    }
    public MySparkMax sparkMax;
    public MySpark MySpark;
    public MotorBase(int CANID, MotorType a) {
        sparkMax = new MySparkMax(CANID, a);
    }

    public MotorBase(int PWM) {
        MySpark = new MySpark(PWM);
    }
}

