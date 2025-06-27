package frc.lib.epramotor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Motor extends MotorBase {
    boolean SparkMax = false;

    public Motor(int CANID, MotorType a) {
        super(CANID, a);
        SparkMax = true;
    }

    public Motor(int PWM) {
        super(PWM);
        SparkMax = false;
    }

    public double getPosition() {
        if(SparkMax) {
        return super.sparkMax.getPosition();
        }
        return 0;
    }

    public void set(double value) {
        if(!SparkMax) {
        super.MySpark.set(value);
        } else {
        super.sparkMax.set(value);
        }
    }

    public void stop() {
        if(!SparkMax) {
            super.MySpark.set(0);
            } else {
            super.sparkMax.set(0);
            }
    }

    public void setInverted() {
        if(SparkMax) {
            super.sparkMax.setInverted();
        }
    }

    public void resetEncoder() {
        super.sparkMax.resetEncoder();
    }

    public void setVoltage(double value) {
        if(!SparkMax) {
            super.MySpark.setVoltage(value);
            } else {
            super.sparkMax.setVoltage(value);
            }
    }

    public RelativeEncoder getEncoder() {
        if(SparkMax) {
            return super.sparkMax.getEncoder();
        }
        return null;
    }

    public boolean SetIfBoolean(boolean input, double speed) {
        if(input) {
            set(speed);
        } else {
            return true;
        }
        return false;
    }
}
