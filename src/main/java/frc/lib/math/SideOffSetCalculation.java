package frc.lib.math;

import edu.wpi.first.math.geometry.Pose2d;

public class SideOffSetCalculation {

    public SideOffSetCalculation() {
        // Initialization code here
    }

    public Pose2d calculate(Pose2d side, int sideNumber) {
        // Code to calculate the side of the AprilTag
        if(sideNumber == 0) {
            return new Pose2d(2.372, 3.916, side.getRotation());
        } else if (sideNumber == 1) {
            return new Pose2d(3.419, 5.491, side.getRotation());
        } else if (sideNumber == 2) {
            return new Pose2d(5.563, 5.791, side.getRotation());
        } else if (sideNumber == 3) {
            return new Pose2d(6.206, 4.072, side.getRotation());
        } else if (sideNumber == 4) {
            return new Pose2d(5.377, 2.549, side.getRotation());
        } else if (sideNumber == 5) {
            return new Pose2d(3.470, 2.466, side.getRotation());
        }

        return side;
    }
}
