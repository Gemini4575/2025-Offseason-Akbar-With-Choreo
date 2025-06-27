package frc.lib.util;

public class AprilTagToSide {
    
    public AprilTagToSide() {
        // Initialization code here
    }

    public int calculate(int tagID) {
        // Code to calculate the side of the AprilTag
        if(tagID == 21 || tagID == 10){
            return 0;
        } else if (tagID == 22 || tagID == 11){
            return 1;
        } else if (tagID == 6 || tagID == 17){
            return 2;
        } else if (tagID == 18 || tagID == 7){
            return 3;
        } else if (tagID == 19 || tagID == 8){
            return 4;
        } else if (tagID == 20 || tagID == 9){
            return 5;
        }
        return 1;
    }
}
