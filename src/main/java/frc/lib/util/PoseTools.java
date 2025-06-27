package frc.lib.util;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class PoseTools {

    public PoseTools() {
        // Initialization code here
    }

    public Translation2d calculate(Translation3d translation3d) {
        // Code to calculate the Translation2d from Translation3d
        return new Translation2d(translation3d.getX(), translation3d.getY());
    }

    public Translation3d calculate(Transform3d cv) {
        return cv.getTranslation();
    }
}