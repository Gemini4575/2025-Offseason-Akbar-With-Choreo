package frc.lib.util;

import static frc.robot.Constants.Vision.ALGAE_CAM_AREA;
import static frc.robot.Constants.Vision.ALGAE_CAM_FOCAL_LENGTH;
import static frc.robot.Constants.Vision.ALGAE_REAL_DIAMETER;

public class GeometryHelper {

    public static double distanceToAlgae(double apparentAreaPct) {
        // convert area to diameter
        double diameter = 2.0 * Math.sqrt(ALGAE_CAM_AREA * apparentAreaPct/(100.0 * Math.PI));
        // distance = (real size * focal length) / (percieved size)
        return ALGAE_REAL_DIAMETER * ALGAE_CAM_FOCAL_LENGTH / diameter;
    }

}
