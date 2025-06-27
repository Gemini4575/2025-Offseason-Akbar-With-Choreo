package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class Pose2dToSpeeds {

    public static ChassisSpeeds fromPose2d(Pose2d pose) {
        return new ChassisSpeeds(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }

    public static ChassisSpeeds fromPose3d(Pose3d pose) {
        return new ChassisSpeeds(pose.getX(), pose.getY(), pose.getRotation().getZ());
    }

    public static ChassisSpeeds fromTranslation2d(Translation2d translation) {
        return new ChassisSpeeds(translation.getX(), translation.getY(), 0);
    }

    public static ChassisSpeeds fromTranslation3d(Translation3d translation) {
        return new ChassisSpeeds(translation.getY() / 10, 0/*translation.getX() / 10*/, translation.getZ() / 10);
    }
}