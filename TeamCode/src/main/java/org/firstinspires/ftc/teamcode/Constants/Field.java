package org.firstinspires.ftc.teamcode.Constants;

import org.firstinspires.ftc.teamcode.Util.VisionUtil.AprilTag;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public interface Field {

    double WIDTH = Units.inchesToMeters(0);
    double LENGTH = Units.inchesToMeters(0);

    public static Pose3d transformToOppositeAlliance(Pose3d pose) {
        Pose3d rotated = pose.rotateBy(new Rotation3d(0, 0, Math.PI));

        return new Pose3d(
                rotated.getTranslation().plus(new Translation3d(LENGTH, WIDTH, 0)),
                rotated.getRotation());
    }

    public static Pose2d transformToOppositeAlliance(Pose2d pose) {
        Pose2d rotated = pose.rotateBy(Rotation2d.fromDegrees(180));

        return new Pose2d(
                rotated.getTranslation().plus(new Translation2d(LENGTH, WIDTH)),
                rotated.getRotation());
    }

    double APRILTAG_SIZE = Units.inchesToMeters(6.125); //check

    enum NamedTags {
        RED_SCORE,
        BLUE_SCORE,
        MIDDLEPPG,
        MIDDLEPGP,
        MIDDLEGPP;

        public final AprilTag tag;

        public int getID() {
            return tag.getID();
        }

        public Pose3d getLocation() {
            return tag.getLocation();
        }

        private NamedTags() {
            tag = APRILTAGS[ordinal()];
        }
    }

    AprilTag[] APRILTAGS = {
            new AprilTag(1, new Pose3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new AprilTag(2, new Pose3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new AprilTag(3, new Pose3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new AprilTag(4, new Pose3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new AprilTag(5, new Pose3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))))
    };

    public static boolean isValidTag(int id) {
        for (AprilTag tag : APRILTAGS) {
            if(tag.getID() == id) {
                return true;
            }
        }
        return false;
    }

    public static AprilTag[] getApriltagLayout(int... ids) {
        ArrayList<AprilTag> tags = new ArrayList<AprilTag>();

        for (int id : ids) {
            for (AprilTag tag : APRILTAGS) {
                if (tag.getID() == id) {
                    tags.add(tag);
                }
            }
        }
        AprilTag[] tags_array = new AprilTag[tags.size()];
        return tags.toArray(tags_array);
    }

    public static AprilTag getTag(int id) {
        for (AprilTag tag : APRILTAGS) {
            if (tag.getID() == id) {
                return tag;
            }
        }
        return null;
    }

//    public static Pose2d getAllianceScore() {
//        return (Robot.isBlue() ? NamedTags.BLUE_SCORE : NamedTags.RED_SCORE)
//                .getLocation().toPose2d();
//    }
}
