package org.firstinspires.ftc.teamcode.Util.VisionUtil;

import org.firstinspires.ftc.teamcode.Constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
public class VisionData {

    private final Pose2d outputPose;
    private final int[] ids;
    private final double timestamp;
    private final double area;

    public VisionData(Pose2d outputPose, int[] ids, double timestamp, double area) {
        this.outputPose = outputPose;
        this.ids = ids;
        this.timestamp = timestamp;
        this.area = area;
    }

    public Pose2d getPose() {
        return outputPose;
    }

    public int[] getIDs() {
        return ids;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public int getPrimaryID() {
        return ids.length == 0 ? 1 : ids[0];
    }

    public double getDistanceToPrimaryTag() {
        return outputPose
                .getTranslation()
                .getDistance(Field.getTag(getPrimaryID()).getLocation().getTranslation().toTranslation2d());
    }

    public double getArea() {
        return area;
    }

    public boolean isValidData() {
        for (long id :ids) {
            boolean found = false;
            for (AprilTag tag : Field.APRILTAGS) {
                if (tag.getID() == id) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                return false;
            }
        }

        if(Double.isNaN(outputPose.getX())
        || outputPose.getX() < 0
        || outputPose.getX() > Field.LENGTH)
            return false;

        if(Double.isNaN(outputPose.getY())
                || outputPose.getY() < 0
                || outputPose.getY() > Field.WIDTH)
            return false;

        return true;
    }
}