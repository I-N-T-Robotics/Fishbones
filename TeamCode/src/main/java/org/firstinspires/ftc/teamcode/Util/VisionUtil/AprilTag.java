package org.firstinspires.ftc.teamcode.Util.VisionUtil;

import org.firstinspires.ftc.teamcode.Constants.Field;

import edu.wpi.first.math.geometry.Pose3d;

public class AprilTag {

    private final int id;
    private final Pose3d blueLocation;

    public AprilTag(int id, Pose3d blueLocation) {
        this.id = id;
        this.blueLocation = blueLocation;
    }

    public int getID() {
        return id;
    }

    public Pose3d getLocation() {
        if (/*Robot.isBlue()*/ true) { //fix
            return blueLocation;
        } else {
            return Field.transformToOppositeAlliance(blueLocation);
        }
    }
}
