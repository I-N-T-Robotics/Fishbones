//package org.firstinspires.ftc.teamcode.Vision;
//
//import static org.firstinspires.ftc.teamcode.Constants.Settings.VisionConstants.LIMELIGHT_NAME;
//
//import org.firstinspires.ftc.teamcode.Constants.Settings;
//import org.firstinspires.ftc.teamcode.Swerve.SwerveDrive;
//import org.firstinspires.ftc.teamcode.Util.VisionUtil.LimelightHelpers;
//import org.firstinspires.ftc.teamcode.Util.VisionUtil.VisionData;
//
//import java.util.ArrayList;
//import java.util.Optional;
//
//import edu.wpi.first.math.Matrix;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.numbers.N1;
//import edu.wpi.first.math.numbers.N3;
//import edu.wpi.first.units.measure.AngularVelocity;
//
//public class AprilTagVision {
//
//    public LimelightHelpers.PoseEstimate lastEstimate = new LimelightHelpers.PoseEstimate();
//
//    private boolean currentUpdate = false;
//
//    private boolean useMT2 = false;
//
//    Pose2d currentPose = new Pose2d();
//
//    public void setMegaTag2(boolean useMT2) {
//        this.useMT2 = useMT2;
//    }
//
//    public boolean getMegaTag2() {
//        return useMT2;
//    }
//
//    public Matrix<N3, N1> getSTD() {
//        if (getMegaTag2()) {
//            return Settings.VisionConstants.MT2_STD;
//        } else {
//            return Settings.VisionConstants.MT1_STD;
//        }
//    }
//
//    public double[] getDeviations() {
//        return getSTD().getData();
//    }
//
//    public double[] getTargetMetrics() {
//        return LimelightHelpers.getT2DArray(LIMELIGHT_NAME[0]);
//    }
//
//    public double getRange() {
//        return LimelightHelpers.getTY(LIMELIGHT_NAME[0]);
//    }
//
//    public double getYaw() {
//        return LimelightHelpers.getTX(LIMELIGHT_NAME[0]);
//    }
//
//    public LimelightHelpers.PoseEstimate[] getPoseEstimates() {
//        return new LimelightHelpers.PoseEstimate[] {lastEstimate};
//    }
//
//    public boolean rejectUpdate(LimelightHelpers.PoseEstimate estimate, AngularVelocity angularVelocity) {
//        if (angularVelocity.compareTo(Settings.VisionConstants.MAX_ANGULAR_VELOCITY) > 0) {
//            return true;
//        }
//
//        if (estimate.tagCount == 0) {
//            return true;
//        }
//
//        if (estimate.tagCount == 1 && estimate.avgTagArea > Settings.VisionConstants.AREA_THRESHOLD) {
//            return false;
//        } else if (estimate.tagCount > 1) {
//            return false;
//        }
//
//        return true;
//    }
//
//    public void setCurrentEstimates(AngularVelocity currentAngularVelocity) {
//        LimelightHelpers.PoseEstimate currentEstimate = new LimelightHelpers.PoseEstimate();
//
//        if (useMT2) {
//            currentEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Settings.VisionConstants.LIMELIGHT_NAME[0]);
//        } else  {
//            currentEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(Settings.VisionConstants.LIMELIGHT_NAME[0]);
//        }
//
//        if (currentEstimate != null && !rejectUpdate(currentEstimate, currentAngularVelocity)) {
//            lastEstimate = currentEstimate;
//            currentPose = currentEstimate.pose;
//            currentUpdate = true;
//        }
//    }
//
//    public static ArrayList<VisionData> getOutputs() {
//        ArrayList<VisionData> outputs = new ArrayList<>();
//
//        AngularVelocity angularVelocity = SwerveDrive.getInstance().getAngularVelocity();
//
//        AprilTagVision vision = new AprilTagVision();
//
//        Optional<LimelightHelpers.PoseEstimate> estimateOpt = vision.updatePoseEstimate(angularVelocity);
//
//        if (estimateOpt.isPresent()) {
//            Optional<VisionData> dataOpt = vision.getVisionData();
//            dataOpt.ifPresent(outputs::add);
//        }
//
//        return outputs;
//    }
//
//    public Optional<LimelightHelpers.PoseEstimate> updatePoseEstimate(AngularVelocity angularVelocity) {
//        setCurrentEstimates(angularVelocity);
//
//        if(!currentUpdate) {
//            return Optional.empty();
//        } else {
//            currentUpdate = false;
//        }
//
//        return Optional.of(lastEstimate); //check?
//    }
//
//    public Optional<VisionData> getVisionData() {
//        double timestampSeconds = lastEstimate.timestampSeconds;
//        double area = lastEstimate.avgTagArea;
//        int[] ids = new int[]{lastEstimate.tagCount};
//
//        VisionData data = new VisionData(currentPose, ids, timestampSeconds, area);
//
//        return Optional.of(data);
//    }
//}

package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.Constants.Settings.VisionConstants.LIMELIGHT_NAME;

import org.firstinspires.ftc.teamcode.Constants.Settings;
import org.firstinspires.ftc.teamcode.Util.VisionUtil.LimelightHelpers;
import org.firstinspires.ftc.teamcode.Util.VisionUtil.VisionData;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class AprilTagVision {

    // Last valid pose estimate
    private LimelightHelpers.PoseEstimate lastEstimate = new LimelightHelpers.PoseEstimate();
    private Pose2d currentPose = new Pose2d();

    private boolean useMT2 = false;
    private boolean currentUpdate = false;

    public void setMegaTag2(boolean useMT2) {
        this.useMT2 = useMT2;
    }

    public boolean getMegaTag2() {
        return useMT2;
    }

    public double[] getDeviations() {
        return getSTD();
    }

    private double[] getSTD() {
        if (getMegaTag2()) {
            return Settings.VisionConstants.MT2_STD.getData();
        } else {
            return Settings.VisionConstants.MT1_STD.getData();
        }
    }

    public double[] getTargetMetrics() {
        return LimelightHelpers.getT2DArray(LIMELIGHT_NAME[0]);
    }

    public double getRange() {
        return LimelightHelpers.getTY(LIMELIGHT_NAME[0]);
    }

    public double getYaw() {
        return LimelightHelpers.getTX(LIMELIGHT_NAME[0]);
    }

    private boolean rejectUpdate(LimelightHelpers.PoseEstimate estimate, AngularVelocity angularVelocity) {
        if (angularVelocity.compareTo(Settings.VisionConstants.MAX_ANGULAR_VELOCITY) > 0) {
            return true;
        }

        if (estimate.tagCount == 0) {
            return true;
        }

        if (estimate.tagCount == 1 && estimate.avgTagArea > Settings.VisionConstants.AREA_THRESHOLD) {
            return false;
        } else if (estimate.tagCount > 1) {
            return false;
        }

        return true;
    }

    private void setCurrentEstimates(AngularVelocity currentAngularVelocity) {
        LimelightHelpers.PoseEstimate currentEstimate;

        if (useMT2) {
            currentEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME[0]);
        } else {
            currentEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME[0]);
        }

        if (currentEstimate != null && !rejectUpdate(currentEstimate, currentAngularVelocity)) {
            lastEstimate = currentEstimate;
            currentPose = currentEstimate.pose;
            currentUpdate = true;
        }
    }

    public Optional<VisionData> getVisionData(AngularVelocity currentAngularVelocity) {
        setCurrentEstimates(currentAngularVelocity);

        if (!currentUpdate) return Optional.empty();

        currentUpdate = false;

        double timestampSeconds = lastEstimate.timestampSeconds;
        double area = lastEstimate.avgTagArea;
        int[] ids = new int[]{lastEstimate.tagCount};

        VisionData data = new VisionData(currentPose, ids, timestampSeconds, area);
        return Optional.of(data);
    }

    public ArrayList<VisionData> getOutputs(AngularVelocity currentAngularVelocity) {
        ArrayList<VisionData> outputs = new ArrayList<>();

        Optional<VisionData> dataOpt = getVisionData(currentAngularVelocity);
        dataOpt.ifPresent(outputs::add);

        return outputs;
    }
}