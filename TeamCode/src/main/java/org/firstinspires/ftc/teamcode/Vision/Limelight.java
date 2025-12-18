package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight {

    private final Limelight3A limelight;

    private LLResult result;
    private double distance = 0.0;
    private int tagId = -1;
    private boolean hasTarget = false;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50);
        limelight.start();
    }

    public void update() {
        result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            hasTarget = false;
            distance = 0.0;
            tagId = -1;
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        if (fiducials.isEmpty()) {
            hasTarget = false;
            distance = 0.0;
            tagId = -1;
            return;
        }

        LLResultTypes.FiducialResult tag = fiducials.get(0);

        hasTarget = true;
        tagId = tag.getFiducialId();
        distance = result.getBotposeAvgDist();
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public double getDistance() {
        return distance;
    }

    public int getTagId() {
        return tagId;
    }
}