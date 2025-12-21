package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight {

    private final Limelight3A limelight;
    private LLResult result;
    private double dist;
    private int tagId;
    private double xFromTag;
    private double lastDist = 0;
    private double lastTx = 0;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50);
        limelight.start();

        update();
    }

    public void update() {
        result = limelight.getLatestResult();
        if (result != null && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            LLResultTypes.FiducialResult fr = fiducialResults.get(0); // first tag seen
            tagId = fr.getFiducialId();
            dist = result.getBotposeAvgDist();
            xFromTag = result.getTx();
            lastDist = dist;
            lastTx = xFromTag;
        } else {
            tagId = -1; // no tag seen
            dist = 0;   // or some safe default
        }
    }

    public double getDistance() {
        return dist;
    }

    public int getTagID() {
        return tagId;
    }

    public double getxFromTag() {
        return xFromTag;
    }

    public double getLastDist() {
        return lastDist;
    }

    public double getLastTx() {
        return lastTx;
    }
}