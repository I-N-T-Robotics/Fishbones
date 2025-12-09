package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight {

    private Limelight3A limelight;
    private LLResult result;
    private double dist;
    private List<LLResultTypes.FiducialResult> fiducialResults;
    private int tagId;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50);
        limelight.start();

        this.result = limelight.getLatestResult();

        this.dist = result.getBotposeAvgDist();

        this.fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            this.tagId = fr.getFiducialId();
        }
    }

    public double getDistance() {
        if (!Double.isFinite(dist) || dist <= 0) {
            dist = 24;
        }
        return dist;
    }

    public int getTagID() {
        return tagId;
    }
}