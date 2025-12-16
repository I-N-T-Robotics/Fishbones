package org.firstinspires.ftc.teamcode.Util;

public class Normalize {

    private double normalizedV1;
    private double normalizedV2;
    private double normalizedV3;

    public Normalize(float v1, float v2, float v3) {

        float sum = v1 + v2 + v3;

        this.normalizedV1 = v1/sum;
        this.normalizedV2 = v2/sum;
        this.normalizedV3 = v3/sum;
    }

    public double rV1() {
        return normalizedV1;
    }

    public double rV2() {
        return normalizedV2;
    }

    public double rV3() {
        return normalizedV3;
    }
}