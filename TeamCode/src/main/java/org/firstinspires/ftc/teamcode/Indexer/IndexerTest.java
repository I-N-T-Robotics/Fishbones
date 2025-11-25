package org.firstinspires.ftc.teamcode.Indexer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IndexerTest {
    private final DcMotorEx intakeToSystem, topFeed;
    private final CRServo bottomMove;

    public IndexerTest(HardwareMap hardwareMap) {
        intakeToSystem = hardwareMap.get(DcMotorEx.class, "intakeToSystem");
        topFeed = hardwareMap.get(DcMotorEx.class, "topFeed");

        bottomMove = hardwareMap.get(CRServo.class, "bottomMove");
    }

    public void frontToBack(double x) {
        bottomMove.setPower(x);
    }

    public void moveToTop(double y) {
        topFeed.setPower(y);
    }

    public void intakeToSystem(double ry) {
        intakeToSystem.setPower(ry);
    }

    public double intakeToSystem() {
        return intakeToSystem.getPower();
    }

    public double topFeedSpeed() {
        return topFeed.getPower();
    }

    public double bottomMoveSpeed() {
        return bottomMove.getPower();
    }
}