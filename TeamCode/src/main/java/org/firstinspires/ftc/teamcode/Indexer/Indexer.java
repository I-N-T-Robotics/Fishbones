package org.firstinspires.ftc.teamcode.Indexer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Indexer {
    private final DcMotorEx shootFeed, topFeed;
    private final CRServo bottomMoveFront, bottomMoveBack;

    public Indexer(HardwareMap hardwareMap) {
        shootFeed = hardwareMap.get(DcMotorEx.class, "shootFeed");
        topFeed = hardwareMap.get(DcMotorEx.class, "topFeed");

        bottomMoveFront = hardwareMap.get(CRServo.class, "bottomMoveFront");
        bottomMoveBack = hardwareMap.get(CRServo.class, "bottomMoveBack");
    }
}