package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Indexer.IndexerTest;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class IndexTestOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        IndexerTest indexer = null;

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            indexer = new IndexerTest(hardwareMap);

            indexer.frontToBack(x); //move bottom horizontal wheel
            indexer.moveToTop(y); //bottom to top compartment
            indexer.intakeToSystem(rx); //intake to bottom

            telemetry.addData("frontToBack", indexer.bottomMoveSpeed());
            telemetry.addData("moveToTop", indexer.topFeedSpeed());
            telemetry.addData("topToFeed", indexer.intakeToSystem());
        }
    }
}