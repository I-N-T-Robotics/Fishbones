package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoRed", group = "RED")
public class AutoRed extends LinearOpMode {
    DcMotorEx FrontRightDrive, FrontLeftDrive, BackRightDrive, BackLeftDrive;
    DcMotorEx CataRight, CataLeft;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        FrontRightDrive = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        FrontLeftDrive = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        BackRightDrive = hardwareMap.get(DcMotorEx.class, "BackRightDrive");
        BackLeftDrive = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");

        FrontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        CataRight = hardwareMap.get(DcMotorEx.class, "cataRight");
        CataLeft = hardwareMap.get(DcMotorEx.class, "cataLeft");

        CataLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        timer.reset();
        while(timer.time() <= 0.1) {
            FrontRightDrive.setPower(0);
            FrontLeftDrive.setPower(0);
            BackRightDrive.setPower(0);
            BackLeftDrive.setPower(0);

            CataRight.setPower(0);
            CataLeft.setPower(0);
        }

        timer.reset();
        while(timer.time() <= 0.5) {
            FrontRightDrive.setPower(-1);
            FrontLeftDrive.setPower(-1);
            BackRightDrive.setPower(-1);
            BackLeftDrive.setPower(-1);

            CataRight.setPower(-1);
            CataLeft.setPower(-1);
        }

        timer.reset();
        while(timer.time() <= 1) {
            CataRight.setPower(1);
            CataLeft.setPower(1);
        }

        timer.reset();
        while(timer.time() <= 0) {
            FrontRightDrive.setPower(-1);
            FrontLeftDrive.setPower(-1);
            BackRightDrive.setPower(-1);
            BackLeftDrive.setPower(-1);
        }

        timer.reset();
        while(timer.time() <= 0.8) {
            FrontRightDrive.setPower(-1);
            BackRightDrive.setPower(-1);

            FrontLeftDrive.setPower(1);
            BackLeftDrive.setPower(1);
        }

        timer.reset();
        while(timer.time() <= 1) {
            FrontRightDrive.setPower(-1);
            FrontLeftDrive.setPower(-1);
            BackRightDrive.setPower(-1);
            BackLeftDrive.setPower(-1);
        }
    }
}