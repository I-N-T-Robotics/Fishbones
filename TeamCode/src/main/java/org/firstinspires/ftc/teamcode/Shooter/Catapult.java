package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Catapult {
    private final DcMotorEx cataRight, cataLeft;

    public Catapult(HardwareMap hardwareMap) {
        cataRight = hardwareMap.get(DcMotorEx.class, "cataRight");
        cataLeft = hardwareMap.get(DcMotorEx.class, "cataLeft");

        cataRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cataRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        cataLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cataLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setShoot() {
        cataRight.setPower(-1);
        cataLeft.setPower(1);
    }

    public void setNothing() {
        cataRight.setPower(0);
        cataLeft.setPower(0);
    }

    public void setHold() {
        cataRight.setPower(1);
        cataLeft.setPower(-1);
    }

    public double getCataREncoder() {
        return cataRight.getCurrentPosition();
    }

    public double getCataLEncoder() {
        return cataLeft.getCurrentPosition();
    }

    public double getCataRVel() {
        return cataRight.getVelocity();
    }

    public double getCataLVel() {
        return cataLeft.getVelocity();
    }
}