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
        cataRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        cataLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cataLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cataLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setShoot(int pos) {
        cataRight.setTargetPosition(-pos);
        cataRight.setPower(-1);
        cataLeft.setTargetPosition(pos);
        cataLeft.setPower(1);

        //if past pos-10
        //zero power this way zero power break launches
    }

    public void setStow(int pos) {
        cataRight.setTargetPosition(-pos);
        cataRight.setPower(-1);
        cataLeft.setTargetPosition(pos);
        cataLeft.setPower(1);
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