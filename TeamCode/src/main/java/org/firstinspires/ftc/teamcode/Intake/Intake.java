package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private CRServo frontIntake, backIntake;
    public boolean isToggled;

    public Intake(HardwareMap hardwareMap) {
        frontIntake = hardwareMap.get(CRServo.class, "frontIntake");
        backIntake = hardwareMap.get(CRServo.class, "backIntake");

        isToggled = false;
    }

    public void intake() {
        frontIntake.setPower(1);
        backIntake.setPower(1);
    }

    public void intakeStop() {
        frontIntake.setPower(0);
        backIntake.setPower(0);
    }

    public void swapToggle() {
        isToggled = !isToggled;
    }
}
