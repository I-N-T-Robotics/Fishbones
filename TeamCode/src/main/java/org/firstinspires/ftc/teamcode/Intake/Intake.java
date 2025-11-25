package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

public class Intake {
    private final CRServo frontIntakeClose, frontIntakeFar;
    private final CRServo backIntakeClose, backIntakeFar;
    public boolean isToggled;

    private final DcMotorEx botToTop, topToShoot;
    private final CRServo ballMove;

    private final ColorSensor frontSense, backSense;
    private String[] pattern;

    private int rightCount = 0;
    private int wrongCount = 0;
    private int pieceCount = 0;

    private States state;

    private enum States {
        NOTHING,
        r,
        rr,
        rrr,
        rrw,
        rw,
        rwr,
        rww,
        w,
        wr,
        wrr,
        ww
    }

    public Intake(HardwareMap hardwareMap) {
        frontIntakeClose = hardwareMap.get(CRServo.class, "frontIntakeClose");
        frontIntakeFar = hardwareMap.get(CRServo.class, "frontIntakeFar");

        backIntakeClose = hardwareMap.get(CRServo.class, "backIntakeClose");
        backIntakeFar = hardwareMap.get(CRServo.class, "backIntakeFar");

        isToggled = false;

        botToTop = hardwareMap.get(DcMotorEx.class, "botToTop");
        topToShoot = hardwareMap.get(DcMotorEx.class, "topToShoot");
        ballMove = hardwareMap.get(CRServo.class, "ballMove");

        frontSense = hardwareMap.get(ColorSensor.class, "frontSensor");
        backSense = hardwareMap.get(ColorSensor.class, "backSensor");

        pattern = new String[3];

        String fileName = "patternTagID.txt";
        try (BufferedReader reader = new BufferedReader(new FileReader(fileName))) {
            String patternID;
            while ((patternID = reader.readLine()) != null) {
                if (Integer.parseInt(patternID) == 21) {
                    String[] pattern = new String[] {"Green", "Purple", "Purple"};
                } else if (Integer.parseInt(patternID) == 22) {
                    String[] pattern = new String[] {"Purple", "Green", "Purple"};
                } else if (Integer.parseInt(patternID) == 23) {
                    String[] pattern = new String[] {"Purple", "Purple", "Green"};
                }
            }
        } catch (IOException e) {
            System.err.println("Error reading from file: " + e.getMessage()); //stop turret? fix
        }

        States state = States.NOTHING;
    }

    public void frontIntake() {
        frontIntakeClose.setPower(1);
        frontIntakeFar.setPower(1);
    }

    public void frontIntakeStop() {
        frontIntakeClose.setPower(0);
        frontIntakeFar.setPower(0);
    }

    public void backIntake() {
        backIntakeClose.setPower(1);
        backIntakeFar.setPower(1);
    }

    public void backIntakeStop() {
        backIntakeClose.setPower(0);
        backIntakeFar.setPower(0);
    }

    public void swapToggle() {
        isToggled = !isToggled;
    }

    public int getFrontColor() {
        return frontSense.green();
    }

    public int getFrontIntaked() {
        return frontSense.argb();
    }

    public int getBackColor() {
        return backSense.green();
    }

    public int getBackIntaked() {
        return backSense.argb();
    }

//    public boolean getRight() {
//        if (getFrontColor() > 200 || getBackColor() > 200 && )
//    }

//    public void stateHandler() {
//        switch (state) {
//            case NOTHING:
//                if ()
//        }
//    }
}
