package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Objects;

public class Intake {
    private final CRServo frontIntakeRight, frontIntakeLeft;
    private final CRServo backIntakeRight, backIntakeLeft;
    public boolean isToggledIntake;
    public boolean isToggledPattern;

    private final DcMotorEx botToTop, topToShoot;
    private final CRServo ballMoveFront, ballMoveBack;

    private final ColorSensor frontSense, backSense;
    private String[] pattern;

    private int rightCount = 0;
    private int wrongCount = 0;
    private int pieceCount = 0;
    private int greenCount = 0;

    private States state;

    private ElapsedTime timer = new ElapsedTime();

    private enum States {
        OFF,
        NOTHING,
        R,
        RR,
        RRR,
        RRW,
        RW,
        RWR,
        RWW,
        W,
        WR,
        WW
    }

    public Intake(HardwareMap hardwareMap) {
        frontIntakeRight = hardwareMap.get(CRServo.class, "frontIntakeRight");
        frontIntakeLeft = hardwareMap.get(CRServo.class, "frontIntakeLeft");

        backIntakeRight = hardwareMap.get(CRServo.class, "backIntakeRight");
        backIntakeLeft = hardwareMap.get(CRServo.class, "backIntakeLeft");

        isToggledIntake = false;
        isToggledPattern = false;

        botToTop = hardwareMap.get(DcMotorEx.class, "botToTop");
        topToShoot = hardwareMap.get(DcMotorEx.class, "topToShoot");
        ballMoveFront = hardwareMap.get(CRServo.class, "ballMoveFront");
        ballMoveBack = hardwareMap.get(CRServo.class, "ballMoveBack");

        frontSense = hardwareMap.get(ColorSensor.class, "frontSensor");
        backSense = hardwareMap.get(ColorSensor.class, "backSensor");

        String fileName = "patternTagID.txt";
        try (BufferedReader reader = new BufferedReader(new FileReader(fileName))) {
            String patternID;
            while ((patternID = reader.readLine()) != null) {
                if (Integer.parseInt(patternID) == 21) {
                    pattern = new String[]{"Green", "Purple", "Purple"};
                } else if (Integer.parseInt(patternID) == 22) {
                    pattern = new String[]{"Purple", "Green", "Purple"};
                } else if (Integer.parseInt(patternID) == 23) {
                    pattern = new String[]{"Purple", "Purple", "Green"};
                }
            }
        } catch (IOException e) {
            System.err.println("Error reading from file: " + e.getMessage()); //stop turret? fix
        } //volatile global enum for pattern saves between auto and teleOp

        state = States.OFF;
    }

    public void frontIntake() {
        frontIntakeRight.setPower(1);
        frontIntakeLeft.setPower(1);
    }

    public void frontIntakeEject() {
        frontIntakeRight.setPower(-1);
        frontIntakeLeft.setPower(-1);
    }

    public void frontIntakeStop() {
        frontIntakeRight.setPower(0);
        frontIntakeLeft.setPower(0);
    }

    public void backIntake() {
        backIntakeRight.setPower(1);
        backIntakeLeft.setPower(1);
    }

    public void backIntakeEject() {
        backIntakeRight.setPower(-1);
        backIntakeLeft.setPower(-1);
    }

    public void backIntakeStop() {
        backIntakeRight.setPower(0);
        backIntakeLeft.setPower(0);
    }

    public void centerHold() {
        ballMoveFront.setPower(1);
        ballMoveBack.setPower(-1);
    }

    public void centerStop() {
        ballMoveFront.setPower(0);
        ballMoveBack.setPower(0);
    }

    public void swapIntakeToggle() {
        isToggledIntake = !isToggledIntake;
    }

    public void swapPatternToggle() {
        isToggledPattern = !isToggledPattern;
    }

    public int getFrontColor() {
        return frontSense.green();
    }

    public int fromFront() {
        return frontSense.argb();
    }

    public int getBackColor() {
        return backSense.green();
    }

    public int fromBack() {
        return backSense.argb();
    }

    public void frontToMidShuffle() {

    }

    public void getRight() {
        if ((getFrontColor() > 200 || getBackColor() > 200) && Objects.equals(pattern[rightCount], "Green")) {
            rightCount++;
            pieceCount++;
            greenCount++;
        } else if ((getFrontColor() < 50 || getBackColor() < 50) && Objects.equals(pattern[rightCount], "Purple")) {
            rightCount++;
            pieceCount++;
        } else {
            wrongCount++;
            pieceCount++;
        }
    }

    public void stateHandler() {
        if (!isToggledPattern) {
            switch (state) {
                case OFF:
                    pieceCount = 0;
                    getRight();
                    if (pieceCount == 1) {
                        centerHold();
                        botToTop.setPower(1);
                    } else if (pieceCount == 2) {
                        centerHold();
                        botToTop.setPower(0);
                    } else if (pieceCount == 3) {
                        timer.reset();
                        if (timer.time() > 0.3) {
                            swapIntakeToggle();
                        }
                    }
                    break;
            }
        } else if (isToggledPattern) {
            switch (state) {
                case NOTHING:
                    rightCount = 0;
                    wrongCount = 0;
                    pieceCount = 0;

                    centerHold();
                    botToTop.setPower(1);
                    getRight();
                    if (rightCount == 1) {
                        state = States.R;
                    } else if (wrongCount == 1) {
                        state = States.W;
                    } else {
                        state = States.NOTHING;
                    }
                    break;

                case R:
                    centerHold();
                    botToTop.setPower(1);
                    getRight();
                    if (rightCount == 2) {
                        state = States.RR;
                    } else if (wrongCount == 1) {
                        state = States.RW;
                    } else {
                        state = States.R;
                    }
                    break;

                case RR:
                    centerHold();
                    botToTop.setPower(0);
                    getRight();
                    if (rightCount == 3) {
                        state = States.RRR;
                    } else if (wrongCount == 1) {
                        state = States.RRW;
                    } else {
                        state = States.RR;
                    }
                    break;

                case RRR:
                    timer.reset();
                    if (timer.time() > 0.3) {
                        swapIntakeToggle();
                    }
                    centerStop();
                    break;

                case RRW:
                    centerHold();
                    if (greenCount == 2 && Objects.equals(pattern[0], "Green")) {
                        timer.reset();
                        if (timer.time() < 0.3) {
                            topToShoot.setPower(1);
                        } else {
                            if (fromFront() != 0) {
                                timer.reset();
                                if (timer.time() < 0.3) {
                                    ballMoveFront.setPower(1);
                                    ballMoveBack.setPower(1);
                                } else if (timer.time() > 0.3 && timer.time() < 0.6) {
                                    centerHold();
                                    botToTop.setPower(1);
                                } else {
                                    centerHold();
                                    state = States.RR;
                                }
                            } else if (fromBack() != 0) {
                                timer.reset();
                                if (timer.time() < 0.3) {
                                    ballMoveFront.setPower(-1);
                                    ballMoveBack.setPower(-1);
                                } else if (timer.time() > 0.3 && timer.time() < 0.6) {
                                    centerHold();
                                    botToTop.setPower(1);
                                } else {
                                    centerHold();
                                    state = States.RR;
                                }
                            }
                        }
                    } else if ((greenCount == 2 && Objects.equals(pattern[1], "Green")) || greenCount == 3) {
                        if (fromFront() != 0) {
                            timer.reset();
                            if (timer.time() < 0.3) {
                                ballMoveFront.setPower(1);
                                ballMoveBack.setPower(1);
                                backIntakeEject();
                            } else {
                                backIntake();
                                state = States.RR;
                            }
                        } else if (fromBack() != 0) {
                            timer.reset();
                            if (timer.time() < 0.3) {
                                ballMoveFront.setPower(-1);
                                ballMoveBack.setPower(-1);
                                frontIntakeEject();
                            } else {
                                frontIntake();
                                state = States.RR;
                            }
                        }
                    }
                    break;
                    //remove wrong from same side intaked
//                    centerHold();
//                    if (fromFront() != 0) {
//                        timer.reset();
//                        if (timer.time() < 0.3) {
//                            frontIntakeEject();
//                        } else {
//                            state = States.RR;
//                        }
//                    } else if (fromBack() != 0) {
//                        timer.reset();
//                        if (timer.time() < 0.3) {
//                            backIntakeEject();
//                        } else {
//                            state = States.RR;
//                        }
//                    }
//                  break;
                case RW:
                    if (greenCount == 2) {
                        timer.reset();
                        if (timer.time() < 0.3) {
                            topToShoot.setPower(1);
                            botToTop.setPower(1);
                        } else {
                            topToShoot.setPower(0);
                            botToTop.setPower(0);
                            state = States.R;
                        }
                    }
                    getRight();
                    if (rightCount == 2 && wrongCount == 1) {
                        state = States.RWR;
                    } else if (rightCount == 1 && wrongCount == 2) {
                        state = States.RWW;
                    } else {
                        state = States.RW;
                    }
                    break;

                case RWR:
                    if (fromFront() != 0) {
                        timer.reset();
                        if (timer.time() < 0.3) {
                            ballMoveFront.setPower(1);
                            ballMoveBack.setPower(1);
                        } else {
                            state = States.RRR;
                        }
                    } else if (fromBack() != 0) {
                        timer.reset();
                        if (timer.time() < 0.3) {
                            ballMoveFront.setPower(-1);
                            ballMoveBack.setPower(-1);
                        } else {
                            state = States.RRR;
                        }
                    }
                    break;

                case RWW:
                    if (fromFront() != 0) {
                        timer.reset();
                        if (timer.time() < 0.3) {
                            ballMoveFront.setPower(1);
                            ballMoveBack.setPower(1);
                            backIntakeEject();
                        } else {
                            backIntake();
                            state = States.RW;
                        }
                    } else if (fromBack() != 0) {
                        timer.reset();
                        if (timer.time() < 0.3) {
                            ballMoveFront.setPower(-1);
                            ballMoveBack.setPower(-1);
                            frontIntakeEject();
                        } else {
                            frontIntake();
                            state = States.RW;
                        }
                    }
                    break;

                case W:
                    centerHold();
                    botToTop.setPower(0);
                    getRight();
                    if (rightCount == 1) {
                        state = States.WR;
                    } else if (wrongCount == 2) {
                        state = States.WW;
                    } else {
                        state = States.W;
                    }
                    break;

                case WR:
                    if (fromFront() != 0) {
                        timer.reset();
                        if (timer.time() < 0.3) {
                            ballMoveFront.setPower(1);
                            ballMoveBack.setPower(1);
                        } else if (timer.time() < 0.3 && timer.time() > 0.6){
                            botToTop.setPower(1);
                            centerHold();
                        } else {
                            state = States.RW;
                        }
                    } else if (fromBack() != 0) {
                        timer.reset();
                        if (timer.time() < 0.3) {
                            ballMoveFront.setPower(-1);
                            ballMoveBack.setPower(-1);
                        } else if (timer.time() < 0.3 && timer.time() > 0.6){
                            botToTop.setPower(1);
                            centerHold();
                        } else {
                            state = States.RW;
                        }
                    }
                    break;

                case WW:
                    if (fromFront() != 0) {
                        timer.reset();
                        if (timer.time() < 0.3) {
                            ballMoveFront.setPower(1);
                            ballMoveBack.setPower(1);
                            backIntakeEject();
                        } else {
                            backIntake();
                            state = States.W;
                        }
                    } else if (fromBack() != 0) {
                        timer.reset();
                        if (timer.time() < 0.3) {
                            ballMoveFront.setPower(-1);
                            ballMoveBack.setPower(-1);
                            frontIntakeEject();
                        } else {
                            frontIntake();
                            state = States.W;
                        }
                    }
                    break;
            }
        }
    }
}