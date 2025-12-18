package org.firstinspires.ftc.teamcode.Intake;

import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.Settings;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Shooter.ShooterHood;
import org.firstinspires.ftc.teamcode.Util.Normalize;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

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

    public States state;
    public States previousState = States.OFF;
    public States stateBeforeShooting = States.OFF;

    private ElapsedTime timer = new ElapsedTime();

    private Shooter shooter;
    private ShooterHood shooterHood;
    private Limelight limelight;
    
    private double firstShuffle;
    private double secondShuffle;
    private double thirdShuffle;

    private Normalize normalizeFront;
    private Normalize normalizeBack;

    public enum States {
        OFF,
        NOTHING,
        SHOOTING,
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

    public Intake(HardwareMap hardwareMap, Shooter shooter, ShooterHood shooterHood) {
        this.shooter = shooter;
        this.shooterHood = shooterHood;
        this.limelight = limelight;

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

        botToTop.setDirection(DcMotorSimple.Direction.REVERSE);
        ballMoveBack.setDirection(DcMotorSimple.Direction.REVERSE);
        topToShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        backIntakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backIntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontSense = hardwareMap.get(ColorSensor.class, "frontSensor");
        backSense = hardwareMap.get(ColorSensor.class, "backSensor");
        
        firstShuffle = Settings.Intake.shuffleOne;
        secondShuffle = Settings.Intake.shuffleTwo;
        thirdShuffle = Settings.Intake.shuffleThree;

        this.normalizeFront = new Normalize(frontSense.red(), frontSense.green(), frontSense.blue());
        this.normalizeBack = new Normalize(backSense.red(), backSense.green(), backSense.blue());

        state = States.OFF;

        loadPatternFromPreferences(hardwareMap);
    }

    private void loadPatternFromPreferences(HardwareMap hardwareMap) {
        Context context = hardwareMap.appContext;
        SharedPreferences prefs = context.getSharedPreferences("RobotPrefs", Context.MODE_PRIVATE);

        int tagID = prefs.getInt("patternTagID", -1); // -1 means no tag stored

        switch (tagID) {
            case 21 -> pattern = new String[]{"Green", "Purple", "Purple"};
            case 22 -> pattern = new String[]{"Purple", "Green", "Purple"};
            case 23 -> pattern = new String[]{"Purple", "Purple", "Green"};
            default -> {
                pattern = new String[]{"Green", "Purple", "Purple"}; // safe default
                System.err.println("No valid tag stored; using default pattern");
            }
        }
    }

    /*** Save the first valid tag (21, 22, 23) seen in Auto ***/
    public static void saveTagID(HardwareMap hardwareMap, int tagID) {
        // Only accept valid tags
        if (tagID != 21 && tagID != 22 && tagID != 23) {
            return; // ignore invalid tags
        }

        Context context = hardwareMap.appContext;
        SharedPreferences prefs = context.getSharedPreferences("RobotPrefs", Context.MODE_PRIVATE);

        // Only store if no tag has been stored yet
        if (!prefs.contains("patternTagID")) {
            prefs.edit().putInt("patternTagID", tagID).apply();
        }
    }

    public static void resetTagID(HardwareMap hardwareMap) {
        Context context = hardwareMap.appContext;
        SharedPreferences prefs = context.getSharedPreferences("RobotPrefs", Context.MODE_PRIVATE);
        prefs.edit().remove("patternTagID").apply(); // remove any stored tag
    }

    public void startBotToTop() {
        botToTop.setPower(1);
    }

    public void startTopToShoot() {
        topToShoot.setPower(1);
    }

    public void intaking() {
        if (isToggledIntake) {
            frontIntake();
            backIntake();
        } else {
            frontIntakeStop();
            backIntakeStop();
        }
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

        rightCount = 0;
        wrongCount = 0;
        pieceCount = 0;
        greenCount = 0;

        if (!isToggledPattern) {
            state = States.OFF;
        }

        else {
            state = States.NOTHING;
        }

        previousState = state;

        centerStop();
        frontIntakeStop();
        backIntakeStop();
        botToTop.setPower(0);
        topToShoot.setPower(0);

        timer.reset();
    }

    public void shuffle() {
        if (frontSense.alpha() != 0) {
            if (timer.time() < firstShuffle) {
                ballMoveFront.setPower(1);
                ballMoveBack.setPower(1);
            }
        } else if (backSense.alpha() != 0) {
            if (timer.time() < firstShuffle) {
                ballMoveFront.setPower(-1);
                ballMoveBack.setPower(-1);
            }
        }
    }

    public void shuffleOut() {
        shuffle();
        if (timer.time() > firstShuffle && timer.time() < secondShuffle) {
            backIntakeEject();
            frontIntakeEject();
        } else if (timer.time() > secondShuffle && timer.time() < thirdShuffle) {
            backIntake();
            frontIntake();
        }
    }

    public void getRight() {
        if (frontSense.alpha() > 0 || backSense.alpha() > 0) {
            if ((normalizeFront.rV2() > 200 || normalizeBack.rV2() > 200) && Objects.equals(pattern[rightCount], "Green")) {
                rightCount++;
                pieceCount++;
                greenCount++;
            } else if ((normalizeFront.rV2() < 50 || normalizeBack.rV2() < 50) && Objects.equals(pattern[rightCount], "Purple")) {
                rightCount++;
                pieceCount++;
            }
        }
    }

    public void stateHandler() {
        if (state == States.SHOOTING) {
            shooter.runShooter();
//            if shooter is at speed and hood is at angle
//            if ((shooter.getShooterRPM(limelight.getDistance()) - shooter.getShooterCurrentRPM() < 100) &&
//                    shooterHood.getHoodTargetAngle(limelight.getDistance()) - shooterHood.getHoodCommandedAngle() < 5) {
            if ((shooter.getShooterCurrentRPM() > 100)) {
                isToggledIntake = true;
                intaking();
                centerHold();
                botToTop.setPower(1);
                topToShoot.setPower(1);
            } else {
                isToggledIntake = false;
                frontIntakeStop();
                backIntakeStop();
                centerStop();
                botToTop.setPower(0);
                topToShoot.setPower(0);
            }
            return;
        }
//        }

        if (isToggledIntake) {
            frontIntake();
            backIntake();
        } else if (!isToggledIntake) {
            frontIntakeStop();
            backIntakeStop();
        }

        if (state != previousState) {
            timer.reset();
            previousState = state;
            pieceCount = 0;

            isToggledIntake = false;
            intaking();
        }

        if (!isToggledPattern && state != States.SHOOTING) {
            state = States.OFF;
            shooter.idle();

            if (state == States.OFF) {
                topToShoot.setPower(0);
                getRight();
                if (pieceCount == 0) {
                    botToTop.setPower(1);
                    centerHold();
                    intaking();
                } else if (pieceCount == 1) {
                    centerHold();
                    botToTop.setPower(1);
                } else if (pieceCount == 2) {
                    centerHold();
                    botToTop.setPower(0);
                } else if (pieceCount == 3) {
                    swapIntakeToggle();
                }
            }
        } else if (isToggledPattern) {
            switch (state) {
//                case SHOOTING:
    //            if ((shooter.getShooterRPM(limelight.getDistance()) - shooter.getShooterCurrentRPM() < 100) &&
    //                    shooterHood.getHoodTargetAngle(limelight.getDistance()) - shooterHood.getHoodCurrentAngle() < 5) {
//                        isToggledIntake = true;
//                        intaking();
//                        centerHold();
//                        botToTop.setPower(1);
//                        topToShoot.setPower(1);
//                        shooter.runShooter();
//                        break;
//                }

                case NOTHING:
                    rightCount = 0;
                    wrongCount = 0;
                    pieceCount = 0;
                    greenCount = 0;

                    topToShoot.setPower(0);
                    shooter.idle();
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
                    shooter.idle();
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
                    shooter.idle();
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
                    shooter.idle();
                    if (timer.time() > firstShuffle) {
                        swapIntakeToggle();
                    }
                    centerStop();
                    botToTop.setPower(0);
                    break;

                case RRW:
                    shooter.idle();
                    centerHold();
                    if (greenCount == 2 && Objects.equals(pattern[0], "Green")) {
                        if (timer.time() < firstShuffle) {
                            topToShoot.setPower(1);
                            shuffle();
                        } else if (timer.time() > firstShuffle && timer.time() < secondShuffle) {
                            centerHold();
                            botToTop.setPower(1);
                        } else {
                            botToTop.setPower(0);
                            state = States.RR;
                        }
                    } else if ((greenCount == 2 && Objects.equals(pattern[1], "Green")) || greenCount == 3) {
                        shuffleOut();
                        state = States.RR;
                    }
                    break;
                //remove wrong from same side intaked
//                    centerHold();
//                    if (fromFront() != 0) {
//                        if (timer.time() < firstShuffle) {
//                            frontIntakeEject();
//                        } else {
//                            state = States.RR;
//                        }
//                    } else if (fromBack() != 0) {
//                        if (timer.time() < firstShuffle) {
//                            backIntakeEject();
//                        } else {
//                            state = States.RR;
//                        }
//                    }
//                  break;
                case RW:
                    shooter.idle();
                    if (greenCount == 2 && timer.time() < firstShuffle) {
                        topToShoot.setPower(1);
                        botToTop.setPower(1);
                    } else {
                        topToShoot.setPower(0);
                        botToTop.setPower(0);
                        state = States.R;
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
                    shooter.idle();
                    shuffle();
                    state = States.RRR;
                    break;

                case RWW:
                    shooter.idle();
                    shuffleOut();
                    state = States.RW;
                    break;

                case W:
                    shooter.idle();
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
                    shooter.idle();
                    shuffle();
                    if (timer.time() > firstShuffle && timer.time() < secondShuffle) {
                        botToTop.setPower(1);
                        centerHold();
                    } else {
                        state = States.RW;
                    }
                    break;

                case WW:
                    shooter.idle();
                    shuffleOut();
                    state = States.W;
                    break;
            }
        }
    }
}
//change all argb ranges
//override for less than 3 balls picked up in auto