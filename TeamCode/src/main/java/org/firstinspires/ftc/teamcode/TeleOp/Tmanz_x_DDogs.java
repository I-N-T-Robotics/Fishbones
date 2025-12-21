package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Shooter.ShooterHood;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tmanz_X_DDogs", group = "teleOp")
public class Tmanz_x_DDogs extends LinearOpMode {

    private DcMotorEx fl, fr, bl, br;

    private GoBildaPinpointDriver gyro;
    private Intake intake;
    private Shooter shooter;
    private ShooterHood shooterHood;
    private Limelight limelight;

    private Follower follower;
    public static Pose startDrivePose;
    private TelemetryManager telemetryM;

    private static double ALIGN_KP = 0.05;
    private static double ALIGN_MAX_POWER = 0.125;
    private static double ALIGN_DEADBAND = 1.5; // degrees of tx

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        List<DcMotorEx> allMotors = Arrays.asList(fl, fr, bl, br);

        allMotors.forEach(motor -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        shooter = new Shooter(hardwareMap);
        shooterHood = new ShooterHood(hardwareMap);
        limelight = new Limelight(hardwareMap);
        intake = new Intake(hardwareMap, shooter, shooterHood, limelight);

        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(8.32258064516129, 8.322580645161283, 90));
        //DDogs auto
        follower.setStartingPose(limelight.getTagID() == 20 ? new Pose(33.871, 10.839, Math.toRadians(90)) : new Pose(33.871, 10.839, Math.toRadians(90)).mirror());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        follower.startTeleopDrive(true);

        intake.stateHandler();
        shooter.idle();

        while (opModeIsActive()) {
            limelight.update();

            double drive = -gamepad1.left_stick_y * 1.1;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            if (gamepad1.y) {
                turn = -getLimelightAlignTurn();
            }

            follower.setTeleOpDrive(
                    drive,
                    strafe,
                    turn,
                    false
            );

            follower.update();
            telemetryM.update();

            if (gamepad1.start) {
                gyro.resetPosAndIMU();
            }

//            if (gamepad1.leftBumperWasPressed()) {
//                intake.decreaseTestAngle();
//            }
//
//            if (gamepad1.rightBumperWasPressed()) {
//                intake.increaseTestAngle();
//            }
//
//            if (gamepad1.aWasPressed()) {
//                intake.decreaseTestRPM();
//            }
//
//            if (gamepad1.xWasPressed()) {
//                intake.increaseTestRPM();
//            }

//            if (gamepad1.y) {
//                intake.swapPatternToggle();
//            }

//            if (gamepad1.y) {
//                intake.startTopToShoot();
//            }

            if (gamepad1.bWasPressed()) {
                intake.swapIntakeToggle();
            }

            if (gamepad1.xWasPressed()) {
                intake.backIntakeEject();
            }

            //shoot no align
            boolean shootingPressedForced = gamepad1.a;

            if (shootingPressedForced) {
                intake.stateBeforeShooting = intake.state;
                intake.state = Intake.States.SHOOTING;
//                shooter.setShooterSpeed(limelight.getLastDist());
//                shooterHood.setHoodPosition(limelight.getLastDist());
//                shooter.runShooter();
            }

            if ((intake.state == Intake.States.SHOOTING) && !shootingPressedForced) {
                if (intake.stateBeforeShooting == Intake.States.OFF) {
                    intake.state = Intake.States.OFF;
                } else {
                    intake.state = Intake.States.NOTHING;
                }
                shooter.setShooterSpeed(0);
            }

            //shoot align
            boolean shootingPressed = gamepad1.y;

            if (shootingPressed && isAligned()) {
                intake.stateBeforeShooting = intake.state;
                intake.state = Intake.States.SHOOTING;
//                shooter.setShooterSpeed(limelight.getLastDist());
//                shooterHood.setHoodPosition(limelight.getLastDist());
//                shooter.runShooter();
            }

            if ((intake.state == Intake.States.SHOOTING) && !shootingPressed && !shootingPressedForced) {
                if (intake.stateBeforeShooting == Intake.States.OFF) {
                    intake.state = Intake.States.OFF;
                } else {
                    intake.state = Intake.States.NOTHING;
                }
                shooter.setShooterSpeed(0);
            }
            intake.stateHandler();

            telemetry.addData("shooterSpeed", shooter.getShooterCurrentRPM());
            telemetry.addData("shooterHoodAngle", shooterHood.getHoodCurrentAngle());
            telemetry.addData("distance", limelight.getDistance());
            telemetry.addData("lastDistance", limelight.getLastDist());
            telemetry.addData("mode", intake.state);
            telemetry.addData("pinpoint", gyro.getHeading(AngleUnit.RADIANS));
            telemetry.addData("shooting", shootingPressed);
            telemetry.addData("testRPM", intake.getTestRPM());
            telemetry.addData("testHoodAngle", intake.getHoodTestAngle());
            telemetry.addData("botToTopShoot", intake.getbotToTopSpeed());
            telemetry.addData("tx", limelight.getxFromTag());
            telemetry.addData("lastTx", limelight.getLastTx());
            telemetry.addData("aligned", isAligned());
            telemetry.update();
        }
    }

    private double getLimelightAlignTurn() {
        double tx = limelight.getLastTx();

        if (Math.abs(tx) <= ALIGN_DEADBAND) {
            return 0;
        }

        double turn = tx * ALIGN_KP;

        turn = Math.max(-ALIGN_MAX_POWER, Math.min(ALIGN_MAX_POWER, turn));

        return turn;
    }

    private boolean isAligned() {
        double tx = limelight.getLastTx();
        return Math.abs(tx) <= ALIGN_DEADBAND;
    }
}

//add align to auto