package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.util.Timer;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

@TeleOp(name = "Red TeleOp", group = "QUAL 9 TeleOp")
public class RedTeleOp extends OpMode {

    private Follower follower;
    public static Pose autoEndPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private DcMotorEx fl, fr, bl, br;

    private Intake intake;
    private Shooter shooter;

    private Timer timer;

    public enum States {
        OFF,
        INTAKE,
        PREP,
        SHOOT,
    }

    States states = States.OFF;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(autoEndPose == null ? new Pose(25.5, 72, Math.toRadians(0)).mirror() : autoEndPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(60.25, 84, Math.toRadians(135)).mirror())))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8)) //TODO: tune t
                .build();

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        List<DcMotorEx> allMotors = Arrays.asList(
                fl, fr, bl, br
        );

        allMotors.forEach(motor -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        timer = new Timer();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
        }

        if (gamepad1.rightBumperWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.leftBumperWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        stateHandler();

        if (gamepad1.x) {
            states = States.OFF;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }

    public void stateHandler() {
        switch (states) {
            case OFF:
                intake.off();
                shooter.shootPower(0.1);
                if (gamepad1.b) { states = States.INTAKE; }
                break;

            case INTAKE:
                intake.enable();

                if (gamepad1.y) {
                    states = States.PREP;
                    timer.resetTimer();
                }
                break;

            case PREP:
                shooter.shootPower(0.3);
                if (gamepad1.y && timer.getElapsedTimeSeconds() > 0.5) {
                    states = States.SHOOT;
                }
                break;

            case SHOOT:
                intake.feed();

                if (gamepad1.b) { states = States.INTAKE; }
                break;
        }
    }
}
