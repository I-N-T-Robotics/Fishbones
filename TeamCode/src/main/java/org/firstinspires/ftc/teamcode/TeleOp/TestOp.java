package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestOp", group = "teleOp")
public class TestOp extends LinearOpMode {

    private Follower follower;
    public static Pose startPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private DcMotorEx fl, fr, bl, br;

    private GoBildaPinpointDriver gyro;
    private Intake intake;
    private Shooter shooter;

    public volatile double yaw;
    private double currSpeed = 0;

    public enum States {
        OFF,
        INTAKE,
        PREP,
        SHOOT,
    }

    States states = States.OFF;

    @Override
    public void runOpMode() throws InterruptedException {
        // Follower init stuff
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose == null ? new Pose() : startPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(58, 85))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8)) //TODO: tune t
                .build();

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        gyro.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        gyro.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        gyro.setOffsets(-1, 6.6, DistanceUnit.INCH);

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

        waitForStart();

        intake.off();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            drive(y, x, rx);

            if (gamepad1.start) {
                gyro.resetPosAndIMU();
            }


            shooter.shootPower(currSpeed);

            if (gamepad1.dpad_down) {
                intake.feed();
            }

            if (gamepad1.dpad_up) {
                intake.off();
            }

            if (gamepad1.x) {
                states = States.OFF;
            }

            stateHandler();

			//telemetry.addData("shooterPct", shooter.targetProgress());
            //telemetry.addData("hootPct", shooter.hood.progress());
            telemetry.addData("mode", states.name());
            telemetry.addData("raw vel", shooter.getRawSpeed());
            telemetry.addData("pinpoint", gyro.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();

        }
    }

    public void drive(double y, double x, double rx) {
        double botHeading = gyro.getHeading(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
    }

    public void stateHandler() {
        switch (states) {
            case OFF:
                intake.off();

                if (gamepad1.a) { states = States.INTAKE; }
                break;

            case INTAKE:
                intake.enable();

                if (gamepad1.b) { states = States.PREP; }
                break;

            case PREP:
                //intake.shoot(shooter.lime.getDistance());
                shooter.shootPower(.25);

                if (gamepad1.y) { states = States.SHOOT; }
                break;

            case SHOOT:
                intake.feed();

                if (gamepad1.a) { states = States.INTAKE; }
                break;
        }
    }
}
