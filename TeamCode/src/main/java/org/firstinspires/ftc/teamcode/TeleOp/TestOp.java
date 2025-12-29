package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;

import java.util.Arrays;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestOp", group = "teleOp")
public class TestOp extends LinearOpMode {

    private DcMotorEx fl, fr, bl, br;

    private GoBildaPinpointDriver gyro;
    private Intake intake;
    private Shooter shooter;

    public volatile double yaw;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro = hardwareMap.get(GoBildaPinpointDriver.class, "gyro");

        List<DcMotorEx> allMotors = Arrays.asList(
                fl, fr, bl, br
        );

        allMotors.forEach(motor -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, shooter );

        waitForStart();

        intake.stateHandler();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            drive(y, x, rx);

            if (gamepad1.start) {
                gyro.resetPosAndIMU();
            }

            if (gamepad1.y) {
                intake.centerHold();
                intake.frontIntake();
                intake.startBotToTop();
                intake.startTopToShoot();
                shooter.runShooter();
            } else {

            }

            if (gamepad1.x) {
                shooter.stop();
            }

            if (gamepad1.a) {
                intake.startBotToTop();
                intake.centerHold();
                intake.backIntake();
            } else {

            }

            if (gamepad1.b) {
                intake.centerHold();
                intake.backIntake();
            } else {

            }

			telemetry.addData("shooterPct", shooter.targetProgress());
            telemetry.addData("hootPct", shooter.hood.progress());
            telemetry.addData("distance", shooter.lime.getDistance());
            telemetry.addData("mode", intake.state);
            telemetry.addData("pinpoint", gyro.getHeading(AngleUnit.RADIANS));
            telemetry.update();

        }
    }

    public void drive(double y, double x, double rx) {
        double botHeading = yaw;

        x = x * 1.1;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

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
}
