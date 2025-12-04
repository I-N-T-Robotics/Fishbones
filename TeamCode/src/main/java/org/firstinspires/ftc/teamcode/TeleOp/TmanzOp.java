package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Shooter.ShooterHood;
import org.firstinspires.ftc.teamcode.Util.navx.AHRS;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

import java.util.Arrays;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TmanzOp", group = "teleOp")
public class TmanzOp extends LinearOpMode {

    private DcMotorEx fl, fr, bl, br;

    private AHRS gyro;
    private Intake intake;
    private Shooter shooter;
    private ShooterHood shooterHood;
    private Limelight limelight;

    public volatile double yaw;

    private boolean shootingButtonPrev = false;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "gyro"),
                AHRS.DeviceDataType.kProcessedData);

        List<DcMotorEx> allMotors = Arrays.asList(
                fl, fr, bl, br
        );

        allMotors.forEach(motor -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        intake = new Intake(hardwareMap, shooter, shooterHood, limelight);
        shooter = new Shooter(hardwareMap);
        shooterHood = new ShooterHood(hardwareMap);
        limelight = new Limelight(hardwareMap);

        new Thread(new gyroReader()).start();

        waitForStart();

        //intake.stateHandler();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;// * 1.1;
            double rx = gamepad1.right_stick_x;
            drive(y, x, rx);

            if (gamepad1.start) {
                gyro.zeroYaw();
            }

            if (gamepad1.y) {
                intake.swapPatternToggle();
            }

            if (gamepad1.b) {
                intake.swapIntakeToggle();
            }

            boolean shootingPressed = gamepad1.a;

            if (shootingPressed && !shootingButtonPrev) {
                intake.stateBeforeShooting = intake.state;
                intake.state = Intake.States.SHOOTING;
//                shooter.setShooterSpeed(limelight.getDistance());
//                shooterHood.setHoodPosition(limelight.getDistance());
                shooter.setShooterSpeed(1);
            }

            if (!shootingPressed && shootingButtonPrev) {
                if (intake.stateBeforeShooting == Intake.States.OFF) {
                    intake.state = Intake.States.OFF;
                } else {
                    intake.state = Intake.States.NOTHING;
                }
                shooter.setShooterSpeed(0);
            }
            shootingButtonPrev = shootingPressed;
            intake.stateHandler();
        }

        telemetry.addData("shooterSpeed", shooter.getShooterCurrentRPM());
        telemetry.addData("shooterHoodAngle", shooterHood.getHoodCurrentAngle());
        telemetry.addData("distance", limelight.getDistance());
        telemetry.update();
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

    private class gyroReader implements Runnable {
        @Override
        public void run() {
            while (opModeIsActive() && !isStopRequested()) {

                yaw = -Math.toRadians(gyro.getYaw());

                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
    }
}