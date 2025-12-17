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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TmanzOp", group = "teleOp")
public class TmanzOp extends LinearOpMode {

    private DcMotorEx fl, fr, bl, br;

    private GoBildaPinpointDriver gyro;
    private Intake intake;
    private Shooter shooter;
    private ShooterHood shooterHood;
    private Limelight limelight;

    private Follower follower;
    public static Pose startDrivePose;
    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro = hardwareMap.get(GoBildaPinpointDriver.class, "gyro");

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
        follower.setStartingPose(limelight.getTagID() == 20 ? new Pose(56.70967741935483, 16.451612903225808, Math.toRadians(112)) : new Pose(56.70967741935483, 16.451612903225808, Math.toRadians(112)).mirror());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        follower.startTeleopDrive(true);

        intake.stateHandler();
        shooter.idle();

        while (opModeIsActive()) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    false // field Centric
            );

            follower.update();
            telemetryM.update();

            if (gamepad1.y) {
                intake.swapPatternToggle();
            }

            if (gamepad1.y) {
                intake.startTopToShoot();
            }

            if (gamepad1.b) {
                intake.swapIntakeToggle();
            }

            boolean shootingPressed = gamepad1.a;

            if (shootingPressed) {
                intake.stateBeforeShooting = intake.state;
                intake.state = Intake.States.SHOOTING;
//                shooter.setShooterSpeed(limelight.getDistance());
//                shooterHood.setHoodPosition(limelight.getDistance());
//                shooter.runShooter();
            }

            if ((intake.state == Intake.States.SHOOTING) && !shootingPressed) {
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
            telemetry.addData("mode", intake.state);
            telemetry.addData("pinpoint", gyro.getHeading(AngleUnit.RADIANS));
            telemetry.addData("shooting", shootingPressed);
            telemetry.update();
        }
    }
}