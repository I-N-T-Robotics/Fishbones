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
        intake = new Intake(hardwareMap, shooter ) ;

        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(8.32258064516129, 8.322580645161283, 90));
        //normal auto
        follower.setStartingPose(limelight.getTagID() == 20 ? new Pose(56.70967741935483, 26.903225806451605, Math.toRadians(112)) : new Pose(56.70967741935483, 26.903225806451605, Math.toRadians(112)).mirror());
        follower.update();
        intake.off();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        follower.startTeleopDrive(true);

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
			intake.update() ;

            if (gamepad1.start) {
                gyro.resetPosAndIMU();
            }

			telemetry.addData("shooterPct", shooter.targetProgress());
            telemetry.addData("hootPct", shooter.hood.progress());
            telemetry.addData("distance", shooter.lime.getDistance());

            telemetry.addData("mode", intake.state);
            telemetry.addData("pinpoint", gyro.getHeading(AngleUnit.RADIANS));
            telemetry.addData("shooting", shootingPressed);
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
