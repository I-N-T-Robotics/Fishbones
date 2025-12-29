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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tman Basic", group = "teleOp")
public class TmanBasicOp extends LinearOpMode {

    private DcMotorEx fl, fr, bl, br;

    private GoBildaPinpointDriver gyro;
    private Intake intake;
    private Shooter shooter;

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

        gyro = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        List<DcMotorEx> allMotors = Arrays.asList(fl, fr, bl, br);

        allMotors.forEach(motor -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, shooter) ;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(limelight.getTagID() == 20 ? new Pose(56.70967741935483, 16.451612903225808, Math.toRadians(112)) : new Pose(56.70967741935483, 16.451612903225808, Math.toRadians(112)).mirror());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        follower.startTeleopDrive(true);

        intake.off();
        shooter.stop();

        while (opModeIsActive()) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    false // field Centric
            );

            follower.update();
            telemetryM.update();
			intake.update() ;

            if (gamepad1.start) {
                gyro.resetPosAndIMU();
            }

            if ( gamepad1.b ) 			{ intake.enable(); }
            if ( gamepad1.a ) 			{ intake.shoot() ; }
			if ( gamepad1.dpad_down ) 	{ intake.off() ; }


			telemetry.addData("shooterPct", shooter.targetProgress());
            telemetry.addData("hootPct", shooter.hood.progress());
            telemetry.addData("distance", shooter.lime.getDistance());

            telemetry.addData("mode", intake.state);
            telemetry.addData("pinpoint", gyro.getHeading(AngleUnit.RADIANS));
            telemetry.addData("shooting", shootingPressed);
            telemetry.update();
        }
    }
}
