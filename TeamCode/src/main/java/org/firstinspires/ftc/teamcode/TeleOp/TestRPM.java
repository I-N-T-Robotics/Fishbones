package org.firstinspires.ftc.teamcode.TeleOp;

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

public class TestRPM extends LinearOpMode {
    private Intake intake;
    private Shooter shooter;
    private ShooterHood shooterHood;
    private Limelight limelight;

    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter = new Shooter(hardwareMap);
        shooterHood = new ShooterHood(hardwareMap);
        limelight = new Limelight(hardwareMap);
        intake = new Intake(hardwareMap, shooter, shooterHood, limelight);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();


        while (opModeIsActive()) {

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



            telemetry.addData("shooterSpeed", shooter.getShooterCurrentRPM());
            telemetry.addData("shooterHoodAngle", shooterHood.getHoodCurrentAngle());
            telemetry.addData("distance", limelight.getDistance());
            telemetry.addData("mode", intake.state);
            telemetry.addData("shooting", shootingPressed);
            telemetry.update();
        }
    }
}
