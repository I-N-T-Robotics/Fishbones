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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;

public class TestRPM extends LinearOpMode {
    private Intake intake;
    private Shooter shooter;

    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, shooter) ;

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

			telemetry.addData("shooterPct", shooter.targetProgress());
            telemetry.addData("hootPct", shooter.hood.progress());
            telemetry.addData("distance", shooter.lime.getDistance());
            telemetry.addData("mode", intake.state);
            telemetry.addData("shooting", shootingPressed);
            telemetry.update();
        }
    }
}
