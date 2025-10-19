package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    private DcMotorEx drive;
    private CRServo turn;

    @Override
    public void runOpMode() throws InterruptedException {
//        drive = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
//        turn = hardwareMap.get(CRServo.class, "FrontRightTurn");
        int i = 0;

        waitForStart();
        while(opModeIsActive()) {
            //drive.setPower(1.0);

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            ChassisSpeeds speeds = new ChassisSpeeds(x, y, rx);
            Translation2d velocity = new Translation2d(x, y);
            telemetry.addData("work1", i);
            telemetry.update();
            SwerveDrive.getInstance().drive(velocity, rx);
            i++;
//            SwerveDrive.getInstance().setFieldRelativeSpeeds(speeds);
        }
    }
}