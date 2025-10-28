package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModuleBase;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModules;
import org.firstinspires.ftc.teamcode.Util.Filters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();
    private final double UPDATE_PERIOD_MS = 20;

    private double prevX = 0.0;
    private double prevY = 0.0;
    private double prevRX = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = null;
        Odometry odometry = null;

        timer.reset();
        double lastUpdateTime = 0;

        waitForStart();

        swerveDrive = SwerveDrive.createInstance(hardwareMap, telemetry);
        odometry = Odometry.createInstance(hardwareMap, telemetry);

        while(opModeIsActive()) {
            double currentTime = timer.milliseconds();

            if(currentTime - lastUpdateTime >= UPDATE_PERIOD_MS) {
//                double rawX = Filters.applyDeadband(gamepad1.left_stick_x, 0.05);
//                double x = Filters.smoothInput(rawX, prevX, 0.2);
//                prevX = x;
//
//                double rawY = Filters.applyDeadband(-gamepad1.left_stick_y, 0.05);
//                double y = Filters.smoothInput(rawY, prevY, 0.2);
//                prevY = y;
//
//                double rawRX = Filters.applyDeadband(gamepad1.right_stick_x, 0.05);
//                double rx = Filters.smoothInput(rawRX, prevRX, 0.2);
//                prevRX = rx;

                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
                double rx = gamepad1.right_stick_x;

//                ChassisSpeeds speeds = new ChassisSpeeds(x, y, rx);
                Translation2d velocity = new Translation2d(x, y);

                swerveDrive.drive(velocity, rx);
                swerveDrive.updateModules();

                lastUpdateTime = currentTime;
            }

            telemetry.addData("inputLeftX", gamepad1.left_stick_x);
            telemetry.addData("inputLeftY", -gamepad1.left_stick_y);
            telemetry.addData("inputRightX", gamepad1.right_stick_x);

            telemetry.addData("filteredX", prevX);
            telemetry.addData("filteredY", prevY);
            telemetry.addData("filteredRX", prevRX);

            telemetry.addData("FR Turn", SwerveDrive.getInstance().getSwerveModules()[0].getAngle().getDegrees());
            telemetry.addData("FR Turn", SwerveDrive.getInstance().getSwerveModules()[0].getTargetStateAngle());
            telemetry.addData("FL Turn", SwerveDrive.getInstance().getSwerveModules()[1].getAngle().getDegrees());
            telemetry.addData("BR Turn",SwerveDrive.getInstance().getSwerveModules()[2].getAngle().getDegrees());
            telemetry.addData("BL Turn", SwerveDrive.getInstance().getSwerveModules()[3].getAngle().getDegrees());


            telemetry.update();

//            SwerveDrive.getInstance().setFieldRelativeSpeeds(speeds);
        }
    }
}