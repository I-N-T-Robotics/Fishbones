package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModules;
import org.firstinspires.ftc.teamcode.Util.navx.AHRS;

import edu.wpi.first.math.geometry.Translation2d;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class Swerve extends LinearOpMode {

    private AHRS gyro;
    public volatile double yaw;
    private boolean xModeActive = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = null;
        Odometry odometry = null;
        gyro = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "gyro"),
                AHRS.DeviceDataType.kProcessedData);

        new Thread(new gyroReader()).start();

        waitForStart();

        swerveDrive = SwerveDrive.createInstance(gamepad1, hardwareMap, telemetry);

        SwerveDrive dr = SwerveDrive.getInstance();

        SwerveModules mod0 = dr.getSwerveModules()[0];
        SwerveModules mod1 = dr.getSwerveModules()[1];
        SwerveModules mod2 = dr.getSwerveModules()[2];
        SwerveModules mod3 = dr.getSwerveModules()[3];

        mod0.setDriveConfig();
        mod1.setDriveConfig();
        mod2.setDriveConfig();
        mod3.setDriveConfig();
        odometry = Odometry.createInstance();

        while(opModeIsActive()) {

            double x = gamepad1.left_stick_x * 1.1;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            //drive
            Translation2d input = new Translation2d(x, y);

            if (!xModeActive) {
                swerveDrive.drive(input, rx);
            }
            swerveDrive.updateModules();

            if (gamepad1.yWasPressed()) {
                xModeActive = !xModeActive;
                if (xModeActive) {
                    swerveDrive.setXMode();
                } else {
                    swerveDrive.stop();
                }
            }
        }
    }
    private class gyroReader implements Runnable {
        @Override
        public void run() {
            while (!isStopRequested()) {
                double currentYaw;

                synchronized (Swerve.this) {
                    currentYaw = -Math.toRadians(gyro.getYaw());
                    yaw = currentYaw;
                }

                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }
}