package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.Constants.Settings.Shooter.distances;
import static org.firstinspires.ftc.teamcode.Constants.Settings.Shooter.hoodAngles;
import static org.firstinspires.ftc.teamcode.Constants.Settings.Shooter.shooterSpeeds;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants.Field;
import org.firstinspires.ftc.teamcode.Odometry.Odometry;
//import org.firstinspires.ftc.teamcode.Shooter.Shooter;
//import org.firstinspires.ftc.teamcode.Shooter.ShooterHood;
import org.firstinspires.ftc.teamcode.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModules;
import org.firstinspires.ftc.teamcode.Util.navx.AHRS;

import edu.wpi.first.math.geometry.Translation2d;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

//    private Shooter shooter;
//    private ShooterHood hood;
//    private Intake intake;
    private AHRS gyro;
    public volatile double yaw;

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = null;
        Odometry odometry = null;
        gyro = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "gyro"),
                AHRS.DeviceDataType.kProcessedData);
//        shooter = new Shooter(hardwareMap);
//        hood = new ShooterHood(hardwareMap);
//        intake = new Intake(hardwareMap);
//
//        shooter.rpmRegression(distances, shooterSpeeds);
//        hood.hoodRegression(distances, hoodAngles);

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

            //fix to get correct tags
            double shooterDistance = Odometry.getInstance().getDistanceToTag(Field.getTag(1)) * (537.6/60 /*encoder tick per rotations*/);
            double shooterHoodDistance = (Odometry.getInstance().getDistanceToTag(Field.getTag(1)) / 45.0 /*Example mapping 0°–45° → 0.0–1.0*/);
            shooterHoodDistance = Math.max(0.0, Math.min(1.0, shooterHoodDistance));

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            //drive
            Translation2d velocity = new Translation2d(x, y);

            swerveDrive.drive(velocity, rx);
            swerveDrive.updateModules();

            //shooter + hood
//            if (gamepad1.bWasPressed()) {
//                shooter.setShooterSpeed(shooterDistance);
//                hood.setHoodPosition(shooterHoodDistance);
//            } else if (gamepad1.bWasReleased()) {
//                shooter.setShooterSpeed(1); //minimum speed (10 is distance)
//            }

            //intake
//            if (gamepad1.yWasPressed() && intake.isToggled == false) {
//                intake.intake();
//                intake.swapToggle();
//            } else if (gamepad1.yWasPressed() && intake.isToggled == true) {
//                intake.intakeStop();
//                intake.swapToggle();
//            }

            telemetry.addData("inputLeftX", gamepad1.left_stick_x);
            telemetry.addData("inputLeftY", -gamepad1.left_stick_y);
            telemetry.addData("inputRightX", gamepad1.right_stick_x);

            telemetry.addData("globalFinalOutput0", mod0.globalFinal);
            telemetry.addData("globalFinalOutput1", mod1.globalFinal);
            telemetry.addData("globalFinalOutput2", mod2.globalFinal);
            telemetry.addData("globalFinalOutput3", mod3.globalFinal);

            telemetry.addData("globalVel0", mod0.globalVel);
            telemetry.addData("globalVel1", mod1.globalVel);
            telemetry.addData("globalVel2", mod2.globalVel);
            telemetry.addData("globalVel", mod3.globalVel);

            telemetry.addData("globalTarget0", mod0.globalTarget);
            telemetry.addData("globalTarget1", mod1.globalTarget);
            telemetry.addData("globalTarget2", mod2.globalTarget);
            telemetry.addData("globalTarget3", mod3.globalTarget);

//            telemetry.addData("FR Turn", mod0.getAngle().getDegrees());
//            telemetry.addData("FL Turn", SwerveDrive.getInstance().getSwerveModules()[1].getAngle().getDegrees());
//            telemetry.addData("BR Turn", SwerveDrive.getInstance().getSwerveModules()[2].getAngle().getDegrees());
//            telemetry.addData("BL Turn", SwerveDrive.getInstance().getSwerveModules()[3].getAngle().getDegrees());
//
//            telemetry.addData("FR Turn", SwerveDrive.getInstance().getSwerveModules()[0].getTargetStateAngle());
//            telemetry.addData("VxSpeed", dr.globalSpeed.vxMetersPerSecond);
//            telemetry.addData("VySpeed", dr.globalSpeed.vyMetersPerSecond);
//
//            telemetry.addData("Twistdx",  dr.globalTwistdx);
//            telemetry.addData("FRPID",  mod0.globalPID);
//            telemetry.addData("FLPID",  mod1.globalPID);
//            telemetry.addData("BRPID",  mod2.globalPID);
//            telemetry.addData("BLPID",  mod3.globalPID);
//            telemetry.addData("Target", mod0.globalTarget);
//            telemetry.addData("Angle", mod0.globalAngle);
//
//            telemetry.addData("rawTurnTargetBR", mod2.getRawTurnTargetAngle());
//            telemetry.addData("rawSpeedTargetBR", mod2.getRawSpeedTarget());
//
//            telemetry.addData("FRV", mod0.getVolts());
//            telemetry.addData("FLV", mod1.getVolts());
//            telemetry.addData("BRV", mod2.getVolts());
//            telemetry.addData("BLV", mod3.getVolts());
//
            telemetry.addData("FRDV", mod0.getRawDriveEncoder());
            telemetry.addData("FLDV", mod1.getRawDriveEncoder());
            telemetry.addData("BRDV", mod2.getRawDriveEncoder());
            telemetry.addData("BLDV", mod3.getRawDriveEncoder());
//
//            telemetry.addData("FRDrV", mod0.getVelocity());
//            telemetry.addData("FLDrV", mod1.getVelocity());
//            telemetry.addData("BRDrV", mod2.getVelocity());
//            telemetry.addData("BLDrV", mod3.getVelocity());

            telemetry.addData("DrivePower", mod0.globalFinal);
            telemetry.addData("MeasuredVelocity", mod0.getVelocity());
            telemetry.addData("TargetVelocity", mod0.globalTarget);

            telemetry.update();
        }
    }
    private class gyroReader implements Runnable {
        @Override
        public void run() {
            while (!isStopRequested()) {
                double currentYaw;

                synchronized (TeleOp.this) {
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