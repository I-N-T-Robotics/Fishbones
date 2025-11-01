package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.Constants.Settings.Shooter.distances;
import static org.firstinspires.ftc.teamcode.Constants.Settings.Shooter.hoodAngles;
import static org.firstinspires.ftc.teamcode.Constants.Settings.Shooter.shooterSpeeds;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants.Field;
import org.firstinspires.ftc.teamcode.Odometry.Odometry;
//import org.firstinspires.ftc.teamcode.Shooter.Shooter;
//import org.firstinspires.ftc.teamcode.Shooter.ShooterHood;
import org.firstinspires.ftc.teamcode.Swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

//    private Shooter shooter;
//    private ShooterHood hood;
//    private Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = null;
        Odometry odometry = null;
//        shooter = new Shooter(hardwareMap);
//        hood = new ShooterHood(hardwareMap);
//        intake = new Intake(hardwareMap);
//
//        shooter.rpmRegression(distances, shooterSpeeds);
//        hood.hoodRegression(distances, hoodAngles);

        waitForStart();

        swerveDrive = SwerveDrive.createInstance(gamepad1, hardwareMap, telemetry);
        odometry = Odometry.createInstance(hardwareMap, telemetry);

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

            telemetry.addData("FR Turn", SwerveDrive.getInstance().getSwerveModules()[0].getAngle().getDegrees());
            telemetry.addData("FR Turn", SwerveDrive.getInstance().getSwerveModules()[0].getTargetStateAngle());
            telemetry.addData("FL Turn", SwerveDrive.getInstance().getSwerveModules()[1].getAngle().getDegrees());
            telemetry.addData("BR Turn",SwerveDrive.getInstance().getSwerveModules()[2].getAngle().getDegrees());
            telemetry.addData("BL Turn", SwerveDrive.getInstance().getSwerveModules()[3].getAngle().getDegrees());

            telemetry.update();
        }
    }
}