package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.Settings;
import org.firstinspires.ftc.teamcode.Util.LinearRegression;

import edu.wpi.first.math.geometry.Translation2d;
//
//
//public class Shooter {
//
//    private DcMotorEx shooterRight, shooterLeft;
//    private LinearRegression rpm = null;
//
//    public Shooter(HardwareMap hardwareMap) {
//        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
//        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
//
//        shooterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//    }
//
//    public void rpmRegression(double[] distances, double[] shooterSpeeds) {
//        if (distances.length != shooterSpeeds.length) {
//            throw new IllegalArgumentException("Shooter array != distance array");
//        }
//
//        Translation2d[] rpmPoints = new Translation2d[shooterSpeeds.length];
//
//        for (int i = 0; i < shooterSpeeds.length; i++) {
//            rpmPoints[i] = new Translation2d(distances[i], shooterSpeeds[i]);
//        }
//
//        rpm = new LinearRegression(rpmPoints);
//    }
//
//    public double getShooterRPM(double distance) {
//        return rpm.calculatePoint(distance);
//    }
//
//    public double getShooterSpeed() {
//        return shooterRight.getVelocity();
//    }
//
//    public double getShooterTargetSpeed(double distance) {
//        return getShooterRPM(distance);
//    }
//
//    public void setShooterSpeed(double distance) {
//        shooterRight.setVelocity(getShooterRPM(distance));
//        shooterLeft.setVelocity(getShooterRPM(distance));
//    }
//}

public class Shooter {

    private DcMotorEx shooterRight, shooterLeft;
    private LinearRegression rpm = null;

    private double ticksPerRev;

    private double[] settingsDistance;
    private double[] settingsShooterSpeeds;

    public Shooter(HardwareMap hardwareMap) {
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterRight.setVelocityPIDFCoefficients(Settings.Shooter.kP, Settings.Shooter.kI, Settings.Shooter.kD, Settings.Shooter.kF);
        shooterLeft.setVelocityPIDFCoefficients(Settings.Shooter.kP, Settings.Shooter.kI, Settings.Shooter.kD, Settings.Shooter.kF);

        ticksPerRev = shooterRight.getMotorType().getTicksPerRev();

        settingsDistance = Settings.Shooter.distances;
        settingsShooterSpeeds = Settings.Shooter.shooterSpeeds;

        rpmRegression();
    }

    public void rpmRegression() {
        if (settingsDistance.length != settingsShooterSpeeds.length) {
            throw new IllegalArgumentException("Shooter array != distance array");
        }

        Translation2d[] points = new Translation2d[settingsDistance.length];
        for (int i = 0; i < settingsDistance.length; i++) {
            points[i] = new Translation2d(settingsDistance[i], settingsShooterSpeeds[i]);
        }

        rpm = new LinearRegression(points);
    }

    public double getShooterRPM(double distance) {
        if (rpm == null) return 0;
        return rpm.calculatePoint(distance);
    }

    public void setShooterSpeed(double distance) {
        double rpmTarget = getShooterRPM(distance);
        double ticksPerSec = (rpmTarget / 60.0) * ticksPerRev;

        shooterRight.setVelocity(ticksPerSec);
        shooterLeft.setVelocity(ticksPerSec);
    }

    public void setShooterSpeed(int x) {
        double ticksPerSec = (x / 60.0) * ticksPerRev;
        shooterRight.setVelocity(ticksPerSec);
        shooterLeft.setVelocity(ticksPerSec);
    }

    public double getShooterCurrentRPM() {
        return shooterRight.getVelocity() * 60.0 / ticksPerRev;
    }

    public void stop() {
        shooterRight.setVelocity(0);
        shooterLeft.setVelocity(0);
    }
}
