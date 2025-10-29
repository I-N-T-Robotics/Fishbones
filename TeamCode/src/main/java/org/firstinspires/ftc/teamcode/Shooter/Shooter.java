package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.Field;
import org.firstinspires.ftc.teamcode.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.Util.LinearRegression;

import edu.wpi.first.math.geometry.Translation2d;

public class Shooter {

    private DcMotorEx shooterRight, shooterLeft;
    private LinearRegression rpm = null;

    public Shooter(HardwareMap hardwareMap) {
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");

        shooterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rpmRegression(double[] distances, double[] shooterSpeeds) {
        if (distances.length != shooterSpeeds.length) {
            throw new IllegalArgumentException("Shooter array != distance array");
        }

        Translation2d[] rpmPoints = new Translation2d[shooterSpeeds.length];

        for (int i = 0; i < shooterSpeeds.length; i++) {
            rpmPoints[i] = new Translation2d(distances[i], shooterSpeeds[i]);
        }

        rpm = new LinearRegression(rpmPoints);
    }

    public double getShooterRPM(double distance) {
        return rpm.calculatePoint(distance);
    }

    public void setShooterSpeed(double distance) {
        //fix to send real distance to correct tag
        shooterRight.setVelocity(getShooterRPM(distance));
        shooterLeft.setVelocity(getShooterRPM(distance));
    }
}
