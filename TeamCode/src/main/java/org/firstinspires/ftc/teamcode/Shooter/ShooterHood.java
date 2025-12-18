package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.Settings;
import org.firstinspires.ftc.teamcode.Util.LinearRegression;

import edu.wpi.first.math.geometry.Translation2d;

public class ShooterHood {

    private Servo hood;
    private LinearRegression hoodAngle;

    private double[] settingsDistance;
    private double[] settingsHoodAngles;

    public ShooterHood(HardwareMap hardwareMap) {
        hood = hardwareMap.get(Servo.class, "hood");

        settingsDistance = Settings.Shooter.distances;
        settingsHoodAngles = Settings.Shooter.hoodAngles;

        hoodRegression();
    }

    public void hoodRegression() {
        if (settingsDistance.length != settingsHoodAngles.length) {
            throw new IllegalArgumentException("Angle array != distance array");
        }

        Translation2d[] hoodPoints = new Translation2d[settingsHoodAngles.length];

        for (int i = 0; i < settingsHoodAngles.length; i++) {
            hoodPoints[i] = new Translation2d(settingsDistance[i], settingsHoodAngles[i]);
        }

        hoodAngle = new LinearRegression(hoodPoints);
    }

    public double getHoodAngle(double distance) {
        return hoodAngle.calculatePoint(distance);
    }

    public double getHoodCommandedAngle() {
        return hood.getPosition();
    }

    public double getHoodTargetAngle(double distance) {
        return getHoodAngle(distance);
    }

    public void setHoodPosition(double distance) {
        double pos = getHoodAngle(distance);
        pos = Math.max(0.0, Math.min(1.0, pos));
        hood.setPosition(pos);
    }
}
