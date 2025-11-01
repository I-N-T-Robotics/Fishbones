//package org.firstinspires.ftc.teamcode.Shooter;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.Constants.Field;
//import org.firstinspires.ftc.teamcode.Odometry.Odometry;
//import org.firstinspires.ftc.teamcode.Util.LinearRegression;
//
//import edu.wpi.first.math.geometry.Translation2d;
//
//public class ShooterHood {
//
//    private Servo hood;
//    private LinearRegression hoodAngle = null;
//
//    public ShooterHood(HardwareMap hardwareMap) {
//        hood = hardwareMap.get(Servo.class, "hood");
//    }
//
//    public void hoodRegression(double[] distances, double[] hoodAngles) {
//        if (distances.length != hoodAngles.length) {
//            throw new IllegalArgumentException("Angle array != distance array");
//        }
//
//        Translation2d[] hoodPoints = new Translation2d[hoodAngles.length];
//
//        for (int i = 0; i < hoodAngles.length; i++) {
//            hoodPoints[i] = new Translation2d(distances[i], hoodAngles[i]);
//        }
//
//        hoodAngle = new LinearRegression(hoodPoints);
//    }
//
//    public double getHoodAngle(double distance) {
//        return hoodAngle.calculatePoint(distance);
//    }
//
//    public void setHoodPosition(double distance) {
//        //fix to send real distance to correct tag
//        hood.setPosition(getHoodAngle(distance));
//    }
//}
