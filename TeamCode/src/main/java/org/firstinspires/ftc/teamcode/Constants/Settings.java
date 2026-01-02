package org.firstinspires.ftc.teamcode.Constants;

public interface Settings {
    public interface Shooter {
        //Array of test values
        double[] distances = {0.472, 0.587, 0.67, 0.714, 0.79, 0.875,
                0.941, 1, 1.10, 1.19, 1.26, 1.53, 1.74, 1.88, 1.97, 2.08}; //meters
        double[] hoodAngles = {0.25, 0.25, 0.25, 0.25, 0.27, 0.28,
                0.3, 0.34, 0.34, 0.37, 0.39, 0.39, 0.4, 0.4, 0.4, 0.35}; //degrees
        double[] shooterSpeeds = {95, 80, 75, 75, 75, 80, 85, 85,
                85, 90, 95, 110, 125, 135, 145, 155}; //rpm

//        double kP = 0.00005;
        double kP = 90;
        double kI = 0.0;
        double kD = 0.00005;
        double kF = 0.000012;
    }
}