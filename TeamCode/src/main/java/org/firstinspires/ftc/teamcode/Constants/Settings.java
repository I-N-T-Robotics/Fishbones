package org.firstinspires.ftc.teamcode.Constants;

import edu.wpi.first.math.util.Units;

public interface Settings {

    double DT = 1.0 / 50.0; //delta time, update time, 20ms

    double WIDTH = Units.inchesToMeters(16);
    double LENGTH = Units.inchesToMeters(16);

    public interface Shooter {
        //Array of test values
        double[] distances = {0, 0, 0};
        double[] hoodAngles = {0, 0, 0};
        double[] shooterSpeeds = {0, 0, 0};

        double kP = 0.00005;
        double kI = 0.0;
        double kD = 0.00005;
        double kF = 0.000012;
    }

    public interface Intake {
        double shuffleOne = 0.3;
        double shuffleTwo = 0.6;
        double shuffleThree = 0.9;
    }
}
