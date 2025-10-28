package org.firstinspires.ftc.teamcode.Constants;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public interface Settings {

    double DT = 1.0 / 50.0; //delta time, update time, 20ms

    double WIDTH = Units.inchesToMeters(16);
    double LENGTH = Units.inchesToMeters(16);

    public interface Swerve {
        public interface FrontRight {
            String DRIVE = "FrontRightDrive";
            String Turn = "FrontRightTurn";
            String DRIVE_ENCODER = "FrontRightDriveEncoder";
            String TURN_ENCODER = "FrontRightTurnEncoder";
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(130.4);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }
        public interface FrontLeft {
            String DRIVE = "FrontLeftDrive";
            String Turn = "FrontLeftTurn";
            String DRIVE_ENCODER = "FrontLeftDriveEncoder";
            String TURN_ENCODER = "FrontLeftTurnEncoder";
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(140.3);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }
        public interface BackRight {
            String DRIVE = "BackRightDrive";
            String Turn = "BackRightTurn";
            String DRIVE_ENCODER = "BackRightDriveEncoder";
            String TURN_ENCODER = "BackRightTurnEncoder";
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(164);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
        }
        public interface BackLeft {
            String DRIVE = "BackLeftDrive";
            String Turn = "BackLeftTurn";
            String DRIVE_ENCODER = "BackLeftDriveEncoder";
            String TURN_ENCODER = "BackLeftTurnEncoder";
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(172.2);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }
        double MODULE_VELOCITY_DEADBAND = 0.05;
    }

    public interface VisionConstants {
        public static String[] LIMELIGHT_NAME = new String[] {"limelight1"};

        public static Matrix<N3, N1> MT1_STD = VecBuilder.fill(0.7, 0.7, 0.7);
        public static Matrix<N3, N1> MT2_STD = VecBuilder.fill(0.5, 0.5, 0.7);

        public static AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(360);
        public static double AREA_THRESHOLD = 0.2;

        /*** LINEAR REGRESSION ***/

        // XY Standard Deviation vs Distance
        Translation2d[] xyStdDevs = new Translation2d[] {
                new Translation2d(0.5, 0.001368361309),
                new Translation2d(1, 0.001890508681),
                new Translation2d(1.5, 0.003221746028),
                new Translation2d(2, 0.009352868105),
                new Translation2d(2.5, 0.009364899366),
                new Translation2d(3, 0.01467209516),
                new Translation2d(3.5, 0.01837679393),
                new Translation2d(4, 0.03000858409),
                new Translation2d(4.5, 0.03192817984)
        };

        // Theta Standard Deviation vs Distance
        Translation2d[] thetaStdDevs = new Translation2d[] {
                new Translation2d(0.5, 0.2641393115),
                new Translation2d(1, 0.4433426481),
                new Translation2d(1.5, 0.660331025),
                new Translation2d(2, 0.6924061873),
                new Translation2d(2.5, 4.624662415),
                new Translation2d(3, 8.000007273),
                new Translation2d(3.5, 6.39384055),
                new Translation2d(4, 9.670544639),
                new Translation2d(4.5, 7.576406229)
        };
    }
}
