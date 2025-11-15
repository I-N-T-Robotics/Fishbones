package org.firstinspires.ftc.teamcode.Shooter;

import org.firstinspires.ftc.teamcode.Constants.Field;
import org.firstinspires.ftc.teamcode.Constants.Settings;
import org.firstinspires.ftc.teamcode.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Util.Clamp;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPose extends Command {

    public DriveToPose(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    public static DriveToPose goalRelative(double angleToGoal, double distanceToGoal) {
        Rotation2d angle = Rotation2d.fromDegrees(Clamp.clamp(angleToGoal, Settings.Alignment.SHOOT_ANGLE));

        double distance;
        if (distanceToGoal >= 0 && distanceToGoal <= Settings.Alignment.BIG_TRIANGLE_POINT) {
            distance = Clamp.clamp(distanceToGoal, Settings.Alignment.BIG_TRIANGLE_POINT);
        } else if (distanceToGoal >= Settings.Alignment.SMALL_TRIANGLE_POINT && distanceToGoal <= Settings.Alignment.SMALL_TRIANGLE_END) {
            distance = Clamp.clamp(distanceToGoal, Settings.Alignment.SMALL_TRIANGLE_POINT, Settings.Alignment.SMALL_TRIANGLE_END);
        } else {
            double nearestShort = Clamp.clamp(distanceToGoal, 0, Settings.Alignment.BIG_TRIANGLE_POINT);
            double nearestLong = Clamp.clamp(distanceToGoal, Settings.Alignment.SMALL_TRIANGLE_POINT, Settings.Alignment.SMALL_TRIANGLE_END);

            if (Math.abs(distanceToGoal - nearestShort) <
                    Math.abs(distanceToGoal - nearestLong)) {
                distance = nearestShort;
            } else {
                distance = nearestLong;
            }
        }

        return new DriveToPose(() -> new Pose2d(
                Field.getAllianceGoalPose().getTranslation()
                        .plus(new Translation2d(distance, angle)),
                angle
        ));
    }

    private SwerveDrive swerve;
    private Odometry odometry;

    private HolonomicDriveController controller;
    private BooleanSupplier isAligned;
    private IntStream velocityError;
    private Supplier<Pose2d> poseSupplier;

    private double xTolerance;
    private double yTolerance;
    private double thetaTolerance;
    private double velTolerance;
    private double maxSpeed;

    private Pose2d targetPose;

    public DriveToPose(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public DriveToPose(Supplier<Pose2d> poseSupplier) {
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        this.poseSupplier = poseSupplier;

        maxSpeed = Settings.Swerve.MAX_SPEED;

        controller = new HolonomicDriveController(
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0),
                new ProfiledPIDController(0, 0, 0,
                        new TrapezoidProfile.Constraints(maxSpeed, 10)));

        isAligned = BooleanSupplier(this::isAligned)
    }
}