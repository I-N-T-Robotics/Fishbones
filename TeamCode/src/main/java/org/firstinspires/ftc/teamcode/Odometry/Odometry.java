package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Field;
import org.firstinspires.ftc.teamcode.Constants.Settings;
import org.firstinspires.ftc.teamcode.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Util.LinearRegression;
import org.firstinspires.ftc.teamcode.Util.VisionUtil.AprilTag;
import org.firstinspires.ftc.teamcode.Util.VisionUtil.VisionData;
import org.firstinspires.ftc.teamcode.Vision.AprilTagVision;

import java.util.ArrayList;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N3;


public class Odometry {

    private static Odometry instance;
    private Telemetry telemetry;

//    public final void debug() {
//        telemetry.addData("FirstOdometryCall", "FirstOdometryCall");
//        telemetry.update();
//    }

//    static {
//        instance = new Odometry();
//    }

    public static Odometry createInstance(HardwareMap hardwareMap, Telemetry telemetry) {
        if (instance == null) {
            instance = new Odometry(hardwareMap, telemetry);
        }
        return instance;
    }

    public static Odometry getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Odometry not initialized");
        }
        return instance;
    }

    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator estimator;
    private Boolean VISION_ACTIVE;

//    private final Field2d field;
//    private final FieldObject2d odometryPose2D;
//    private final FieldObject2d estimatorPose2D;

    private final LinearRegression xyRegression;
    private final LinearRegression thetaRegression;

    private Pose2d lastGoodPose;

    private Translation2d robotVelocity;
    private Translation2d lastPose;

    private Odometry(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry = new SwerveDriveOdometry(
                swerve.getKinematics(),
                swerve.getGyroAngle(),
                swerve.getModulePositions(),
                new Pose2d());
        estimator = new SwerveDrivePoseEstimator(
                swerve.getKinematics(),
                swerve.getGyroAngle(),
                swerve.getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1), //std of pose estimate
                VecBuilder.fill(0.9, 0.9, 10.0)); //std of vision estimate

        VISION_ACTIVE = true;
        telemetry.addData("Vision Active", VISION_ACTIVE);

//        field = new Field2d();
//        swerve.initFieldObject(field);
//        odometryPose2D = field.getObject("Odometry Pose");
//        estimatorPose2D = field.getObject("Estimator Pose");

        xyRegression = new LinearRegression(Settings.VisionConstants.xyStdDevs);
        thetaRegression = new LinearRegression(Settings.VisionConstants.thetaStdDevs);

        lastGoodPose = new Pose2d();

        robotVelocity = new Translation2d();
        lastPose = new Translation2d();

//        telemetry.addData("Field", field);
    }

    public void setVisionEnabled(boolean enabled) {
        VISION_ACTIVE = enabled;
    }

//    public Field2d getField() {
//        return field;
//    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public double getDistanceToTag(AprilTag tag) {
        return getPose()
                .getTranslation()
                .getDistance(tag.getLocation().getTranslation().toTranslation2d());
    }

    public void reset(Pose2d pose) {
        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry.resetPosition(swerve.getGyroAngle(), swerve.getModulePositions(), pose);
        estimator.resetPosition(swerve.getGyroAngle(), swerve.getModulePositions(), pose);
    }

    private void updateOdometry() {
        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry.update(swerve.getGyroAngle(), swerve.getModulePositions());
        estimator.update(swerve.getGyroAngle(), swerve.getModulePositions());
    }

    private Vector<N3> getStandardDeviation(VisionData data) {
        double distance = data.getDistanceToPrimaryTag();

        double xyStdDev = xyRegression.calculatePoint(distance);
        double thetaStdDev = thetaRegression.calculatePoint(distance);

        telemetry.addData("xyStdDev", xyStdDev);
        telemetry.addData("thetaStdDev", thetaStdDev);

        return VecBuilder.fill(
                xyStdDev,
                xyStdDev,
                thetaStdDev);
    }

    private void updateEstimatorWithVisionData(ArrayList<VisionData> outputs) {
        Pose2d poseSum = new Pose2d();
        double timestampSum = 0;
        double areaSum = 0;

        for (VisionData data : outputs) {
            Pose2d weighted = data.getPose().times(data.getArea());

            poseSum = new Pose2d(
                    poseSum.getTranslation().plus(weighted.getTranslation()),
                    poseSum.getRotation().plus(weighted.getRotation())
            );

            areaSum += data.getArea();

            timestampSum += data.getTimestamp() * data.getArea();
        }

        estimator.addVisionMeasurement(poseSum.div(areaSum), timestampSum/areaSum, VecBuilder.fill(0.7, 0.7, 10));
    }

    public void periodic() {
        ArrayList<VisionData> outputs = AprilTagVision.getOutputs();

        updateOdometry();

        if(VISION_ACTIVE && !outputs.isEmpty()) {
            updateEstimatorWithVisionData(outputs);
        }

        if(estimator.getEstimatedPosition().getTranslation().getNorm() > new Translation2d(Field.LENGTH, Field.WIDTH).getNorm() ||
            odometry.getPoseMeters().getTranslation().getNorm() > new Translation2d(Field.LENGTH, Field.WIDTH).getNorm() ||
                Double.isNaN(estimator.getEstimatedPosition().getX()) || Double.isNaN(estimator.getEstimatedPosition().getY()) ||
                Double.isNaN(odometry.getPoseMeters().getX()) || Double.isNaN(odometry.getPoseMeters().getY())
        ) {
            reset(lastGoodPose);
        } else {
            lastGoodPose = getPose();
        }

        robotVelocity = getPose().getTranslation().minus(lastPose).div(Settings.DT);
        lastPose = getPose().getTranslation();

        telemetry.addData("Odometry X", odometry.getPoseMeters().getTranslation().getX());
        telemetry.addData("Odometry Y", odometry.getPoseMeters().getTranslation().getY());
        telemetry.addData("Odometry Rotation", odometry.getPoseMeters().getRotation().getDegrees());

        telemetry.addData("Estimator Odometry X", estimator.getEstimatedPosition().getTranslation().getX());
        telemetry.addData("Estimator Odometry Y", estimator.getEstimatedPosition().getTranslation().getY());
        telemetry.addData("Estimator Odometry Rotation", estimator.getEstimatedPosition().getRotation().getDegrees());

        telemetry.addData("Estimator VX", robotVelocity.getX());
        telemetry.addData("Estimator VY", robotVelocity.getX());

//        odometryPose2D.setPose(odometry.getPoseMeters());
//        estimatorPose2D.setPose(estimator.getEstimatedPosition());
    }

    public Translation2d getRobotVelocity() {
        return robotVelocity;
    }
}