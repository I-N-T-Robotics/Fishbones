package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Shooter.ShooterHood;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "finalBlueAuto", group = "Autos")
public class finalBlueAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Limelight limelight;
    private Intake intake;
    private Shooter shooter;
    private ShooterHood shooterHood;

    public enum PathState{
        DRIVESTART_TO_SHOOTPRELOAD,
        SHOOT_PRELOAD,
        SHOOTPRELOAD_TO_INTAKE1,
        INTAKE1_TO_SHOOT1,
        SHOOT1_TO_INTAKE2,
        INTAKE2_TO_SHOOT2,
        SHOOT2_TO_INTAKE3,
        INTAKE3_TO_SHOOT3,
        SHOOT3_TO_INTAKE4,
        INTAKE4_TO_SHOOT4
    }

    PathState pathState;

    private final Pose start = new Pose(20.903225806451616, 98.9032258064516, Math.toRadians(135));
    private final Pose shootPreload = new Pose(44.12903225806452, 98.9032258064516, Math.toRadians(135));
    private final Pose intake1 = new Pose(14.70967741935484, 87.09677419354838, Math.toRadians(180));
    private final Pose intake1Assist = new Pose(49.5483870967742, 72.19354838709677);
    private final Pose shoot1 = new Pose(51.67741935483871, 82.06451612903226, Math.toRadians(128));
    private final Pose intake2 = new Pose(14.516129032258064, 67.74193548387098, Math.toRadians(180));
    private final Pose intake2Assist1 = new Pose(63.096774193548384, 64.45161290322581);
    private final Pose intake2Assist2 = new Pose(26.516129032258064, 44.32258064516128);
    private final Pose shoot2 = new Pose(56.516129032258064, 16.645161290322584, Math.toRadians(112));
    private final Pose shoot2Assist = new Pose(38.32258064516129, 52.25806451612903);
    private final Pose intake3 = new Pose(7.935483870967742, 34.645161290322584, Math.toRadians(180));
    private final Pose intake3Assist = new Pose(45.29032258064516, 36.38709677419355);
    private final Pose shoot3 = new Pose(56.516129032258064, 16.645161290322584, Math.toRadians(112));
    private final Pose shoot3Assist = new Pose(45.29032258064516, 36.38709677419355);
    private final Pose intake4 = new Pose(8.70967741935484, 7.935483870967735, Math.toRadians(270));
    private final Pose intake4Assist = new Pose(7.548387096774194, 33.09677419354839);
    private final Pose shoot4 = new Pose(56.70967741935483, 16.451612903225808, Math.toRadians(112));
    private final Pose shoot4Assist = new Pose(27.677419354838708, 26.903225806451605);

    private PathChain startToShootPreload,
            shootPreloadToIntake1,
            intake1ToShoot1,
            shoot1ToIntake2,
            intake2ToShoot2,
            shoot2ToIntake3,
            intake3ToShoot3,
            shoot3ToIntake4,
            intake4ToShoot4;

    public void buildPaths() {
        startToShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(start, shootPreload))
                .setLinearHeadingInterpolation(start.getHeading(), shootPreload.getHeading())
                .build();
        shootPreloadToIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPreload, intake1Assist, intake1))
                .setLinearHeadingInterpolation(shootPreload.getHeading(),intake1.getHeading())
                .build();
        intake1ToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, shoot1))
                .setLinearHeadingInterpolation(intake1.getHeading(), shoot1.getHeading())
                .build();
        shoot1ToIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1, intake2Assist1, intake2Assist2, intake2))
                .setLinearHeadingInterpolation(shoot1.getHeading(), intake2.getHeading())
                .build();
        intake2ToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake2, shoot2Assist, shoot2))
                .setLinearHeadingInterpolation(intake2.getHeading(), shoot2.getHeading())
                .build();
        shoot2ToIntake3 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot2, intake3Assist, intake3))
                .setLinearHeadingInterpolation(shoot2.getHeading(), intake3.getHeading())
                .build();
        intake3ToShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(intake3, shoot3Assist, shoot3))
                .setLinearHeadingInterpolation(intake3.getHeading(), shoot3.getHeading())
                .build();
        shoot3ToIntake4 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot3, intake4Assist, intake4))
                .setLinearHeadingInterpolation(shoot3.getHeading(), intake4.getHeading())
                .build();
        intake4ToShoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(intake4, shoot4Assist, shoot4))
                .setLinearHeadingInterpolation(intake4.getHeading(), shoot4.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVESTART_TO_SHOOTPRELOAD:
                follower.followPath(startToShootPreload, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if(!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() < 1) {
                        shooter.setShooterSpeed(limelight.getDistance());
                        shooterHood.setHoodPosition(limelight.getDistance());
                        intake.state = Intake.States.SHOOTING;
                    }
                    setPathState(PathState.SHOOTPRELOAD_TO_INTAKE1);
                }
                break;

            case SHOOTPRELOAD_TO_INTAKE1:
                if(!follower.isBusy()) {
                    intake.frontIntake();
                    intake.backIntake();
                    intake.state = Intake.States.OFF;
                    follower.followPath(shootPreloadToIntake1, true);
                    setPathState(PathState.INTAKE1_TO_SHOOT1);
                }
                break;

            case INTAKE1_TO_SHOOT1:
                if(!follower.isBusy()) {
                    intake.frontIntakeStop();
                    intake.backIntakeStop();
                    follower.followPath(intake1ToShoot1, true);
                    setPathState(PathState.SHOOT1_TO_INTAKE2);
                }
                break;

            case SHOOT1_TO_INTAKE2:
                if(!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() < 1) {
                        shooter.setShooterSpeed(limelight.getDistance());
                        shooterHood.setHoodPosition(limelight.getDistance());
                        intake.state = Intake.States.SHOOTING;
                    }
                    intake.frontIntake();
                    intake.backIntake();
                    intake.state = Intake.States.NOTHING;
                    follower.followPath(shoot1ToIntake2, true);
                    setPathState(PathState.INTAKE2_TO_SHOOT2);
                }
                break;

            case INTAKE2_TO_SHOOT2:
                if(!follower.isBusy()) {
                    intake.frontIntakeStop();
                    intake.backIntakeStop();
                    follower.followPath(intake2ToShoot2, true);
                    setPathState(PathState.SHOOT2_TO_INTAKE3);
                }
                break;

            case SHOOT2_TO_INTAKE3:
                if(!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() < 1) {
                        shooter.setShooterSpeed(limelight.getDistance());
                        shooterHood.setHoodPosition(limelight.getDistance());
                        intake.state = Intake.States.SHOOTING;
                    }
                    intake.frontIntake();
                    intake.backIntake();
                    intake.state = Intake.States.NOTHING;
                    follower.followPath(shoot2ToIntake3, true);
                    setPathState(PathState.INTAKE3_TO_SHOOT3);
                }
                break;

            case INTAKE3_TO_SHOOT3:
                if(!follower.isBusy()) {
                    intake.frontIntakeStop();
                    intake.backIntakeStop();
                    follower.followPath(intake3ToShoot3, true);
                    setPathState(PathState.SHOOT3_TO_INTAKE4);
                }
                break;

            case SHOOT3_TO_INTAKE4:
                if(!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() < 1) {
                        shooter.setShooterSpeed(limelight.getDistance());
                        shooterHood.setHoodPosition(limelight.getDistance());
                        intake.state = Intake.States.SHOOTING;
                    }
                    intake.frontIntake();
                    intake.backIntake();
                    intake.state = Intake.States.NOTHING;
                    follower.followPath(shoot3ToIntake4, true);
                    setPathState(PathState.INTAKE4_TO_SHOOT4);
                }
                break;

            case INTAKE4_TO_SHOOT4:
                if(!follower.isBusy()) {
                    intake.frontIntakeStop();
                    intake.backIntakeStop();
                    follower.followPath(intake4ToShoot4, true);
                    if(!follower.isBusy()) {
                        shooter.setShooterSpeed(limelight.getDistance());
                        shooterHood.setHoodPosition(limelight.getDistance());
                        intake.state = Intake.States.SHOOTING;
                    }
                }
                break;

            default:
                telemetry.addLine("broken");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVESTART_TO_SHOOTPRELOAD;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        intake = new Intake(hardwareMap, shooter, shooterHood, limelight);
        shooter = new Shooter(hardwareMap);
        shooterHood = new ShooterHood(hardwareMap);

        limelight = new Limelight(hardwareMap);

        Intake.resetTagID(hardwareMap);

        buildPaths();
        follower.setPose(start);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        int detectedTag = limelight.getTagID();
        Intake.saveTagID(hardwareMap, detectedTag);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
    }
}
