package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Shooter.ShooterHood;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "testBlueNine", group = "Autos")
public class testBlueNine extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Intake intake;
    private Shooter shooter;
    private ShooterHood shooterHood;

    public enum PathState {
        DRIVESTART_TO_SHOOTPRELOAD,
        SHOOT_PRELOAD,
        SHOOTPRELOAD_TO_INTAKE1,
        INTAKE1_TO_SHOOT1,
        SHOOT1_TO_INTAKE2,
        INTAKE2_TO_SHOOT2
    }

    PathState pathState = PathState.DRIVESTART_TO_SHOOTPRELOAD;

    private final Pose start = new Pose(23.225806451612904, 126.38709677419355, Math.toRadians(144));
    private final Pose shootPreload = new Pose(47.225806451612904, 96.00000000000001, Math.toRadians(132));
    private final Pose intake1 = new Pose(13.741935483870968, 83.61290322580643, Math.toRadians(180));
    private final Pose intake1Assist = new Pose(52.45161290322581, 77.22580645161291);
    private final Pose shoot1 = new Pose(47.225806451612904, 96.00000000000001, Math.toRadians(132));
    private final Pose intake2 = new Pose(8.32258064516129, 57.677419354838705, Math.toRadians(180));
    private final Pose intake2Assist = new Pose(74.90322580645162, 54.774193548387096);
    private final Pose shoot2 = new Pose(47.225806451612904, 96.00000000000001, Math.toRadians(132));
    private final Pose shoot2Assist = new Pose(30.774193548387096, 54);

    private PathChain startToShootPreload,
    preloadToIntake1,
    intake1ToShoot1,
    shoot1ToIntake2,
    intake2ToShoot2;

    public void buildPaths() {
        startToShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(start, shootPreload))
                .setLinearHeadingInterpolation(start.getHeading(), shootPreload.getHeading())
                .build();
        preloadToIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPreload, intake1Assist, intake1))
                .setLinearHeadingInterpolation(shootPreload.getHeading(), intake1.getHeading())
                .build();
        intake1ToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, shoot1))
                .setLinearHeadingInterpolation(intake1.getHeading(), shoot1.getHeading())
                .build();
        shoot1ToIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1, intake2Assist, intake2))
                .setLinearHeadingInterpolation(shoot1.getHeading(), intake2.getHeading())
                .build();
        intake2ToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake2, shoot2Assist, shoot2))
                .setLinearHeadingInterpolation(intake2.getHeading(), shoot2.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVESTART_TO_SHOOTPRELOAD:
                follower.followPath(startToShootPreload, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() < 1) {
                        intake.state = Intake.States.SHOOTING;
                    }
                    setPathState(PathState.SHOOTPRELOAD_TO_INTAKE1);
                }
                break;

            case SHOOTPRELOAD_TO_INTAKE1:
                if (!follower.isBusy()) {
                    intake.frontIntake();
                    intake.backIntake();
                    intake.state = Intake.States.OFF;
                    follower.followPath(preloadToIntake1, true);
                    setPathState(PathState.INTAKE1_TO_SHOOT1);
                }
                break;

            case INTAKE1_TO_SHOOT1:
                if (!follower.isBusy()) {
                    intake.frontIntakeStop();
                    intake.backIntakeStop();
                    follower.followPath(intake1ToShoot1, true);
                    setPathState(PathState.SHOOT1_TO_INTAKE2);
                }
                break;

            case SHOOT1_TO_INTAKE2:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() < 1) {
                        intake.state = Intake.States.SHOOTING;
                    }
                    intake.frontIntake();
                    intake.backIntake();
                    intake.state = Intake.States.OFF;
                    follower.followPath(shoot1ToIntake2, true);
                    setPathState(PathState.INTAKE2_TO_SHOOT2);
                }
                break;

            case INTAKE2_TO_SHOOT2:
                if(!follower.isBusy()) {
                    intake.frontIntakeStop();
                    intake.backIntakeStop();
                    follower.followPath(intake2ToShoot2, true);
                    if(!follower.isBusy()) {
                        intake.state = Intake.States.SHOOTING;
                    }
                }
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

        intake = new Intake(hardwareMap, shooter, shooterHood);
        shooter = new Shooter(hardwareMap);
        shooterHood = new ShooterHood(hardwareMap);

        buildPaths();
        follower.setStartingPose(start);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
    }
}