package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Truly Not That Deep", group = "Victor Go Brr")

public class BlueAutoNotThatDeep extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose start = new Pose(25.35, 129.65, Math.toRadians(144)); // Start pose of our robot
    private final Pose score = new Pose(60.25, 84, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose intake1End = new Pose(17, 83, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose intake1Start = intake1End.withX(50); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose intake2End = new Pose(17, 59, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose intake2Start = intake2End.withX(50); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose intake2Ctrl = new Pose(70, 72);
    private final Pose intake3End = new Pose(17, 35, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose intake3Start = intake3End.withX(50); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose intake3Ctrl = new Pose(70, 35);
    private final Pose park = new Pose(22.5, 72, Math.toRadians(0));

    private PathChain startToScore, scoreToIntake1Start, intake1StartToIntake1End, intake1EndToScore, scoreToIntake2Start, intake2StartToIntake2End, intake2EndToScore, scoreToIntake3Start, intake3StartToIntake3End, intake3EndToScore, scoreToPark;

    private Intake m_intake;
    private Shooter m_shooter;

    public void buildPaths() {
        // Intake enable and shooter rev is done using temporal callbacks
        startToScore = follower.pathBuilder()
                .addPath(new BezierLine(start, score))
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading(), 0.75)
                .addTemporalCallback(0, () -> {m_shooter.shootPower(0.25);}) // Enable shooter, remains on for entire auto
                .addParametricCallback(1, () -> {actionTimer.resetTimer();})
                .build();

        // Intake and score first artifact group
        scoreToIntake1Start = follower.pathBuilder()
                .addPath(new BezierLine(score, intake1Start))
                .setLinearHeadingInterpolation(score.getHeading(), intake1Start.getHeading(), 0.9)
                .addTemporalCallback(0, () -> {m_intake.enable();}) // Enable intake and run motor outake in reverse. Intake stays on
                .addPath(new BezierLine(intake1Start, intake1End))
                .setConstantHeadingInterpolation(intake1End.getHeading())
                .addParametricCallback(1, () -> {actionTimer.resetTimer();})
                .build();
        intake1EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(intake1End, score))
                .setLinearHeadingInterpolation(intake1End.getHeading(), score.getHeading(), 0.75)
                .addParametricCallback(1, () -> {actionTimer.resetTimer();})
                .build();

        // Intake and score second artifact group
        scoreToIntake2Start = follower.pathBuilder()
                .addPath(new BezierLine(score, intake2Start))
                .setLinearHeadingInterpolation(score.getHeading(), intake2Start.getHeading(), 0.9)
                .addTemporalCallback(0, () -> {m_intake.enable();})
                .addPath(new BezierLine(intake2Start, intake2End))
                .setConstantHeadingInterpolation(intake2End.getHeading())
                .addParametricCallback(1, () -> {actionTimer.resetTimer();})
                .build();
        intake2EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(intake2End, score))
                .setLinearHeadingInterpolation(intake2End.getHeading(), score.getHeading(), 0.75)
                .addParametricCallback(1, () -> {actionTimer.resetTimer();})
                .build();

        // Intake and score third artifact group
        scoreToIntake3Start = follower.pathBuilder()
                .addPath(new BezierLine(score, intake3Start))
                .setLinearHeadingInterpolation(score.getHeading(), intake3Start.getHeading(), 0.9)
                .addTemporalCallback(0, () -> {m_intake.enable();})
                .addPath(new BezierLine(intake3Start, intake3End))
                .setConstantHeadingInterpolation(intake3End.getHeading())
                .addParametricCallback(1, () -> {actionTimer.resetTimer();})
                .build();
        intake3EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(intake3End, score))
                .setLinearHeadingInterpolation(intake3End.getHeading(), score.getHeading(), 0.75)
                .addParametricCallback(1, () -> {actionTimer.resetTimer();})
                .build();

        // Park in front of gate in prep for teleop. Gets robot off launch line for auto leave
        scoreToPark = follower.pathBuilder()
                .addPath(new BezierLine(score, park))
                .setLinearHeadingInterpolation(score.getHeading(), park.getHeading())
                .addParametricCallback(1, () -> {actionTimer.resetTimer();})
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Go to scoring position
                follower.followPath(startToScore);
                setPathState(1);
                break;
            case 1: // Score preload then go to start intake 1
                if (!follower.isBusy()) {
                    m_intake.feed();

                    if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                        follower.followPath(scoreToIntake1Start, 0.85, true); // Path has temporal callback to reset the motor outake to intake power
                        setPathState(2);
                    }
                }
                break;
            case 2: // Hold at the end of intake path for 2 seconds and then go to score. 10 lines of code for wait command trollDespair
                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(intake1EndToScore, true);
                    setPathState(3);
                }
                break;
            case 3: // Score and then go to intake 2
                if(!follower.isBusy()) {
                    m_intake.feed();

                    if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                        follower.followPath(scoreToIntake2Start, 0.85, true); // Path has temporal callback to reset the motor outake to intake power
                        setPathState(4);
                    }
                }
                break;
            case 4: // Hold at the end of intake path for 2 seconds and then go to score
                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(intake2EndToScore, true);
                    setPathState(5);
                }
                break;
            case 5: // Score and then go to intake 3
                if(!follower.isBusy()) {
                    m_intake.feed();

                    if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                        follower.followPath(scoreToIntake3Start, 0.85, true); // Path has temporal callback to reset the motor outake to intake power
                        setPathState(6);
                    }
                }
                break;
            case 6: // Hold at the end of intake path for 2 seconds and then go to score
                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(intake3EndToScore, true);
                    setPathState(7);
                }
                break;
            case 7: // Score and then go park in front of gate
                if(!follower.isBusy()) {
                    m_intake.feed();

                    if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                        follower.followPath(scoreToPark, true); // Path has temporal callback to reset the motor outake to intake power
                        setPathState(-1);
                    }
                }
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        m_intake = new Intake(hardwareMap);
        m_shooter = new Shooter(hardwareMap);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(start);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower State", follower.isBusy());
        telemetry.addData("Action Timer", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}