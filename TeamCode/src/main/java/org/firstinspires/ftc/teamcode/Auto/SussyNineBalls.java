package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Shooter.ShooterHood;
import org.firstinspires.ftc.teamcode.Util.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.Util.actions.IntakeActions;
import org.firstinspires.ftc.teamcode.Util.actions.ShooterActions;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "nine", group = "Autos")
public class SussyNineBalls extends ActionOpMode {

    private boolean dariuscrackhead = false;

    private static class WaitAction {
        double triggerTime; // seconds into the waiting phase
        Action action;
        boolean triggered;

        WaitAction(double triggerTime, Action action) {
            this.triggerTime = triggerTime;
            this.action = action;
            this.triggered = false;
        }
    }

    private static class PathChainTask {
        PathChain pathChain;
        double waitTime; // how long to wait after the chain
        List<WaitAction> waitActions = new ArrayList<>();

        PathChainTask(PathChain pathChain, double waitTime) {
            this.pathChain = pathChain;
            this.waitTime = waitTime;
        }

        // Add a "wait action," triggered at a certain second in the WAIT phase
        PathChainTask addWaitAction(double triggerTime, Action action) {
            waitActions.add(new WaitAction(triggerTime, action));
            return this;
        }

        void resetWaitActions() {
            for (WaitAction wa : waitActions) {
                wa.triggered = false;
            }
        }
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private Intake intake;
    private Shooter shooter;

    private IntakeActions intakeActions;
    private ShooterActions shooterActions;

    private static final double PATH_COMPLETION_T = 0.982;

    private final List<PathChainTask> tasks = new ArrayList<>();
    private int currentTaskIndex = 0;

    private int taskPhase = 0;

    private final Pose startPose = new Pose(14.639, 105.431, Math.toRadians(0));

    public PathChain scorePreload, intakeFirst, scoreSecond, intakeSecond, scoreThird;

    private ParallelAction stopInsides;

    private void slowMode(boolean slow) {
        dariuscrackhead = slow;
    }

    public void buildPaths() {

        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(14.639, 103.883), new Pose(47.578, 96.704))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(132))
                .addTemporalCallback(0, () -> run(shooterActions.shooterA.shoot()))
                .addTemporalCallback(0, () -> run(intakeActions.intakeA.forwardIntake()))
                .build();

        intakeFirst = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(47.578, 96.704),
                                new Pose(56.164, 96.704),
                                new Pose(72.633, 87.132),
                                new Pose(16.469, 83.754)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(0))
                .build();

        scoreSecond = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.469, 83.754), new Pose(48.000, 96.141))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(132))
                .addParametricCallback(0.5, () -> run(shooterActions.shooterA.shoot()))
                .build();

        intakeSecond = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.000, 96.141),
                                new Pose(45.326, 64.469),
                                new Pose(68.974, 73.619),
                                new Pose(13.935, 57.150)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(0))
                .addTemporalCallback(0, () -> slowMode(true))
                //.setVelocityConstraint(.00005)
                //.setBrakingStrength(6)
               // .setGlobalDeceleration(4)

                .build();

        scoreThird = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(13.935, 57.150), new Pose(47.859, 96.141))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(132))
                .addParametricCallback(0.7, () -> run(shooterActions.shooterA.shoot()))
                .addTemporalCallback(0, () -> slowMode(false))
                .build();
    }

    private void buildTaskList() {
        tasks.clear();

        PathChainTask preloadTask = new PathChainTask(scorePreload, 3)
                .addWaitAction(0.1, new ParallelAction(intakeActions.intakeA.index(), intakeActions.intakeA.feed(), intakeActions.intakeA.holdNuts()))
                .addWaitAction(2.9, new ParallelAction(intakeActions.intakeA.stopFeed(), intakeActions.intakeA.forwardIntake()));
        tasks.add(preloadTask);

        PathChainTask intakeFirstTask = new PathChainTask(intakeFirst, 1);
        tasks.add(intakeFirstTask);

        PathChainTask scoreSecondTask = new PathChainTask(scoreSecond, 3)
                .addWaitAction(0.1, new ParallelAction(intakeActions.intakeA.index(), intakeActions.intakeA.feed(), intakeActions.intakeA.holdNuts()))
                .addWaitAction(2.9, new ParallelAction(intakeActions.intakeA.stopFeed(), intakeActions.intakeA.forwardIntake()));
        tasks.add(scoreSecondTask);

        PathChainTask intakeSecondTask = new PathChainTask(intakeSecond, 1);
        tasks.add(intakeSecondTask);

        PathChainTask scoreThirdTask = new PathChainTask(scoreThird, 3)
                .addWaitAction(0.1, new ParallelAction(intakeActions.intakeA.index(), intakeActions.intakeA.feed(), intakeActions.intakeA.holdNuts()))
                .addWaitAction(2.9, new ParallelAction(intakeActions.intakeA.stopFeed(), intakeActions.intakeA.forwardIntake()));
        tasks.add(scoreThirdTask);
    }

    private void runTasks() {
        if (currentTaskIndex >= tasks.size()) {
            return; // all done
        }

        PathChainTask currentTask = tasks.get(currentTaskIndex);

        switch (taskPhase) {
            case 0: // == DRIVING ==
                // If we aren't following yet, start
                if (!follower.isBusy()) {
                    follower.followPath(currentTask.pathChain, true);
                    pathTimer.resetTimer();

                    // We only "reset" the *wait* actions here.
                    // Param-based callbacks are attached in the chain already.
                    currentTask.resetWaitActions();
                }

                double tValue = follower.getCurrentTValue(); // param progress [0..1]
                // NOTE: Param-based callbacks happen automatically in the `Follower`
                // when tValue crosses the callback thresholds.

                // Consider chain done at 99%
                if (tValue >= PATH_COMPLETION_T) {
                    // Move to WAIT
                    pathTimer.resetTimer();
                    taskPhase = 1;
                }
                break;

            case 1: // == WAITING ==
                double waitElapsed = pathTimer.getElapsedTimeSeconds();

                // Trigger any "wait actions" whose time has arrived
                for (WaitAction wa : currentTask.waitActions) {
                    if (!wa.triggered && waitElapsed >= wa.triggerTime) {
                        run(wa.action); // schedule this action
                        wa.triggered = true;
                    }
                }

                // Once we've fully waited out the entire waitTime, move on
                if (waitElapsed >= currentTask.waitTime) {
                    currentTaskIndex++;
                    taskPhase = 0;
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, shooter, new ShooterHood(hardwareMap), new Limelight(hardwareMap));

        shooterActions = new ShooterActions(shooter);
        intakeActions = new IntakeActions(intake);
        stopInsides = new ParallelAction(intakeActions.intakeA.stopIndex(), intakeActions.intakeA.stopFeed());


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        buildTaskList();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();

        follower.update();

        if (dariuscrackhead) {
            follower.setMaxPower(.65);
        } else {
            follower.setMaxPower(1);
        }

        runTasks();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("isbusy?", follower.isBusy());
        telemetry.addData("progression:", follower.getCurrentTValue());
        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("Wait Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Running Actions", runningActions.size());

        telemetry.update();
    }

    @Override
    public void init_loop() {
        super.loop();

        runTasks();

    }
}

