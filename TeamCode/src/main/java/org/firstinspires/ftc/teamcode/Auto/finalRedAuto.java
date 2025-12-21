package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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

@Autonomous(name = "finalRedAuto", group = "Autos")
public class finalRedAuto extends ActionOpMode {

    private static class WaitAction {
        double triggerTime;
        Action action;
        boolean triggered = false;

        WaitAction(double triggerTime, Action action) {
            this.triggerTime = triggerTime;
            this.action = action;
        }
    }

    private static class PathChainTask {
        PathChain path;
        double waitTime;
        List<WaitAction> waitActions = new ArrayList<>();

        PathChainTask(PathChain path, double waitTime) {
            this.path = path;
            this.waitTime = waitTime;
        }

        PathChainTask addWaitAction(double triggerTime, Action action) {
            waitActions.add(new WaitAction(triggerTime, action));
            return this;
        }

        void reset() {
            for (WaitAction wa : waitActions) {
                wa.triggered = false;
            }
        }
    }

    private static final double PATH_COMPLETION_T = 0.985;

    private Follower follower;
    private Timer pathTimer;

    private Intake intake;
    private Shooter shooter;
    private IntakeActions intakeActions;
    private ShooterActions shooterActions;

    private final List<PathChainTask> tasks = new ArrayList<>();
    private int currentTaskIndex = 0;
    private int taskPhase = 0; // 0 = DRIVE, 1 = WAIT

    private final Pose start = new Pose(20.903225806451616, 98.9032258064516, Math.toRadians(-36));

    private final Pose shootPreload = new Pose(44.12903225806452, 98.9032258064516, Math.toRadians(143));
    private final Pose shootPreloadAssist = new Pose(76.64516129032258, 89.2258064516129);

    private final Pose intake1 = new Pose(14.70967741935484, 87.09677419354838, Math.toRadians(180));
    private final Pose intake1Assist = new Pose(49.5483870967742, 72.19354838709677);

    private final Pose shoot1Pose = new Pose(51.67741935483871, 82.06451612903226, Math.toRadians(128));

    private final Pose intake2 = new Pose(14.516129032258064, 67.74193548387098, Math.toRadians(180));
    private final Pose intake2Assist1 = new Pose(63.096774193548384, 64.45161290322581);
    private final Pose intake2Assist2 = new Pose(26.516129032258064, 44.32258064516128);

    private final Pose shoot2Pose = new Pose(56.516129032258064, 16.645161290322584, Math.toRadians(112));
    private final Pose shoot2Assist = new Pose(38.32258064516129, 52.25806451612903);

    private final Pose intake3 = new Pose(7.935483870967742, 34.645161290322584, Math.toRadians(180));
    private final Pose intake3Assist = new Pose(45.29032258064516, 36.38709677419355);

    private final Pose shoot3Pose = new Pose(56.516129032258064, 16.645161290322584, Math.toRadians(112));
    private final Pose shoot3Assist = new Pose(45.29032258064516, 36.38709677419355);

    private final Pose intake4 = new Pose(8.70967741935484, 7.935483870967735, Math.toRadians(270));
    private final Pose intake4Assist = new Pose(7.548387096774194, 33.09677419354839);

    private final Pose shoot4Pose = new Pose(56.70967741935483, 16.451612903225808, Math.toRadians(112));
    private final Pose shoot4Assist = new Pose(27.677419354838708, 26.903225806451605);

    private Paths paths;

    public class Paths {

        public PathChain preload;
        public PathChain intake1P, shoot1P;
        public PathChain intake2P, shoot2P;
        public PathChain intake3P, shoot3P;
        public PathChain intake4P, shoot4P;

        Paths(Follower follower) {

            preload = follower.pathBuilder()
                    .addPath(new BezierCurve(start, shootPreloadAssist, shootPreload))
                    .setLinearHeadingInterpolation(start.getHeading(), shootPreload.getHeading())
                    .build();

            intake1P = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPreload, intake1Assist, intake1))
                    .setLinearHeadingInterpolation(shootPreload.getHeading(), intake1.getHeading())
                    .build();

            shoot1P = follower.pathBuilder()
                    .addPath(new BezierCurve(intake1, intake1Assist, shoot1Pose))
                    .setLinearHeadingInterpolation(intake1.getHeading(), shoot1Pose.getHeading())
                    .build();

            intake2P = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot1Pose, intake2Assist1, intake2Assist2, intake2))
                    .setLinearHeadingInterpolation(shoot1Pose.getHeading(), intake2.getHeading())
                    .build();

            shoot2P = follower.pathBuilder()
                    .addPath(new BezierCurve(intake2, shoot2Assist, shoot2Pose))
                    .setLinearHeadingInterpolation(intake2.getHeading(), shoot2Pose.getHeading())
                    .build();

            intake3P = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot2Pose, intake3Assist, intake3))
                    .setLinearHeadingInterpolation(shoot2Pose.getHeading(), intake3.getHeading())
                    .build();

            shoot3P = follower.pathBuilder()
                    .addPath(new BezierCurve(intake3, shoot3Assist, shoot3Pose))
                    .setLinearHeadingInterpolation(intake3.getHeading(), shoot3Pose.getHeading())
                    .build();

            intake4P = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot3Pose, intake4Assist, intake4))
                    .setLinearHeadingInterpolation(shoot3Pose.getHeading(), intake4.getHeading())
                    .build();

            shoot4P = follower.pathBuilder()
                    .addPath(new BezierCurve(intake4, shoot4Assist, shoot4Pose))
                    .setLinearHeadingInterpolation(intake4.getHeading(), shoot4Pose.getHeading())
                    .build();
        }
    }

    private void buildTaskList() {
        tasks.clear();

        tasks.add(new PathChainTask(paths.preload, 3.0)
                .addWaitAction(0.0, shootRamp())
                .addWaitAction(0.0, intakeAction())
                .addWaitAction(1, intakeActions.intakeA.feed()));

        tasks.add(new PathChainTask(paths.intake1P, 0.4)
                .addWaitAction(0.0, intakeActions.intakeA.stopFeed())
                .addWaitAction(0.0, intakeAction()));

        tasks.add(new PathChainTask(paths.shoot1P, 3.0)
                .addWaitAction(0.0, shootRamp())
                .addWaitAction(0.0, intakeAction())
                .addWaitAction(1, intakeActions.intakeA.feed()));

        tasks.add(new PathChainTask(paths.intake2P, 1) //clear balls
                .addWaitAction(0.0, intakeActions.intakeA.stopFeed())
                .addWaitAction(0.0, intakeAction()));

        tasks.add(new PathChainTask(paths.shoot2P, 3.5)
                .addWaitAction(0.0, shootRamp())
                .addWaitAction(0.0, intakeAction())
                .addWaitAction(1.5, intakeActions.intakeA.feed()));

        tasks.add(new PathChainTask(paths.intake3P, 0.4)
                .addWaitAction(0.0, intakeActions.intakeA.stopFeed())
                .addWaitAction(0.0, intakeAction()));

        tasks.add(new PathChainTask(paths.shoot3P, 3.0)
                .addWaitAction(0.0, shootRamp())
                .addWaitAction(0.0, intakeAction())
                .addWaitAction(1, intakeActions.intakeA.feed()));

        tasks.add(new PathChainTask(paths.intake4P, 0.4)
                .addWaitAction(0.0, intakeActions.intakeA.stopFeed())
                .addWaitAction(0.0, intakeAction()));

        tasks.add(new PathChainTask(paths.shoot4P, 3.0)
                .addWaitAction(0.0, shootRamp())
                .addWaitAction(0.0, intakeAction())
                .addWaitAction(1, intakeActions.intakeA.feed()));
    }

    private Action shootRamp() {
        return new ParallelAction(
                shooterActions.shooterA.shoot(),
                shooterActions.shooterA.hoodAngle()
        );
    }

    private Action intakeAction() {
        return new ParallelAction(
                intakeActions.intakeA.forwardIntake(),
                intakeActions.intakeA.index(),
                intakeActions.intakeA.holdNuts()
        );
    }

    private void runTasks() {
        if (currentTaskIndex >= tasks.size()) return;

        PathChainTask task = tasks.get(currentTaskIndex);

        switch (taskPhase) {

            case 0: // DRIVE
                if (!follower.isBusy()) {
                    follower.followPath(task.path, true);
                    pathTimer.resetTimer();
                    task.reset();
                }

                if (follower.getCurrentTValue() >= PATH_COMPLETION_T) {
                    pathTimer.resetTimer();
                    taskPhase = 1;
                }
                break;

            case 1: // WAIT
                double t = pathTimer.getElapsedTimeSeconds();

                for (WaitAction wa : task.waitActions) {
                    if (!wa.triggered && t >= wa.triggerTime) {
                        run(wa.action);
                        wa.triggered = true;
                    }
                }

                if (t >= task.waitTime) {
                    currentTaskIndex++;
                    taskPhase = 0;
                }
                break;
        }
    }

    @Override
    public void init() {

        pathTimer = new Timer();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, shooter, new ShooterHood(hardwareMap), new Limelight(hardwareMap));

        shooterActions = new ShooterActions(shooter, new Limelight(hardwareMap), new ShooterHood(hardwareMap));
        intakeActions = new IntakeActions(intake);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);

        paths = new Paths(follower);
        buildTaskList();
    }

    @Override
    public void start() {
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        runTasks();
    }
}