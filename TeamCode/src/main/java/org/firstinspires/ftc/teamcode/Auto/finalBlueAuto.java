package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
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

@Autonomous(name = "finalBlueAuto", group = "Autos")
public class finalBlueAuto extends ActionOpMode {

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

    private final Pose startPose = new Pose(25.337, 129.642, Math.toRadians(144));

    private Paths paths;

    public class Paths {

        public PathChain preload;
        public PathChain intake1P, intake12P, shoot1P;
        public PathChain intake2P, shoot2P;
        public PathChain intake3P, shoot3P;
        public PathChain intake4P, shoot4P;

        Paths(Follower follower) {

            preload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(25.337, 129.642), new Pose(60.246, 83.472))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(132))
                    .build();

            intake1P = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.246, 83.472), new Pose(53.208, 83.331))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(0))
                    .addTemporalCallback(0, () -> intakeActions.intakeA.forwardIntake())
                    .build();

            intake12P = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.208, 83.331), new Pose(16.751, 83.472))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            shoot1P = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.751, 83.472), new Pose(60.246, 83.754))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(132))
                    .build();


            intake2P = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.246, 83.754),
                                    new Pose(83.894, 53.490),
                                    new Pose(13.232, 62.639)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(-10))
                    .addTemporalCallback(0, () -> intakeActions.intakeA.forwardIntake())
                    .build();

            shoot2P = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.232, 62.639), new Pose(59.824, 83.894))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(132))
                    .build();
        }
    }

    private void buildTaskList() {
        tasks.clear();

        tasks.add(new PathChainTask(paths.preload, 3.0)
                .addWaitAction(0.5, intakeActions.intakeA.feed()));

        tasks.add(new PathChainTask(paths.intake1P, 0.4)
                .addWaitAction(0.0, intakeActions.intakeA.stop()));

        tasks.add(new PathChainTask(paths.intake12P, 0));

        tasks.add(new PathChainTask(paths.shoot1P, 3.0)
                .addWaitAction(0, intakeActions.intakeA.feed()));

//        tasks.add(new PathChainTask(paths.intake2P, 1)
//                .addWaitAction(0.0, intakeActions.intakeA.stop()));
//
//        tasks.add(new PathChainTask(paths.shoot2P, 3.5)
//                .addWaitAction(0, intakeActions.intakeA.feed()));
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
        intake = new Intake(hardwareMap);

        shooterActions = new ShooterActions(shooter, new Limelight(hardwareMap), new ShooterHood(hardwareMap));
        intakeActions = new IntakeActions(intake);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);


        paths = new Paths(follower);
        buildTaskList();
    }

    @Override
    public void start() {
        shooterActions.shooterA.shoot();
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