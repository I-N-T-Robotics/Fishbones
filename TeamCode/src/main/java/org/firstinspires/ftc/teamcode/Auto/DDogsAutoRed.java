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

@Autonomous(name = "DDogsAutoRed", group = "Autos")
public class DDogsAutoRed extends ActionOpMode {

    private static class WaitAction {
        double triggerTime;
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
        double waitTime;
        List<WaitAction> waitActions = new ArrayList<>();

        PathChainTask(PathChain pathChain, double waitTime) {
            this.pathChain = pathChain;
            this.waitTime = waitTime;
        }

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

    private static final double PATH_COMPLETION_T = 0.985;

    private Follower follower;
    private Timer pathTimer;

    private Intake intake;
    private Shooter shooter;

    private IntakeActions intakeActions;
    private ShooterActions shooterActions;

    private final List<PathChainTask> tasks = new ArrayList<>();
    private int currentTaskIndex = 0;
    private int taskPhase = 0;

    private boolean pathStarted = false;

    private final Pose startPose = new Pose(56.129, 8.129, Math.toRadians(90)).mirror();

    private Paths paths;

    public static class Paths {

        public PathChain shootPreload;
        public PathChain intake1P1;
        public PathChain intake1P2;
        public PathChain intake;
        public PathChain shoot1;
        public PathChain move;

        public Paths(Follower follower) {

            shootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.129, 8.129), new Pose(58.258, 12.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                    .build();

            intake1P1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(58.258, 12.000).mirror(),
                                    new Pose(64.645, 21.871).mirror(),
                                    new Pose(48.194, 20.323).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                    .build();

            intake1P2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.194, 20.323).mirror(),
                                    new Pose(11.806, 16.452).mirror(),
                                    new Pose(8.516, 24.968).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            intake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.516, 24.968).mirror(),
                                    new Pose(10.452, 20.903).mirror(),
                                    new Pose(9.097, 8.516).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.097, 8.516).mirror(),
                                    new Pose(39.097, 19.742).mirror(),
                                    new Pose(58.258, 12.000).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                    .build();

            move = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.258, 12.000).mirror(), new Pose(33.871, 10.839).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(90))
                    .build();
        }
    }

    private void buildTaskList() {
        tasks.clear();

        tasks.add(
                new PathChainTask(paths.shootPreload, 5)
                        .addWaitAction(0.5, new ParallelAction(shooterActions.shooterA.shoot(), shooterActions.shooterA.hoodAngle()))
                        .addWaitAction(2, new ParallelAction(intakeActions.intakeA.index(), intakeActions.intakeA.feed(), intakeActions.intakeA.holdNuts(), intakeActions.intakeA.forwardIntake()))
        );

        tasks.add(
                new PathChainTask(paths.intake1P1, 0.5)
                        .addWaitAction(0.0, new ParallelAction(intakeActions.intakeA.index(), intakeActions.intakeA.holdNuts(), intakeActions.intakeA.forwardIntake()))
                        .addWaitAction(0.0, intakeActions.intakeA.stopFeed())
        );

        tasks.add(
                new PathChainTask(paths.intake1P2, 0.5)
                        .addWaitAction(0.0, new ParallelAction(intakeActions.intakeA.index(), intakeActions.intakeA.holdNuts(), intakeActions.intakeA.forwardIntake()))
        );

        tasks.add(
                new PathChainTask(paths.intake, 0.5)
                        .addWaitAction(0.0, new ParallelAction(intakeActions.intakeA.index(), intakeActions.intakeA.holdNuts(), intakeActions.intakeA.forwardIntake()))
        );

        tasks.add(
                new PathChainTask(paths.shoot1, 6.0)
                        .addWaitAction(1.0,
                                new ParallelAction(
                                        shooterActions.shooterA.shoot(),
                                        shooterActions.shooterA.hoodAngle()
                                )
                        )
                        .addWaitAction(3.5,
                                new ParallelAction(
                                        intakeActions.intakeA.index(),
                                        intakeActions.intakeA.feed(),
                                        intakeActions.intakeA.holdNuts(),
                                        intakeActions.intakeA.forwardIntake()
                                )
                        )
        );

        tasks.add(
                new PathChainTask(paths.move, 3)
        );
    }

    private void runTasks() {

        if (currentTaskIndex >= tasks.size()) return;

        PathChainTask task = tasks.get(currentTaskIndex);

        switch (taskPhase) {

            case 0: // DRIVE
                if (!pathStarted) {
                    follower.followPath(task.pathChain, true);
                    pathTimer.resetTimer();
                    task.resetWaitActions();
                    pathStarted = true;
                }

                if (follower.getCurrentTValue() >= PATH_COMPLETION_T) {
                    pathTimer.resetTimer();
                    taskPhase = 1;
                    pathStarted = false;
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
        follower.setStartingPose(startPose);

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