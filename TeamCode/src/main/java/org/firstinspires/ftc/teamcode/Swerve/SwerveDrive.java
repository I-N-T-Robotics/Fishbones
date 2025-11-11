//add pathplanner

package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Settings;
import org.firstinspires.ftc.teamcode.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.Util.navx.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class SwerveDrive {
    private static SwerveDrive instance;
    private final Gamepad gamepad1;

    public ChassisSpeeds globalSpeed;
    public double globalTwistdx;
//    private final void debug() {
//        telemetry.addData("FirstSwerveCall", "FirstSwerveCall");
//        telemetry.update();
//    }

//    static {
//        instance = new SwerveDrive(
//                new SwerveModules("Front Right", Settings.Swerve.FrontRight.MODULE_OFFSET, Rotation2d.fromDegrees(-153.632812 + 180), Settings.Swerve.FrontRight.Turn, Settings.Swerve.FrontRight.DRIVE, Settings.Swerve.FrontRight.TURN_ENCODER, Settings.Swerve.FrontRight.DRIVE_ENCODER)
//                //new SwerveModules("Front Left", Settings.Swerve.FrontLeft.MODULE_OFFSET, Rotation2d.fromDegrees(147.919922 + 180), Settings.Swerve.FrontLeft.Turn, Settings.Swerve.FrontLeft.DRIVE, Settings.Swerve.FrontLeft.TURN_ENCODER, Settings.Swerve.FrontLeft.DRIVE_ENCODER),
//                //new SwerveModules("Back Left", Settings.Swerve.BackLeft.MODULE_OFFSET, Rotation2d.fromDegrees(73.125 + 180), Settings.Swerve.BackLeft.Turn, Settings.Swerve.BackLeft.DRIVE, Settings.Swerve.BackLeft.TURN_ENCODER, Settings.Swerve.BackLeft.DRIVE_ENCODER),
//                //new SwerveModules("Back Right", Settings.Swerve.BackRight.MODULE_OFFSET, Rotation2d.fromDegrees(-2.02184 + 180), Settings.Swerve.BackRight.Turn, Settings.Swerve.BackRight.DRIVE, Settings.Swerve.BackRight.TURN_ENCODER, Settings.Swerve.BackRight.DRIVE_ENCODER)
//        );
//    }

    public static SwerveDrive createInstance(Gamepad gamepad1, HardwareMap hardwareMap, Telemetry telemetry) {
        if (instance == null) {
            instance = new SwerveDrive(gamepad1,hardwareMap, telemetry);
        }
        return instance;
    }

    public static SwerveDrive getInstance() {

//        telemetry.addData("SwerveInstanceCall", "SwerveInstanceCall");
//        telemetry.update();

        return instance;
    }

    //pathplanner later

    private final SwerveModules[] modules;
    private final SwerveDriveKinematics kinematics;
    private final AHRS gyro;
//    private final FieldObject2d[] modules2D;
//    private final TelemetryPacket statesPub;
    //private final FtcDashboard dashboard;

    protected SwerveDrive(Gamepad gamepad1, HardwareMap hardwareMap, Telemetry telemetry) {
        this.gamepad1 = gamepad1;

        this.modules = new SwerveModules[] {
                new SwerveModules(hardwareMap, telemetry, "Front Right",
                        Settings.Swerve.FrontRight.MODULE_OFFSET,
                        Settings.Swerve.FrontRight.ABSOLUTE_OFFSET,
                        Settings.Swerve.FrontRight.Turn,
                        Settings.Swerve.FrontRight.DRIVE,
                        Settings.Swerve.FrontRight.TURN_ENCODER,
                        new PIDController(0.007, 0, 0.0002),
                        true),
                new SwerveModules(hardwareMap, telemetry, "Front Left",
                        Settings.Swerve.FrontLeft.MODULE_OFFSET,
                        Settings.Swerve.FrontLeft.ABSOLUTE_OFFSET,
                        Settings.Swerve.FrontLeft.Turn,
                        Settings.Swerve.FrontLeft.DRIVE,
                        Settings.Swerve.FrontLeft.TURN_ENCODER,
                        new PIDController(0.007, 0, 0.0002),
                        false),
                new SwerveModules(hardwareMap, telemetry, "Back Right",
                        Settings.Swerve.BackRight.MODULE_OFFSET,
                        Settings.Swerve.BackRight.ABSOLUTE_OFFSET,
                        Settings.Swerve.BackRight.Turn,
                        Settings.Swerve.BackRight.DRIVE,
                        Settings.Swerve.BackRight.TURN_ENCODER,
                        new PIDController(0.007, 0, 0.0002),
                        true),
                new SwerveModules(hardwareMap, telemetry, "Back Left",
                        Settings.Swerve.BackLeft.MODULE_OFFSET,
                        Settings.Swerve.BackLeft.ABSOLUTE_OFFSET,
                        Settings.Swerve.BackLeft.Turn,
                        Settings.Swerve.BackLeft.DRIVE,
                        Settings.Swerve.BackLeft.TURN_ENCODER,
                        new PIDController(0.007, 0, 0.0002),
                        false)//real testing 0.03, 0, 0.0000021))//0.05, 0.0045, 0.01))
        };

        kinematics = new SwerveDriveKinematics(getModuleOffsets());
        gyro = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "gyro"),
                AHRS.DeviceDataType.kProcessedData); //check all info
//        modules2D = new FieldObject2d[modules.length];

//        statesPub = new TelemetryPacket();
//        statesPub.put("speed", SwerveModuleState.struct);
//        this.dashboard = FtcDashboard.getInstance();

//        telemetry.addData("SwerveStates", SwerveModuleState.struct);
    }

//    public void initFieldObject(Field2d field) {
//        for (int i = 0; i < modules.length; i++) {
//            modules2D[i] = field.getObject(modules[i].getID() + "-2d");
//        }
//    }

    //getters
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }
    public SwerveModules[] getSwerveModules() {
        return this.modules;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] offsets = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            offsets[i] = modules[i].getModulePosition();
        }
        return offsets;
    }

    public Translation2d[] getModuleOffsets() {
        Translation2d[] offsets = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            offsets[i] = modules[i].getModuleOffset();
        }
        return offsets;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    //logs
    public double getTargetAngle() {
        double tAngle = 0;
        for (int i = 0; i < modules.length; i++) {
            tAngle = modules[i].getTargetStateAngle();
        }
        return tAngle;
    }

    //setters
    public void setModuleStates(SwerveModuleState[] states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("Provided incorrect number of states for swerve drive modules");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, 100 /*max module speed*/); //fix

        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(states[i]);
        }
    }

    public void setFieldRelativeSpeeds(ChassisSpeeds chassisSpeeds) {
        setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds,
                Odometry.getInstance().getPose().getRotation()));
    }

    public void setChassisSpeeds(ChassisSpeeds robotSpeeds) {
        setModuleStates(kinematics.toSwerveModuleStates(robotSpeeds));
    }

    public void updateModules() {
        for (SwerveModules module : modules) {
            module.periodic();
        }
    }

    public void setXMode() {
        setModuleStates(
                new SwerveModuleState[] {
                        new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(135))
                }
        );
    }

    //drive functions
    public void drive(Translation2d velocity, double rotation) {

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.getY(), -velocity.getX(),
                -rotation,
                Odometry.getInstance().getPose().getRotation());

        globalSpeed = speeds;
        Pose2d robotVel = new Pose2d(
                Settings.DT * speeds.vxMetersPerSecond,
                Settings.DT * speeds.vyMetersPerSecond,
                Rotation2d.fromRadians(Settings.DT * speeds.omegaRadiansPerSecond));
        Twist2d twistVel = new Pose2d().log(robotVel);

        setChassisSpeeds(new ChassisSpeeds(
                twistVel.dx / Settings.DT,
                twistVel.dy / Settings.DT,
                twistVel.dtheta / Settings.DT));
        globalTwistdx = twistVel.dx / Settings.DT;
    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    }

    //Gyro
    public Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public double getForwardAccelerationGs() {
        return gyro.getWorldLinearAccelY(); //Y is forwards, change if otherwise
    }

    public AngularVelocity getAngularVelocity() {
        return Units.RadiansPerSecond.of(this.getChassisSpeeds().omegaRadiansPerSecond);
    }

    public void periodic() {
//        telemetry.update();

        Odometry odometry = Odometry.getInstance();
        Pose2d pose = odometry.getPose();
        Rotation2d angle = pose.getRotation();

//        for (int i = 0; i < modules.length; i++) {
//            modules2D[i].setPose(new Pose2d(
//                    pose.getTranslation().plus(modules[i].getModuleOffset().rotateBy(angle)),
//                    modules[i].getAngle().plus(angle)
//            ));
//        }

    }
}