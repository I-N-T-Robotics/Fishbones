package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Settings;
import org.firstinspires.ftc.teamcode.Util.AnalogEncoder;
import org.firstinspires.ftc.teamcode.Util.EncoderConversion;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModules extends SwerveModuleBase {
    private final Rotation2d angleOffset;

    private final PIDController driveControllerPID;
    private final SimpleMotorFeedforward driveControllerFF;
    private final PIDController turnControllerPID;

    private final DcMotorEx driveMotor;
    private final EncoderConversion realDriveEncoder; //digital, converted to meters

    private final CRServo turnMotor;
    private final AnalogInput turnEncoder; //better info'
    private final boolean reversed;

    public double globalff;
    public double globalPID;
    public double globalTarget;
    public double globalAngle;

//    private final void debug() {
//        telemetry.addData("FirstSwerveModuleCall", "FirstSwerveModuleCall");
//        telemetry.update();
//    }

//    public static SwerveModuleBase[] getModules() {
//        return new SwerveModuleBase[] {
//                new SwerveModules("Front Right", Settings.Swerve.FrontRight.MODULE_OFFSET, Rotation2d.fromDegrees(-153.632812 + 180), Settings.Swerve.FrontRight.Turn, Settings.Swerve.FrontRight.DRIVE, Settings.Swerve.FrontRight.TURN_ENCODER, Settings.Swerve.FrontRight.DRIVE_ENCODER)
//                //new SwerveModules("Front Left",  Settings.Swerve.FrontLeft.MODULE_OFFSET,  Rotation2d.fromDegrees(147.919922 + 180),  Settings.Swerve.FrontLeft.Turn, Settings.Swerve.FrontLeft.DRIVE, Settings.Swerve.FrontLeft.TURN_ENCODER, Settings.Swerve.FrontLeft.DRIVE_ENCODER),
//                //new SwerveModules("Back Left",   Settings.Swerve.BackLeft.MODULE_OFFSET,   Rotation2d.fromDegrees(73.125 + 180),   Settings.Swerve.BackLeft.Turn, Settings.Swerve.BackLeft.DRIVE, Settings.Swerve.BackLeft.TURN_ENCODER, Settings.Swerve.BackLeft.DRIVE_ENCODER),
//                //new SwerveModules("Back Right",  Settings.Swerve.BackRight.MODULE_OFFSET,  Rotation2d.fromDegrees(-2.02184 + 180),  Settings.Swerve.BackRight.Turn, Settings.Swerve.BackRight.DRIVE, Settings.Swerve.BackRight.TURN_ENCODER, Settings.Swerve.BackRight.DRIVE_ENCODER)
//        };
//    }

    public SwerveModules(HardwareMap hardwareMap, Telemetry telemetry, String id, Translation2d translationOffset, Rotation2d angleOffset, String turnName, String driveName, String turnEncoderName, PIDController turnPID, boolean reversed) {
        super(id, translationOffset);

        this.angleOffset = angleOffset;

        driveMotor = hardwareMap.get(DcMotorEx.class, driveName);
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turnMotor = hardwareMap.get(CRServo.class, turnName);
        turnEncoder = hardwareMap.get(AnalogInput.class, turnEncoderName);

        realDriveEncoder = new EncoderConversion(driveMotor, 2.5, 8.6, 8192);

        driveControllerPID = new PIDController(0.7, 0, 0);
        driveControllerFF = new SimpleMotorFeedforward(0, 0, 0);

        turnControllerPID = turnPID;
        turnControllerPID.enableContinuousInput(-180, 180);

        this.reversed = reversed;

        if (reversed) {
            driveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        //configure?
    }

    public double getVolts() {
        return turnEncoder.getVoltage();
    }

    public int getRawDriveEncoder() {
        return driveMotor.getCurrentPosition();
    }

    public double getRawTurnTargetAngle() {
        return rawTargetAngleState;
    }

    public double getRawSpeedTarget() {
        return rawTargetSpeedState;
    }

    @Override
    public double getVelocity() {
        return realDriveEncoder.getVelocityMetersPerSecond();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(new AnalogEncoder(turnEncoder, 3.3, 1).getDegrees() - angleOffset.getDegrees());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(realDriveEncoder.getPositionMeters(), getAngle());
    }

    private Thread driveThread;
    private Thread turnThread;
    private volatile boolean running = false;

    private static final double loopTime = 0.02;

    public void startThreads() {
        if (running) return;
        running = true;

        driveThread = new Thread(() -> {
            ElapsedTime timer = new ElapsedTime();
            while (running) {
                timer.reset();

                double targetSpeed = getTargetState().speedMetersPerSecond;
                double ffOutput = driveControllerFF.calculate(targetSpeed);
                double PIDOutput = driveControllerPID.calculate(getVelocity(), getTargetState().speedMetersPerSecond);
                double finalOutput = Math.max(-1.0, Math.min(1.0, ffOutput + PIDOutput));

                globalTarget = getTargetState().speedMetersPerSecond;

                if (Math.abs(driveControllerPID.getSetpoint()) < Settings.Swerve.MODULE_VELOCITY_DEADBAND) {
                    driveMotor.setPower(0);
//                } else if (reversed) {
//                    driveMotor.setPower(finalOutput);
                } else {
                    driveMotor.setPower(-finalOutput);
                }

                double elapsed = timer.seconds();
                long sleepMs = (long) Math.max(0, (loopTime - elapsed) * 1000);
                if (elapsed > loopTime && sleepMs > 0) {
                    try {
                        Thread.sleep(sleepMs);
                    } catch (InterruptedException e) {
                        running = false;
                        Thread.currentThread().interrupt();
                    }
                }
            }
        });

        turnThread = new Thread(() -> {
            ElapsedTime timer = new ElapsedTime();
            while (running) {
                timer.reset();

                double turnOutput = turnControllerPID.calculate(getAngle().getDegrees(), getTargetStateAngle());

                globalAngle = getTargetState().angle.getDegrees() ;

                if (Math.abs(turnControllerPID.getSetpoint()) < Settings.Swerve.MODULE_TURN_DEADBAND) {
                    turnMotor.setPower(0);
                } else {
                    turnMotor.setPower(Math.max(-1.0, Math.min(1.0, turnOutput)));
                }

                double elapsed = timer.seconds();
                long sleepMs = (long) Math.max(0, (loopTime - elapsed) * 1000);
                if (elapsed > loopTime && sleepMs > 0) {
                    try {
                        Thread.sleep(sleepMs);
                    } catch (InterruptedException e) {
                        running = false;
                        Thread.currentThread().interrupt();
                    }
                }
            }
        });

        driveThread.start();
        turnThread.start();
    }

    public void stopThreads() {
        running = false;
        if (driveThread != null) driveThread.interrupt();
        if (turnThread != null) turnThread.interrupt();
    }

//    public void periodic() {
//        double ffOutput = driveControllerFF.calculate(getTargetState().speedMetersPerSecond);
//        double PIDOutput = driveControllerPID.calculate(getVelocity(), getTargetState().speedMetersPerSecond);
//        double finalOutput = ffOutput + PIDOutput;
//
//        globalff = ffOutput;
//        globalPID = PIDOutput;
//        globalTarget = getTargetState().speedMetersPerSecond;
//        globalAngle = getTargetState().angle.getDegrees();
//
//        double turnOutput = turnControllerPID.calculate(getAngle().getDegrees(), getTargetStateAngle());
//
//        if(Math.abs(driveControllerPID.getSetpoint()) < Settings.Swerve.MODULE_VELOCITY_DEADBAND) {
//            driveMotor.setPower(0);
//            turnMotor.setPower(0);
//        } else {
//            driveMotor.setPower(-finalOutput);
//            turnMotor.setPower(turnOutput);
//        }
//    }
}