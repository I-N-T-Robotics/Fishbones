package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.ServoHubConfiguration;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Settings;
import org.firstinspires.ftc.teamcode.Util.AnalogEncoder;
import org.firstinspires.ftc.teamcode.Util.EncoderConversion;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

    public double globalFinal;
    public double globalTurn;
    public double globalTarget;
    public double globalAngle;
    public double globalVel;

    private double lastDirection = 0;
    static double filteredVel = 0;

    public SwerveModules(HardwareMap hardwareMap, Telemetry telemetry, String id, Translation2d translationOffset, Rotation2d angleOffset, String turnName, String driveName, String turnEncoderName, PIDController turnPID, boolean reversed) {
        super(id, translationOffset);

        this.angleOffset = angleOffset;

        driveMotor = hardwareMap.get(DcMotorEx.class, driveName);

        turnMotor = hardwareMap.get(CRServo.class, turnName);
        turnEncoder = hardwareMap.get(AnalogInput.class, turnEncoderName);

        realDriveEncoder = new EncoderConversion(driveMotor, 2.5, 8.6, 8192);

        driveControllerPID = new PIDController(0.7, 0, 0);
        driveControllerFF = new SimpleMotorFeedforward(0.1, 0.5, 0);

        turnControllerPID = turnPID;
        turnControllerPID.enableContinuousInput(-180, 180);

        this.reversed = reversed;

//        if (!reversed) {
//            driveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        } else {
//            driveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
        //configure?
    }

    public void setDriveConfig() {
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void periodic() {
        double targetSpeed = getTargetState().speedMetersPerSecond;
        double measuredVel = getVelocity();
        double absTarget = Math.abs(targetSpeed);
        double direction = Math.signum(targetSpeed);

        final double STOP_DEADBAND = 0.10;
        final double ZERO_VEL_BAND = 0.2;
        final double ZERO_CROSS_SMOOTH = 0.7;

        double finalOutput = 0.0;

        if (absTarget < STOP_DEADBAND && Math.abs(measuredVel) < ZERO_VEL_BAND) {
            driveControllerPID.reset();
            finalOutput = 0;
        } else {
            double ffOutput = driveControllerFF.calculate(absTarget);
            double pidOutput = driveControllerPID.calculate(Math.abs(measuredVel), absTarget);
            finalOutput = direction * (ffOutput + pidOutput);

            if (direction != 0 && direction != lastDirection) {
                if (absTarget < ZERO_CROSS_SMOOTH) {
                    finalOutput *= absTarget / ZERO_CROSS_SMOOTH;
                } else if (Math.abs(measuredVel) < Settings.Swerve.MODULE_VELOCITY_DEADBAND) {
                    driveControllerPID.reset();
                    lastDirection = direction;
                } else {
                    direction = lastDirection;
                    finalOutput = direction * Math.abs(finalOutput);
                }
            }

            if (!reversed) {
                driveMotor.setPower(finalOutput);
            } else {
                driveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                driveMotor.setPower(finalOutput);
            }
        }

        double turnOutput = turnControllerPID.calculate(getAngle().getDegrees(), getTargetStateAngle());
        globalTurn = turnOutput;
        globalAngle = getTargetState().angle.getDegrees();

        if (Math.abs(turnControllerPID.getSetpoint()) < Settings.Swerve.MODULE_TURN_DEADBAND) {
            turnMotor.setPower(0);
        } else {
            turnMotor.setPower(turnOutput);
        }

        globalFinal = driveMotor.getPower();
        globalTarget = targetSpeed;
        globalVel = measuredVel;
    }
}