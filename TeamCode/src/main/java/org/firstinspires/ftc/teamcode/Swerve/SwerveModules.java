package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private final AnalogInput turnEncoder; //better info

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

    public SwerveModules(HardwareMap hardwareMap, Telemetry telemetry, String id, Translation2d translationOffset, Rotation2d angleOffset, String turnName, String driveName, String turnEncoderName) {
        super(id, translationOffset);

        this.angleOffset = angleOffset;

        driveMotor = hardwareMap.get(DcMotorEx.class, driveName);

        turnMotor = hardwareMap.get(CRServo.class, turnName);
        turnEncoder = hardwareMap.get(AnalogInput.class, turnEncoderName);

        realDriveEncoder = new EncoderConversion(driveMotor, 2.5, 4, 8192);

        driveControllerPID = new PIDController(/*0.7*/0, 0, 0);
        driveControllerFF = new SimpleMotorFeedforward(0, 0, 0);

        turnControllerPID = new PIDController(0.003, 0, 0);
        turnControllerPID.enableContinuousInput(0, 360); //choose between this and .optimize

        //configure?
    }

    @Override
    public double getVelocity() {
        return realDriveEncoder.getVelocityMetersPerSecond();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(new AnalogEncoder(turnEncoder, 3.3, 8192).getDegrees() - angleOffset.getDegrees());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(realDriveEncoder.getPositionMeters(), getAngle());
    }

    public void periodic() {
//        super.periodic();

        double ffOutput = driveControllerFF.calculate(getTargetState().speedMetersPerSecond);
        double PIDOutput = driveControllerPID.calculate(getVelocity(), getTargetState().speedMetersPerSecond);
        double finalOutput = ffOutput + PIDOutput;

        double turnOutput = turnControllerPID.calculate(getAngle().getDegrees(), getTargetStateAngle());

        if(Math.abs(driveControllerPID.getSetpoint()) < Settings.Swerve.MODULE_VELOCITY_DEADBAND) {
            driveMotor.setPower(0);
            turnMotor.setPower(0);
        } else {
            driveMotor.setPower(-finalOutput);
            turnMotor.setPower(turnOutput);
        }

//        telemetry.addData("SwerveModules" + getID() + "DriveVoltage", finalOutput);
//        telemetry.addData("SwerveModules" + getID() + "TurnVoltage", turnOutput);
//        telemetry.addData("SwerveModule" + getID() + "Angle", getAngle().getDegrees());
//        telemetry.addData("SwerveModule" + getID() + "AngleError", getTargetStateAngle() - getAngle().getDegrees());
//        telemetry.addData("SwerveModule" + getID() + "Velocity", getVelocity());
    }
}