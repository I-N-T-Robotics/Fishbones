package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.Settings;

import org.firstinspires.ftc.teamcode.Util.LinearInterpolator;

public class Shooter {

    private DcMotorEx shooterRight, shooterLeft;
    private ShooterHood shooterHood;

    private double[][] speedData = {{.2, 35}, {.4, 65}};
    private LinearInterpolator SPEED_INTERPOLATOR;

    private double ticksPerRev;
    private double[] settingsDistance;
    private double[] settingsShooterSpeeds;

    private double  rightTarget = 0 ;
    private static final double IDLE_POWER = 0.1;
    private static final double TICK_RATIO = 537.6 ;

    public Shooter(HardwareMap hardwareMap) {
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");

        shooterHood = new ShooterHood(hardwareMap);
        //shooterHood.setHood(0.9);

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterRight.setVelocityPIDFCoefficients(Settings.Shooter.kP, Settings.Shooter.kI, Settings.Shooter.kD, Settings.Shooter.kF);
        shooterLeft.setVelocityPIDFCoefficients(Settings.Shooter.kP, Settings.Shooter.kI, Settings.Shooter.kD, Settings.Shooter.kF);
        ticksPerRev = shooterRight.getMotorType().getTicksPerRev();

        settingsDistance = Settings.Shooter.distances;
        settingsShooterSpeeds = Settings.Shooter.shooterSpeeds;

        SPEED_INTERPOLATOR = new LinearInterpolator(speedData);

    }

    public void idle() {
        shooterRight.setPower(IDLE_POWER);
        shooterLeft.setPower(IDLE_POWER);
    }

    public double getShooterRPM(double distance) {
        return SPEED_INTERPOLATOR.getInterpolatedValue(distance);
    }

    public double getRawSpeed() {
        return shooterLeft.getVelocity();
    }

    public void targetDistance( double distance ) {
        double rate = getShooterRPM ( distance ) * TICK_RATIO / 60.;
        shootVelocity(rate);
    }

    public void targetSpeed( double speed )
    {
        double rate = speed * TICK_RATIO / 60. ;
        shootVelocity(rate);
    }

    public void shootVelocity( double vel )
    {
        shooterRight.setVelocity(vel);
        shooterLeft.setVelocity(vel);
        rightTarget = vel;
    }

    public void shootPower(double power) {
        shooterRight.setPower(power);
        shooterLeft.setPower(power);
    }

    public void stop() {
        shooterRight.setPower(0);
        shooterLeft.setPower(0);
    }

    public double targetProgress() {
        return ( rightTarget > 0. ) ? shooterRight.getVelocity() / rightTarget : 1. ;
    }
}