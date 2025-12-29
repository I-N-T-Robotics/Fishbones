package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.Settings;
import org.firstinspires.ftc.teamcode.Util.LinearRegression;

import edu.wpi.first.math.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Shooter.ShooterHood ;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

public class Shooter {

    private DcMotorEx shooterRight, shooterLeft;
    private LinearRegression rpm = null;

    private double ticksPerRev;

    private double[] settingsDistance;
    private double[] settingsShooterSpeeds;

	private double  rightTarget = 0 ;

    private static final double IDLE_POWER = 0.1;
	private static final double	TICK_RATIO = 537.6 ;

	public ShooterHood	hood ;
	public Limelight	lime ;

    public Shooter(HardwareMap hardwareMap) {
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
		hood         = new ShooterHood( hardwareMap ) ;
		lime		 = new Limelight( hardwareMap ) ;

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterRight.setVelocityPIDFCoefficients(Settings.Shooter.kP, Settings.Shooter.kI, Settings.Shooter.kD, Settings.Shooter.kF);
        shooterLeft.setVelocityPIDFCoefficients(Settings.Shooter.kP, Settings.Shooter.kI, Settings.Shooter.kD, Settings.Shooter.kF);
        ticksPerRev = shooterRight.getMotorType().getTicksPerRev();

        settingsDistance = Settings.Shooter.distances;
        settingsShooterSpeeds = Settings.Shooter.shooterSpeeds;

        rpmRegression();
    }

    public void idle() {
        shooterRight.setPower(IDLE_POWER);
        shooterLeft.setPower(IDLE_POWER);
    }

    public void rpmRegression() {
        if (settingsDistance.length != settingsShooterSpeeds.length) {
            throw new IllegalArgumentException("Shooter array != distance array");
        }

        Translation2d[] points = new Translation2d[settingsDistance.length];
        for (int i = 0; i < settingsDistance.length; i++) {
            points[i] = new Translation2d(settingsDistance[i], settingsShooterSpeeds[i]);
        }

        rpm = new LinearRegression(points);
    }

	public void	target()
	{
		double dist = limelight.getLastDist() ;

		targetDistance( dist ) ;
		hood.targetDistance( dist ) ;
	}

	public void	targetDistance( double distance ) {
		double rate = getShooterRPM ( distance ) * TICK_RATIO / 60. ;

		setRate( rate ) ;
	}

	public void	targetSpeed( double speed )
	{
		double rate = speed * TICK_RATIO / 60. ;
		setRate( rate ) ;
	}

	public void	setRate( double rate )
	{
        shooterRight.setVelocity(rate);
        shooterLeft.setVelocity(rate);
		rightTarget = rate ;
	}

    public void runShooter() {
        shooterRight.setPower(1);
        shooterLeft.setPower(1);
		rightTarget = 0 ;
    }

	public boolean	atTarget() {
		return hood.atAngle() && shooterRight.getVelocity() >= rightTarget ;
	}
	public double   targetProgress() {
		return ( rightTarget > 0. ) ? shooterRight.getVelocity() / rightTarget : 1. ;
	}

    public void stop() {
        shooterRight.setPower(0);
        shooterLeft.setPower(0);
    }
}
