package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.Settings;
import org.firstinspires.ftc.teamcode.Util.LinearRegression;

import edu.wpi.first.math.geometry.Translation2d;

public class ShooterHood {

    private Servo hood;
    private LinearRegression hoodAngle;

    private double[] settingsDistance;
    private double[] settingsHoodAngles;

	private double	 _target = 0 ;

    public ShooterHood(HardwareMap hardwareMap) {
        hood = hardwareMap.get(Servo.class, "hood");

        settingsDistance = Settings.Shooter.distances;
        settingsHoodAngles = Settings.Shooter.hoodAngles;

        hoodRegression();
    }

    public void hoodRegression() {
        if (settingsDistance.length != settingsHoodAngles.length) {
            throw new IllegalArgumentException("Angle array != distance array");
        }

        Translation2d[] hoodPoints = new Translation2d[settingsHoodAngles.length];

        for (int i = 0; i < settingsHoodAngles.length; i++) {
            hoodPoints[i] = new Translation2d(settingsDistance[i], settingsHoodAngles[i]);
        }

        hoodAngle = new LinearRegression(hoodPoints);
    }

	public	void	targetDistance( double distance )
	{
		double pwd =
			Math.max( 0., Math.min( hoodAngle.calculatePoint( distance ), 1.)) ;
		targetPort( pwd ) ;
	}

	public	void	targetPort( double pwd )
	{
		hood.setPosition( pwd ) ;
		_target = pwd ;
	}

	public boolean	atAngle()
	{
		double delt = Math.abs( hood.getPosition() - _target ) ;
		return delt < 0.04 ;
	}
	public	double	progress()
	{
		double delt = Math.abs( hood.getPosition() - _target ) ;
		return 1 - delt ;
	}
}

