package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.LinearInterpolator;

public class ShooterHood {

    private Servo hood;
//Min .15 max 1
	private double hoodData[][] = {{0.2, 0.2}, {1, .5}};
    private LinearInterpolator HOOD_INTERPOLATOR;

	private double	 _target = 0 ;

    public ShooterHood(HardwareMap hardwareMap) {
        hood = hardwareMap.get(Servo.class, "hood");

        HOOD_INTERPOLATOR = new LinearInterpolator(hoodData);
    }

	public	void	targetDistance( double distance )
	{
		double pwd =
				HOOD_INTERPOLATOR.getInterpolatedValue(distance);
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

	public void setHood(double angle){
		hood.setPosition(angle);
	}
	public double getHoodPos(){
		return hood.getPosition();
	}
}

