package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Util.Normalize;

public class Intake {
    private final CRServo s_intake ;

    private final DcMotorEx m_intake, m_outake ;

    private ElapsedTime timer= new ElapsedTime();

    private Shooter shooter;

    public Intake(HardwareMap hardwareMap, Shooter shooter ) {
        this.shooter= shooter;

        s_intake= hardwareMap.get(CRServo.class, "sin");

        m_intake= hardwareMap.get(DcMotorEx.class, "min");
        m_outake= hardwareMap.get(DcMotorEx.class, "mou");

        s_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        m_intake.setDirection(DcMotorSimple.Direction.FORWARD);
        m_outake.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void enable()
    {
        s_intake.setPower( 1) ;
        m_intake.setPower( 1) ;
        m_outake.setPower( -.1) ;
    }

    public void shoot(double distance )
    {
            shooter.setRate(shooter.getShooterRPM(distance)); ;
        }

    public void    off()
    {
        s_intake.setPower( 0) ;
        m_intake.setPower( 0) ;
        m_outake.setPower( 0) ;
    }

    public void feed() {
        m_outake.setPower(1);
        s_intake.setPower( 1) ;
        m_intake.setPower( 1) ;
    }

}