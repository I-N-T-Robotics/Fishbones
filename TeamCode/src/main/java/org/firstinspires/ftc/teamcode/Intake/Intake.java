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

    private Normalize normalizeFront;
    private Normalize normalizeBack;

    public enum States {
        OFF,
        INTAKE,
        PREP,
        SHOOT,
    }

    public States state;

    public Intake(HardwareMap hardwareMap, Shooter shooter ) {
        this.shooter= shooter;

        s_intake= hardwareMap.get(CRServo.class, "sin");

        m_intake= hardwareMap.get(DcMotorEx.class, "min");
        m_outake= hardwareMap.get(DcMotorEx.class, "mou");

        s_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        m_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        m_outake.setDirection(DcMotorSimple.Direction.REVERSE);

        state = States.OFF;
    }

    public void enable()
    {
        s_intake.setPower( 1) ;
        m_intake.setPower( 1) ;
        m_outake.setPower( 0) ;
        state = States.INTAKE ;
    }

    public void shoot( )
    {
        if ( States.INTAKE == state ) {
            shooter.target() ;
            state = States.PREP ;
        }

        if ( States.SHOOT == state ) {
            enable() ;
        }
    }

    public void    off()
    {
        s_intake.setPower( 0) ;
        m_intake.setPower( 0) ;
        m_outake.setPower( 0) ;
        state = States.OFF ;
    }

    public void update() {
        if ( States.PREP == state ) {
            if ( ! shooter.atTarget() ) { return ; }

            m_outake.setPower( 1) ;
            state = States.SHOOT ;
        }
    }

    public boolean stable()
    {
        return States.SHOOT == state ;
    }
}