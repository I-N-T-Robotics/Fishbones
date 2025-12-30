package org.firstinspires.ftc.teamcode.Util.actions;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Shooter.ShooterHood;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

public class ShooterActions {

    public final Shooter shooter;
    public final Limelight limelight;
    public final ShooterHood shooterHood;
    public final ShooterA shooterA;

    public ShooterActions(Shooter shooter, Limelight limelight, ShooterHood shooterHood) {
        this.shooter = shooter;
        this.limelight = limelight;
        this.shooterHood = shooterHood;
        this.shooterA = new ShooterA();
    }

    public class ShooterA {
        public Action setShooterSpeed() {
            return t -> {
                shooter.setSpeed(.25);
                return false;
            };
        }

        public Action setHoodAngle(double distance) {
            return t -> {
                //shooterHood.setHoodPosition(distance);
                return false;

            };
        }

        public Action shoot() {
            return setShooterSpeed();
        }

        public Action hoodAngle() {
            return setHoodAngle(limelight.getLastDist());
        }
    }
}