package org.firstinspires.ftc.teamcode.Util.actions;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Shooter.Shooter;

public class ShooterActions {

    public final Shooter shooter;
    public final ShooterA shooterA;

    public ShooterActions(Shooter shooter) {
        this.shooter = shooter;
        this.shooterA = new ShooterA();
    }

    public class ShooterA {
        public Action setShooter(double speed) {
            return t -> {
                shooter.setShooter(speed);
                return false;
            };
        }

        public Action shoot() { return setShooter(.3); }
        public Action stopShoot() { return setShooter(0); }
    }
}
