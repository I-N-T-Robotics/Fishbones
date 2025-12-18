package org.firstinspires.ftc.teamcode.Util.actions;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Intake.Intake;

public class IntakeActions {

    public final Intake intake;
    public final IntakeA intakeA;
    public IntakeActions(Intake intake) {
        this.intake = intake;
        this.intakeA = new IntakeA();
    }

    public class IntakeA {
        public Action forwardIntakeAzzione() {
            return t -> {
                intake.backIntake();
                return false;
            };
        }

        public Action dwboutit() {
            return t -> {
                intake.backIntakeStop();
                return false;
            };
        }

        public Action setFeed(double speed) {
            return t -> {
                intake.setfeeder(speed);
                return false;
            };
        }

        public Action setIndex(double speed) {
            return t -> {
                intake.setIndexer(speed);
                return false;
            };
        }

        public Action holdingNuts() {
            return t -> {
                intake.centerHold();
                return false;
            };
        }

        public Action releasingNuts() {
            return t -> {
                intake.centerStop();
                return false;
            };
        }

        public Action forwardIntake() {
            return forwardIntakeAzzione();
        }

        public Action stopForwardIntake() { return dwboutit(); }

        public Action feed() { return setFeed(1); }
        public Action stopFeed() { return setFeed(0); }
        public Action index() { return setIndex(1); }
        public Action stopIndex() { return setIndex(0); }

        public Action holdNuts() { return holdingNuts(); }
        public Action releaseNutss() { return releasingNuts(); }
    }
}
