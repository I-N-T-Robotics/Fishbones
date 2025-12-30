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
                intake.enable();
                return false;
            };
        }

        public Action dwboutit() {
            return t -> {
                intake.off();
                return false;
            };
        }

        public Action setFeed() {
            return t -> {
                intake.feed();
                return false;
            };
        }

        public Action forwardIntake() {
            return forwardIntakeAzzione();
        }

        public Action stop() {
            return dwboutit();
        }

        public Action feed() {
            return setFeed();
        }

    }
}