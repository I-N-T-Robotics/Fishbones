package org.firstinspires.ftc.teamcode.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModuleBase{
    private final String id;
    private final Translation2d offset;
    private SwerveModuleState targetState;

    public double rawTargetAngleState;
    public double rawTargetSpeedState;

    public SwerveModuleBase(String id, Translation2d offset) {
        this.id = id;
        this.offset = offset;

        targetState = new SwerveModuleState();
    }

    public final String getID() {
        return this.id;
    }

    public final Translation2d getModuleOffset() {
        return this.offset;
    }

    public abstract double getVelocity();
    public abstract Rotation2d getAngle();
    public abstract SwerveModulePosition getModulePosition();
    public final SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public final void setTargetState(SwerveModuleState state) {
        targetState = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
        rawTargetAngleState = targetState.angle.getDegrees();
        rawTargetSpeedState = targetState.speedMetersPerSecond;
        targetState.optimize(getAngle());
    }

    public final SwerveModuleState getTargetState() {
        return targetState;
    }

    public double getTargetStateAngle() {
        return targetState.angle.getDegrees();
    }
}