package org.usfirst.frc.team1806.robot.auto.actions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.LiftSubsystem;

public class LiftToHeight implements Action {

    private LiftSubsystem.LiftPosition wantedHeight;
    private boolean instant;
    public LiftToHeight(LiftSubsystem.LiftPosition _wantedHeight, boolean _instant) {
        wantedHeight = _wantedHeight;
        instant = _instant;
    }

    LiftSubsystem mLiftSubsystem = LiftSubsystem.getInstance();
    @Override
    public boolean isFinished() {
        return mLiftSubsystem.isAtPosition() || instant;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mLiftSubsystem.goToSetpoint(wantedHeight);
    }
}
