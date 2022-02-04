package org.usfirst.frc.team1806.robot.auto.actions.LiftActions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.ElevatorSubsystem;

public class LiftToHeight implements Action {

    private double wantedHeight;
    private boolean instant;
    public LiftToHeight(double position, boolean _instant) {
        wantedHeight = position;
        instant = _instant;
    }

    ElevatorSubsystem mLiftSubsystem = ElevatorSubsystem.getInstance();
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
        mLiftSubsystem.goToSetpointInches(wantedHeight);
    }
}
