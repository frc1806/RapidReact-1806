package org.usfirst.frc.team1806.robot.auto.actions.LiftActions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.LiftSubsystem;

public class LeanBackLift implements Action {

    LiftSubsystem mLiftSubsystem = LiftSubsystem.getInstance();

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mLiftSubsystem.leanLiftBack();
    }
}
