package org.usfirst.frc.team1806.robot.auto.actions.SquidActions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.RunOnceAction;
import org.usfirst.frc.team1806.robot.subsystems.SquidSubsystem;

public class OpenSquid implements Action{
    SquidSubsystem mSquidSubsystem = SquidSubsystem.getInstance();
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
        mSquidSubsystem.openSquid();
    }
}
