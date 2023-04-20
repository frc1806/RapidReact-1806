package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;

import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;

public class SwitchToLowPID implements Action {
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
        DriveTrainSubsystem.isWantedLowPID = true;
    }
}
