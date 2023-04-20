package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;

import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;

public class SwitchToHighPID implements Action {
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
        DriveTrainSubsystem.isWantedLowPID = false;
    }
}
