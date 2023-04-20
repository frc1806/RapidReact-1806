package org.usfirst.frc.team1806.robot.auto.actions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;

public class DriveToStall implements Action {
    DriveTrainSubsystem mDrive = DriveTrainSubsystem.getInstance();
    @Override
    public boolean isFinished() {
        return mDrive.driveToStall(false, true);
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mDrive.driveToStall(true, false);
    }
}
