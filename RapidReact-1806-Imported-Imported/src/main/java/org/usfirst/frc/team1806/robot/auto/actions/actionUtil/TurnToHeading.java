package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;

import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;
import org.usfirst.frc.team1806.robot.util.DriveSignal;
import org.usfirst.frc.team1806.robot.util.Rotation2d;

public class TurnToHeading implements Action {

    private double mTargetHeading;
    private DriveTrainSubsystem mDrive = DriveTrainSubsystem.getInstance();

    public TurnToHeading(double heading) {
        mTargetHeading = heading;
    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithTurn();
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {
    	mDrive.setCoastMode();
    	mDrive.setOpenLoop(new DriveSignal(0, 0));
    }

    @Override
    public void start() {
    	mDrive.setBrakeMode();
    	System.out.println("yooo turning");
        mDrive.setWantTurnToHeading(Rotation2d.fromDegrees(mTargetHeading));
    }
}