package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 * 
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class DrivePathAction implements Action {

    private PathContainer mPathContainer;
    private Path mPath;
    private DriveTrainSubsystem mDrive = DriveTrainSubsystem.getInstance();

    public DrivePathAction(PathContainer p) {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithPath();
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {
        mDrive.stopDrive();
        System.out.println("Done with path following!");
    }

    @Override
    public void start() {
        DriveTrainSubsystem.getInstance().reloadHighGearVelocityGains();
        mDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
//        mDrive.setHighGear(true);
    }
}
