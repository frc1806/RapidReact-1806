package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj.Timer;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 * 
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class DrivePathActionWithTimeout implements Action {

    private PathContainer mPathContainer;
    private Path mPath;
    private DriveTrainSubsystem mDrive = DriveTrainSubsystem.getInstance();
    private double timeout;
    private double startTime;

    public DrivePathActionWithTimeout(PathContainer p, Double timeout) {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
        this.timeout = timeout;
    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithPath() || (Timer.getFPGATimestamp() > startTime + timeout);
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
        startTime = Timer.getFPGATimestamp();
    }
}
