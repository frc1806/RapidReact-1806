package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;

import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;

import edu.wpi.first.wpilibj.Timer;



/**
 * Resets the robot's current pose based on the starting pose stored in the pathContainer object.
 * 
 * @see PathContainer
 * @see Action
 * @see RunOnceAction
 */
public class ResetPoseFromPathAction extends RunOnceAction {

    protected PathContainer mPathContainer;

    public ResetPoseFromPathAction(PathContainer pathContainer) {
        mPathContainer = pathContainer;
    }

    @Override
    public synchronized void runOnce() {
        RigidTransform2d startPose = mPathContainer.getStartPose();
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
        DriveTrainSubsystem.getInstance().setGyroAngle(startPose.getRotation());
    }
}