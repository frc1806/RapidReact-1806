package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;

import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;

/**
 * Forces the current path the robot is driving on to end early
 * 
 * @see DrivePathAction
 * @see Action
 * @see RunOnceAction
 */
public class ForceEndPathAction extends RunOnceAction {

    @Override
    public synchronized void runOnce() {
        DriveTrainSubsystem.getInstance().forceDoneWithPath();
    }
}