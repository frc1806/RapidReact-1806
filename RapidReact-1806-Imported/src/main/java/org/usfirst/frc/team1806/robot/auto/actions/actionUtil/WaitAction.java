package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time To use this Action, call runAction(new WaitAction(your_time))
 */
public class WaitAction implements Action {

    private double mTimeToWait;
    private double mStartTime;

    public WaitAction(double timeToWait) {
        mTimeToWait = timeToWait;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime >= mTimeToWait;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
    	System.out.println("Wait is done!");
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();

    }
}