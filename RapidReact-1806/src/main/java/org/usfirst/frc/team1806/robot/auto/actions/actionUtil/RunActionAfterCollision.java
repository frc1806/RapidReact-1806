package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;

public class RunActionAfterCollision implements Action {

    private DriveTrainSubsystem mDriveTrainSubsystem;
    private Action action = null;
    private boolean hasActionRun;
    private float lastAcceleration = 0;
    private float currentAcceleration = 0;


    public RunActionAfterCollision(Action actionToRun) {
        mDriveTrainSubsystem = DriveTrainSubsystem.getInstance();
        action = actionToRun;
        hasActionRun = false;

    }

    @Override
    public boolean isFinished() {return action.isFinished();}



    @Override
    public void update() {
    currentAcceleration = mDriveTrainSubsystem.getWorldLinearAccelZ();
    if (currentAcceleration - lastAcceleration >= Constants.habDropAccelerationThreshold && !hasActionRun){
        action.start();
        hasActionRun = true;
    }
    if (hasActionRun){
        action.update();
    }
    lastAcceleration = currentAcceleration;
    }


    @Override
    public void done() { action.done();}

    @Override
    public void start() {

    }

}