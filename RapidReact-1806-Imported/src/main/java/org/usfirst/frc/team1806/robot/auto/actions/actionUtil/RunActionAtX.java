package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;

import org.usfirst.frc.team1806.robot.RobotState;

public class RunActionAtX implements Action{
    private double triggerX;
    private double currentX;
    private double lastX;
    private Action action = null;
    private boolean hasRunAction = false;
    public RunActionAtX(double x, Action action){
        this.triggerX = x;
        this.action = action;
    }
    @Override
    public boolean isFinished() {
        return action.isFinished();
    }

    @Override
    public void update() {
        currentX = RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x();
        if((lastX <= triggerX && currentX > triggerX) || (currentX <= triggerX && lastX > triggerX) && !hasRunAction){
            action.start();
            hasRunAction = true;
        }
        lastX = currentX;
        if(hasRunAction){
            action.update();
        }
    }

    @Override
    public void done() {
        action.done();

    }

    @Override
    public void start() {
        lastX = RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x();
        currentX = RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x();
    }
}
