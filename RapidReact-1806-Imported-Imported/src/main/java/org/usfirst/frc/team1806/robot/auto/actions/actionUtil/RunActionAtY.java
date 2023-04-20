package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;


import org.usfirst.frc.team1806.robot.RobotState;

public class RunActionAtY implements Action{
    private double triggerY;
    private double currentY;
    private double lastY;
    private Action action = null;
    private boolean hasRunAction = false;
    public RunActionAtY(double y, Action action){
        this.triggerY = y;
        this.action = action;
    }
    @Override
    public boolean isFinished() {
        return action.isFinished();
    }

    @Override
    public void update() {
        currentY = RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y();
        if((lastY <= triggerY && currentY > triggerY) || (currentY <= triggerY && lastY> triggerY) && !hasRunAction){
            action.start();
            hasRunAction = true;
        }
        lastY = currentY;
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
        lastY = RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y();
        currentY = RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y();
    }
}
