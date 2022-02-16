package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;


import org.usfirst.frc.team1806.robot.RobotState;

public class RunActionUntilX implements Action {

    private double triggerX;
    private double currentX;
    private double lastX;
    private Action action = null;

    public RunActionUntilX(double x, Action action){
        this.triggerX = x;
        this.action = action;
    }

    @Override
    public boolean isFinished() {
        currentX = RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x();
        return (lastX <= triggerX && currentX > triggerX) || (currentX <= triggerX && lastX > triggerX);   
    }

    @Override
    public void update() {
        action.update();
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
