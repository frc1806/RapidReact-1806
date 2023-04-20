package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;


import org.usfirst.frc.team1806.robot.RobotState;

public class RunActionUntilY implements Action{
    private double triggerY;
    private double currentY;
    private double lastY;
    private Action action = null;

    public RunActionUntilY(double y, Action action){
        this.triggerY = y;
        this.action = action;
    }
    @Override
    public boolean isFinished() {
        currentY = RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y();
        return (lastY <= triggerY && currentY > triggerY) || (currentY <= triggerY && lastY> triggerY);
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
        lastY = RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y();
        currentY = RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y();
    }
}
