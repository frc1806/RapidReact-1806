package org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.SuperStructure;

public class FrontFeedThrough implements Action{

    private SuperStructure mSuperStructure = SuperStructure.getInstance();

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void update() {
        mSuperStructure.wantFeedFrontIntake();
        
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void start() {
        mSuperStructure.wantFeedFrontIntake();
    }
    
}
