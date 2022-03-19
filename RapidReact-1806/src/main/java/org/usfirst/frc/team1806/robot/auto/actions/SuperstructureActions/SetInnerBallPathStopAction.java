package org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.SuperStructure;

public class SetInnerBallPathStopAction implements Action {

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void start() {
        // TODO Auto-generated method stub
        SuperStructure.getInstance().wantInnerBallPathStop();
    }
    
}
