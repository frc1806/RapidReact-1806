package org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.SuperStructure;

public class StopIntake implements Action {

    private SuperStructure mSuperStructure = SuperStructure.getInstance();

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {
        
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void start() {
        mSuperStructure.stop();
    }
    
}
