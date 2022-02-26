package org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.SuperStructure;

public class IntakeBack implements Action{

    private SuperStructure mSuperStructure;

    @Override
    public boolean isFinished() {
        return true;
    }
    
    @Override
    public void update() {
        mSuperStructure.wantIntakeBack();        
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void start() {
        mSuperStructure.wantIntakeBack();
        
    }
    
}
