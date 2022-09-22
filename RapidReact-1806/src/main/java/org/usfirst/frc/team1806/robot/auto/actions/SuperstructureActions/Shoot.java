package org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.game.Shot;
import org.usfirst.frc.team1806.robot.subsystems.SuperStructure;

import edu.wpi.first.wpilibj.Timer;

public class Shoot implements Action {

    private SuperStructure mSuperStructure = SuperStructure.getInstance();
    private Shot mShot;
    private Double start;
    private Double end;
    private Double mDelayTime;
    public Shoot(Shot shot, Double delayTime){
        mShot = shot;
        mDelayTime = delayTime;
    }

    @Override
    public boolean isFinished() {
        end = Timer.getFPGATimestamp();
        if ((end - start) >= mDelayTime) return true;
        return false;
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
        start = Timer.getFPGATimestamp();
        mSuperStructure.wantPrepareShot(mShot);
        mSuperStructure.wantConfirmLaunch(true);
    }

    
}
