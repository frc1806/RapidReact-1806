package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.Shoot;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.StopSuperStructure;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ResetPoseFromPathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.WaitAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.MobilityPath;
import org.usfirst.frc.team1806.robot.game.Shot;
import org.usfirst.frc.team1806.robot.path.PathContainer;

public class HighGoalAndTaxi extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new Shoot(Shot.FLIPPED_CLOSE_SHOT, 3.0));
        runAction(new StopSuperStructure());
        runAction(new WaitAction(Robot.autoDelay.getDouble(0.0) - 3.0));
        PathContainer mobilityPath = new MobilityPath();
        runAction(new ResetPoseFromPathAction(mobilityPath));
        runAction(new DrivePathAction(mobilityPath));
        
    }
    
}
