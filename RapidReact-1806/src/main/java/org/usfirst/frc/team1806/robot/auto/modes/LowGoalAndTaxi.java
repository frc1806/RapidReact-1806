package org.usfirst.frc.team1806.robot.auto.modes;

import java.nio.file.Path;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.Shoot;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ResetPoseFromPathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.WaitAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.UpOneFootRR;
import org.usfirst.frc.team1806.robot.game.Shot;
import org.usfirst.frc.team1806.robot.path.PathContainer;

public class LowGoalAndTaxi extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new Shoot(Shot.LOW_GOAL, 3.0));
        runAction(new WaitAction(Robot.autoDelay.getDouble(0.0) - 3.0));
        PathContainer mobilityPath = new UpOneFootRR(50, 50, 96, true);
        runAction(new ResetPoseFromPathAction(mobilityPath));
        runAction(new DrivePathAction(mobilityPath));
        
    }
    
}
