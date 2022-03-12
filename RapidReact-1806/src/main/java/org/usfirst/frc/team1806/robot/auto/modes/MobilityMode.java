package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ResetPoseFromPathAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.MobilityPath;

public class MobilityMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ResetPoseFromPathAction(new MobilityPath()));
        runAction(new DrivePathAction(new MobilityPath()));
        
    }
    
}
