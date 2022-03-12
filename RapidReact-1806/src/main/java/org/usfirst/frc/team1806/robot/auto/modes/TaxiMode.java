package org.usfirst.frc.team1806.robot.auto.modes;

import java.nio.file.Path;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ResetPoseFromPathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.WaitAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.UpOneFootRR;
import org.usfirst.frc.team1806.robot.auto.paths.UpOneFootY;
import org.usfirst.frc.team1806.robot.path.PathContainer;

public class TaxiMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        // TODO Auto-generated method stub
        runAction(new WaitAction(Robot.autoDelay.getDouble(0.0)));
        PathContainer mobilityPath = new UpOneFootY(50, 50, -48, true);
        runAction(new ResetPoseFromPathAction(mobilityPath));
        runAction(new DrivePathAction(mobilityPath));
        
    }
    
}
