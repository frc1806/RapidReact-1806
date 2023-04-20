package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.PIDTestPath;
import org.usfirst.frc.team1806.robot.path.PathContainer;

public class PIDTestMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer PID_path = new PIDTestPath();
        runAction(new SwitchToHighPID());
        runAction(new ResetPoseFromPathAction(PID_path));
        runAction(new DrivePathAction(PID_path));
        runAction(new WaitAction(15));
        //runAction(new TurnTowardsPoint(new Translation2d(0, 0)));
    }

}
