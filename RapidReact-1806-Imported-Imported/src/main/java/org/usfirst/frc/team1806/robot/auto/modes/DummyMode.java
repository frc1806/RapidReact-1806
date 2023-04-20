package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.SwitchToLowPID;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ResetPoseFromPathAction;
import org.usfirst.frc.team1806.robot.auto.paths.DumbMode;
import org.usfirst.frc.team1806.robot.path.PathContainer;

public class DummyMode extends AutoModeBase {
	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer dumbMode = new DumbMode();
		runAction(new SwitchToLowPID());
		runAction(new ResetPoseFromPathAction(dumbMode));
		runAction(new DrivePathAction(dumbMode));
		//runAction(new TurnTowardsPoint(new Translation2d(0, 0)));
	}

}
