package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.FeatureFlags;
import org.usfirst.frc.team1806.robot.auto.actions.LiftActions.StandUpLift;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.ExtendSquid;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.DumbMode;
import org.usfirst.frc.team1806.robot.auto.paths.PIDTestPath;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.beans.FeatureDescriptor;

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
