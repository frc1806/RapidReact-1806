package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.FeatureFlags;
import org.usfirst.frc.team1806.robot.auto.actions.LiftActions.StandUpLift;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.ExtendSquid;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.SwitchToLowPID;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.LeftCamTaniumDefend;
import org.usfirst.frc.team1806.robot.auto.paths.RightCamTaniumDefend;

public class RightCamTaniumDefendMode extends AutoModeBase {
    @Override
    public void routine() throws AutoModeEndedException {
        if (FeatureFlags.FF_LIFT_TILT) {
            runAction(new StandUpLift());
        }
        runAction(new SwitchToLowPID());

        runAction(new DrivePathAction(new RightCamTaniumDefend()));

    }

}
