package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.FeatureFlags;
import org.usfirst.frc.team1806.robot.auto.actions.DriveToStall;
import org.usfirst.frc.team1806.robot.auto.actions.LiftActions.StandUpLift;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.CloseSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.ExtendSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.OpenSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.RetractSquid;
import org.usfirst.frc.team1806.robot.auto.actions.VisionPathExecuter;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.actions.BeelineAF;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.*;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.Arrays;

public class RightNonSendTwoHatch extends AutoModeBase {
    @Override
    public void routine() throws AutoModeEndedException {
        runAction(new RetractSquid());
        runAction(new OpenSquid());

        if(FeatureFlags.FF_LIFT_TILT){
            runAction(new StandUpLift());
        }
        runAction(new SwitchToHighPID());

        runAction(new SwitchToLowPID());

        PathContainer driveOffHab = new RightHabDriveOff();
        runAction(new ResetPoseFromPathAction(driveOffHab));
        runAction(new DrivePathAction(driveOffHab));

        runAction(new SwitchToHighPID());

        PathContainer driveToRocket = new RightSideHAB1ToCloseHatchRocketNoVision();
        runAction(new ResetPoseFromPathAction(driveToRocket));
        runAction(new DrivePathAction(driveToRocket));

        runAction(new SwitchToLowPID());
        runAction(new VisionPathExecuter());

        //Scoring sequence

        runAction(new ExtendSquid());
        runAction(new DriveToStall());

        runAction(new CloseSquid());

        runAction(new WaitAction(.4));
        runAction(new RetractSquid());

        runAction(new SwitchToLowPID());

        PathContainer backUpFromRocket = new DriveStraightPath(-13,30);
        runAction(new ResetPoseFromPathAction(backUpFromRocket));
        runAction(new DrivePathAction(backUpFromRocket));

        //runAction(new SwitchToHighPID());

        PathContainer whipPath = new WhipFromRocketRight();
        runAction(new ResetPoseFromPathAction(whipPath));
        runAction(new DrivePathAction(whipPath));

        /*
        PathContainer driveToFeeder = new ToFeederAfterWhipRIght();
        runAction(new DrivePathAction(driveToFeeder));

        //vision path
        runAction(new VisionPathExecuter());

        //Collect hatch
        runAction(new ExtendSquid());
        runAction(new DriveToStall());
        runAction(new WaitAction(1));
        runAction(new OpenSquid());
        runAction(new WaitAction(1.5));
        runAction(new RetractSquid());
        */

        //runAction(new WaitAction(15));

    }

}
