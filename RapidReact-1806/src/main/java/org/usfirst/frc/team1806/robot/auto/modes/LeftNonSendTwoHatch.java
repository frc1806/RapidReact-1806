package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.FeatureFlags;
import org.usfirst.frc.team1806.robot.auto.actions.BeelineAF;
import org.usfirst.frc.team1806.robot.auto.actions.LiftActions.StandUpLift;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.CloseSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.ExtendSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.OpenSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.RetractSquid;
import org.usfirst.frc.team1806.robot.auto.actions.VisionPathExecuter;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.*;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.Arrays;

public class LeftNonSendTwoHatch extends AutoModeBase {
    @Override
    public void routine() throws AutoModeEndedException {
        runAction(new ExtendSquid());
        if(FeatureFlags.FF_LIFT_TILT){
            runAction(new StandUpLift());
        }
        runAction(new SwitchToLowPID());

        PathContainer startPath = new LeftSideHAB1ToCloseHatchRocket();

        //Drive out to close side of rocket
        runAction(new ResetPoseFromPathAction(startPath));
        runAction(new DrivePathAction(startPath));

        //runAction(new TurnTowardsPoint(new Translation2d(210, 10)));

        runAction(new ParallelAction(Arrays.asList(
                new ExtendSquid(),
                new VisionPathExecuter()
        )));



        runAction(new BeelineAF(.25, .35));
        //Scoring sequence


        runAction(new SwitchToLowPID());
        runAction(new CloseSquid());
        runAction(new WaitAction(.3));

        //runAction(new SwitchToHighPID());

        //Give some time to lower lift to a safe driving height
/*
        //Concurrently finish lowering lift shoot backwards towards feeder station, turn towards it
        runAction(new DrivePathAction(new LeftSideCloseHatchRocketToFeeder()));
        runAction(new TurnTowardsPoint(new Translation2d(0,289)));

        runAction(new WaitAction(0.25));
        //vision path
        runAction(new VisionPathExecuter());

        //Collect hatch
        runAction(new ExtendSquid());
        runAction(new BeelineAF(0.3, 0.2));
        runAction(new OpenSquid());
        runAction(new WaitAction(1.5));
        runAction(new ParallelAction(Arrays.asList(new RetractSquid(), new DrivePathAction(new DriveStraightPath(-4, 20)))));

        //runAction(new WaitAction(15));
        */
    }

}
