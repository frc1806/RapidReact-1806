package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.FeatureFlags;
import org.usfirst.frc.team1806.robot.auto.actions.BeelineAF;
import org.usfirst.frc.team1806.robot.auto.actions.LiftActions.StandUpLift;
import org.usfirst.frc.team1806.robot.auto.actions.LiftToHeight;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.CloseSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.ExtendSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.OpenSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.RetractSquid;
import org.usfirst.frc.team1806.robot.auto.actions.VisionPathExecuter;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.*;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.lang.reflect.Array;
import java.util.Arrays;

public class PurePathingNonSendTwoHatchRight extends AutoModeBase {
    @Override
    public void routine() throws AutoModeEndedException {
    runAction(new OpenSquid());
    runAction(new RetractSquid());

    runAction(new WaitAction(.15));
    if(FeatureFlags.FF_LIFT_TILT){
        runAction(new StandUpLift());
    }
    runAction(new SwitchToLowPID());
    //Drive out to close side of rocket

    runAction(new DrivePathAction(new RightHabDriveOff()));
    PathContainer pc = new RightSideHAB1ToCloseHatchRocketNoVision();
    runAction(new ResetPoseFromPathAction(pc));
    runAction(new DrivePathAction(pc));

    pc = new DriveStraightPath(29, 35);
    runAction(new ResetPoseFromPathAction(pc));
    runAction(new DrivePathAction(pc));
    runAction(new ExtendSquid());

    runAction(new WaitAction(.8));
    runAction(new CloseSquid());
    runAction(new WaitAction(.8));
    pc = new DriveStraightPath(-4, 35);
    runAction(new ResetPoseFromPathAction(pc));
    runAction(new DrivePathAction(pc));
    runAction(new WaitAction(15));
//runAction(new ResetPoseFromPathAction(new RightSideHAB1ToCloseHatchRocketNoVision()));

    //runAction(new TurnTowardsPoint(new Translation2d(210, 10)));



    //runAction(new BeelineAF(.25, .35));
    //Scoring sequence


    runAction(new SwitchToLowPID());
    runAction(new CloseSquid());
   // runAction(new WaitAction(.3));

    //runAction(new SwitchToHighPID());

    //Give some time to lower lift to a safe driving height
/*
        //Concurrently finish lowering lift shoot backwards towards feeder station, turn towards it
        runAction(new DrivePathAction(new RightSideCloseHatchRocketToFeeder()));
        runAction(new TurnTowardsPoint(new Translation2d(0,35)));

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
