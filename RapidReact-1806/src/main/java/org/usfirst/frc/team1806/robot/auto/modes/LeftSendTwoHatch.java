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
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.*;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.Arrays;

public class LeftSendTwoHatch extends AutoModeBase {
    @Override
    public void routine() throws AutoModeEndedException {
        runAction(new RetractSquid());
        runAction(new OpenSquid());

        if(FeatureFlags.FF_LIFT_TILT){
            runAction(new StandUpLift());
        }

        runAction(new SwitchToLowPID());

        PathContainer driveOffHab = new RightHab2DriveOff();
        runAction(new ResetPoseFromPathAction(driveOffHab));
        runAction(new ParallelAction(Arrays.asList(
                new DrivePathAction(driveOffHab),
                new RunActionAtX(107, new SeriesAction(Arrays.asList(new ForceEndPathAction(), new DrivePathAction(new LeftHab2ToFarRocket())))),
                new RunActionAtX(282, new SeriesAction(Arrays.asList(new ForceEndPathAction(), new VisionPathExecuter())))
        )));

        runAction(new ExtendSquid());
        runAction(new ParallelAction(Arrays.asList(new DriveToStall(), new SeriesAction(Arrays.asList(new WaitAction(0.3), new CloseSquid())))));

        runAction(new RetractSquid());

        runAction(new ParallelAction(Arrays.asList(
                new DrivePathAction(new RightFarRocketBackUp()),
                new  RunActionAtX(284, new SeriesAction(Arrays.asList(new ForceEndPathAction(),new DrivePathAction(new LeftFarRocketToLeftFeeder())))),
                new  RunActionAtX(90, new SeriesAction(Arrays.asList(new ForceEndPathAction(), new VisionPathExecuter())))
        )));
        runAction(new ExtendSquid());
        runAction(new DriveToStall());
        runAction(new OpenSquid());
        runAction(new WaitAction( 0.5));

        runAction(new RetractSquid());
        runAction(new DrivePathAction( new LeftFeederToCloseLeftRocket()));
        runAction(new TurnTowardsPoint(new Translation2d(215, 20)));
        runAction(new WaitAction(0.5));
        runAction(new VisionPathExecuter());

        runAction(new ExtendSquid());
        runAction(new ParallelAction(Arrays.asList(new DriveToStall(), new SeriesAction(Arrays.asList(new WaitAction(0.3), new CloseSquid())))));

        runAction(new RetractSquid());

//        runAction(new CloseSquid());
//
//        runAction(new WaitAction(.4));
//        runAction(new RetractSquid());
//
//        runAction(new SwitchToLowPID());
//
//        PathContainer backUpFromRocket = new DriveStraightPath(-13,30);
//        runAction(new ResetPoseFromPathAction(backUpFromRocket));
//        runAction(new DrivePathAction(backUpFromRocket));
//
//        //runAction(new SwitchToHighPID());
//
//        PathContainer whipPath = new WhipFromRocketRight();
//        runAction(new ResetPoseFromPathAction(whipPath));
//        runAction(new DrivePathAction(whipPath));

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
