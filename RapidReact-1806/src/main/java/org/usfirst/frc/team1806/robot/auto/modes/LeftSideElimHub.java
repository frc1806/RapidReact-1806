package org.usfirst.frc.team1806.robot.auto.modes;

import java.util.Arrays;

import org.usfirst.frc.team1806.robot.auto.actions.SloppyTurnToHeading;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.BackFeedThrough;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.FrontFeedThrough;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.IntakeFront;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.Shoot;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.StopSuperStructure;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ParallelAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ResetPoseFromPathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.RunActionAtY;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.WaitAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.LeftSideElimHub1;
import org.usfirst.frc.team1806.robot.auto.paths.LeftSideElimHub2;
import org.usfirst.frc.team1806.robot.auto.paths.SixBallPath1;
import org.usfirst.frc.team1806.robot.game.Shot;
import org.usfirst.frc.team1806.robot.util.Rotation2d;

//Hi Chris

public class LeftSideElimHub extends AutoModeBase{

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new IntakeFront());
        runAction(new ResetPoseFromPathAction(new SixBallPath1()));
        runAction(new DrivePathAction(new SixBallPath1()));
        runAction(new StopSuperStructure());
        runAction(new SloppyTurnToHeading(Rotation2d.fromDegrees(153), 2.0));
        runAction(new Shoot(Shot.TARMAC_EDGE_SHOT_FLIPPED, 3.5));
        runAction(new StopSuperStructure());
        runAction(new FrontFeedThrough());
        runAction(new DrivePathAction(new LeftSideElimHub1()));
        runAction(new StopSuperStructure());
        runAction(new ParallelAction(Arrays.asList(
            new DrivePathAction(new LeftSideElimHub2()),
            new RunActionAtY(190.0, new BackFeedThrough())
        )));
        runAction(new WaitAction(2.0));
        runAction(new StopSuperStructure());
        
    }
    
}
