package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.IntakeBack;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.IntakeFront;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.Shoot;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.StopSuperStructure;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ResetPoseFromPathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.TurnTowardsPoint;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.WaitAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.SixBallPath1;
import org.usfirst.frc.team1806.robot.auto.paths.SixBallPath2;
import org.usfirst.frc.team1806.robot.auto.paths.SixBallPath3;
import org.usfirst.frc.team1806.robot.auto.paths.SixBallPath4;
import org.usfirst.frc.team1806.robot.game.Shot;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class SixBallMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new IntakeFront());
        runAction(new ResetPoseFromPathAction(new SixBallPath1()));
        runAction(new DrivePathAction(new SixBallPath1()));
        runAction(new StopSuperStructure());
        runAction(new TurnTowardsPoint(new Translation2d(100, 300)));
        runAction(new Shoot(Shot.TARMAC_EDGE_SHOT_FLIPPED, 4.0));
        runAction(new StopSuperStructure());
        runAction(new IntakeBack());
        runAction(new DrivePathAction(new SixBallPath2()));
        runAction(new StopSuperStructure());
        runAction(new IntakeBack());
        runAction(new DrivePathAction(new SixBallPath3()));
        runAction(new StopSuperStructure());
        runAction(new TurnTowardsPoint(new Translation2d(327, 163)));
        runAction(new Shoot(Shot.TARMAC_EDGE_SHOT, 2.0));
        runAction(new StopSuperStructure());
        runAction(new IntakeFront());
        runAction(new DrivePathAction(new SixBallPath4()));
        runAction(new StopSuperStructure());
        runAction(new TurnTowardsPoint(new Translation2d(0, 40)));
        runAction(new Shoot(Shot.BIG_SHOT_FLIPPED, 2.0));
        runAction(new StopSuperStructure());
        runAction(new WaitAction(5.0));
    }
    
}
