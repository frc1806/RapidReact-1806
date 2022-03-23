package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.SloppyTurnToHeading;
import org.usfirst.frc.team1806.robot.auto.actions.SloppyTurnToPoint;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.IntakeFront;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.SetInnerBallPathIntakeAction;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.SetInnerBallPathStopAction;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.Shoot;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.StopSuperStructure;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ResetPoseFromPathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.WaitAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.SixBallPath1;
import org.usfirst.frc.team1806.robot.game.Shot;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;



public class LeftTwoBall extends AutoModeBase{

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new IntakeFront());
        runAction(new ResetPoseFromPathAction(new SixBallPath1()));
        runAction(new DrivePathAction(new SixBallPath1()));
        runAction(new StopSuperStructure());
        runAction(new SloppyTurnToHeading(Rotation2d.fromDegrees(153), 5.0));
        runAction(new Shoot(Shot.TARMAC_EDGE_SHOT_FLIPPED, 4.0));
        runAction(new StopSuperStructure());
        runAction(new SetInnerBallPathIntakeAction());
        runAction(new WaitAction(0.5));
        runAction(new Shoot(Shot.TARMAC_EDGE_SHOT_FLIPPED, 4.0));
        runAction(new StopSuperStructure());
        runAction(new SetInnerBallPathStopAction());
        
    }
    
}
