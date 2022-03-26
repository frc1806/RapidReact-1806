package org.usfirst.frc.team1806.robot.auto.modes;

import java.util.Arrays;

import org.usfirst.frc.team1806.robot.auto.actions.SloppyTurnToHeading;
import org.usfirst.frc.team1806.robot.auto.actions.SloppyTurnToPoint;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.IntakeBack;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.IntakeFront;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.SetInnerBallPathIntakeAction;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.SetInnerBallPathStopAction;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.Shoot;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.StopSuperStructure;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathActionWithTimeout;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ParallelAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ResetPoseFromPathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.RunActionAtAngle;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.WaitAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.Right3Ball1;
import org.usfirst.frc.team1806.robot.auto.paths.Right3Ball2;
import org.usfirst.frc.team1806.robot.game.Shot;
import org.usfirst.frc.team1806.robot.util.Rotation2d;



public class Right3Ball extends AutoModeBase{

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new IntakeBack());
        runAction(new ResetPoseFromPathAction(new Right3Ball1()));
        runAction(new DrivePathAction(new Right3Ball1()));
        runAction(new WaitAction(0.5));
        runAction(new StopSuperStructure());
        runAction(new ParallelAction(Arrays.asList(
            new SloppyTurnToHeading(Rotation2d.fromDegrees(75), 5.0),
            new Shoot(Shot.TARMAC_EDGE_SHOT, 3.7)
        )));
        runAction(new StopSuperStructure());
        runAction(new SloppyTurnToHeading(Rotation2d.fromDegrees(-27.9), 5.0));
        runAction(new IntakeBack());
        runAction(new DrivePathActionWithTimeout(new Right3Ball2(), 6.0));
        runAction(new StopSuperStructure());
        runAction(new ParallelAction(Arrays.asList(
            new SloppyTurnToHeading(Rotation2d.fromDegrees(40.0), 5.0),
            new Shoot(Shot.AUTO_FAR_TARMAC_EDGE, 4.0)
        )));
        runAction(new StopSuperStructure());
        
    }
    
}
