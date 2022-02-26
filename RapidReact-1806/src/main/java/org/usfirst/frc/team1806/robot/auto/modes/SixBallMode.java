package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.IntakeBack;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.IntakeFront;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.Shoot;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.StopIntake;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ResetPoseFromPathAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.SixBallPath1;
import org.usfirst.frc.team1806.robot.auto.paths.SixBallPath2;
import org.usfirst.frc.team1806.robot.auto.paths.SixBallPath3;
import org.usfirst.frc.team1806.robot.auto.paths.SixBallPath4;
import org.usfirst.frc.team1806.robot.auto.paths.SixBallPath5;
import org.usfirst.frc.team1806.robot.game.Shot;

public class SixBallMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new IntakeFront());
        runAction(new ResetPoseFromPathAction(new SixBallPath1()));
        runAction(new StopIntake());
        runAction(new Shoot(new Shot(175.0, 2500.0, 2300.0, false, false), 0.5));
        runAction(new IntakeBack());
        runAction(new ResetPoseFromPathAction(new SixBallPath2()));
        runAction(new StopIntake());
        runAction(new Shoot(new Shot(175.0, 2500.0, 2300.0, false, false), 0.5));
        runAction(new IntakeBack());
        runAction(new ResetPoseFromPathAction(new SixBallPath3()));
        runAction(new StopIntake());
        runAction(new Shoot(new Shot(175.0, 2500.0, 2300.0, false, false), 0.5));
        runAction(new IntakeFront());
        runAction(new ResetPoseFromPathAction(new SixBallPath4()));
        runAction(new StopIntake());
        runAction(new Shoot(new Shot(175.0, 2500.0, 2300.0, false, false), 0.5));
        runAction(new IntakeBack());
        runAction(new ResetPoseFromPathAction(new SixBallPath5()));
        runAction(new StopIntake());
        runAction(new Shoot(new Shot(175.0, 2500.0, 2300.0, false, false), 0.5));
        
        
    }
    
}
