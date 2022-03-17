package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.IntakeBack;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.IntakeFront;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.Shoot;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.StopSuperStructure;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ResetPoseFromPathAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.TwoBallClose1;
import org.usfirst.frc.team1806.robot.auto.paths.TwoBallClose2;
import org.usfirst.frc.team1806.robot.auto.paths.TwoBallClose3;
import org.usfirst.frc.team1806.robot.game.Shot;

public class TwoBallCloseMode extends AutoModeBase{
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ResetPoseFromPathAction(new TwoBallClose1()));
        runAction(new IntakeBack());
        runAction(new DrivePathAction(new TwoBallClose1()));
        
        runAction(new DrivePathAction(new TwoBallClose2()));
        runAction(new Shoot(Shot.LOW_GOAL, 5.0));
        runAction(new IntakeBack());
        runAction(new DrivePathAction(new TwoBallClose3()));
        runAction(new StopSuperStructure());
    }
}

