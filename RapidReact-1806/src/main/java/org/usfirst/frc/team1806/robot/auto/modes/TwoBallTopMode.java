package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.IntakeFront;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.Shoot;
import org.usfirst.frc.team1806.robot.auto.actions.SuperstructureActions.StopIntake;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ResetPoseFromPathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.WaitAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.TwoBallTopBackMore;
import org.usfirst.frc.team1806.robot.auto.paths.SixBallPath1;
import org.usfirst.frc.team1806.robot.game.Shot;

public class TwoBallTopMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(Robot.autoDelay.getDouble(0.0)));
        runAction(new IntakeFront());
        runAction(new ResetPoseFromPathAction(new SixBallPath1()));
        runAction(new DrivePathAction(new SixBallPath1()));
        runAction(new StopIntake());
        runAction(new Shoot(new Shot(175.0, 2500.0, 2300.0, false, false), 0.5));
        runAction(new DrivePathAction(new TwoBallTopBackMore()));
    }
    
}
