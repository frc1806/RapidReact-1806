package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.CloseSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.ExtendSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.RetractSquid;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.ParallelAction;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.SwitchToLowPID;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.WaitAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.RocketPos;
import org.usfirst.frc.team1806.robot.auto.paths.DriveStraightPath;
import org.usfirst.frc.team1806.robot.subsystems.LiftSubsystem;

import java.util.Arrays;

public class ScoreHatchBetterThanJoel extends AutoModeBase {
    RocketPos rockPos;



    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ExtendSquid());
        runAction(new WaitAction(.2));
        runAction(new SwitchToLowPID());
        runAction(new DrivePathAction(new DriveStraightPath(2, 25)));
        runAction(new WaitAction(1.5));
        runAction(new CloseSquid());
        runAction(new WaitAction(1));
        runAction(new ParallelAction(Arrays.asList(new RetractSquid(), new DrivePathAction(new DriveStraightPath(-3, 20)))));

    }
}
