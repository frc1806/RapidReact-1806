package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.DumbMode;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class TurnTester extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {

        runAction(new ResetPoseFromPathAction(new DumbMode()));
        runAction(new WaitAction(1));
        runAction(new TurnTowardsPoint(new Translation2d(30,10)));
        runAction(new TurnTowardsPoint(new Translation2d(30,10)));
        System.out.println("Finished 1st point");
        runAction(new WaitAction(1));
        runAction(new TurnTowardsPoint(new Translation2d(30,0)));
        System.out.println("Finished 2nd point");
        runAction(new WaitAction(1));
        runAction(new TurnTowardsPoint(new Translation2d(30,-10)));
        System.out.println("Finished 3nd point");
        runAction(new WaitAction(1));
        runAction(new TurnTowardsPoint(new Translation2d(30,0)));
        System.out.println("Finished 4nd point");
        runAction(new TurnTowardsPoint(new Translation2d(0,-10)));
        runAction(new TurnTowardsPoint(new Translation2d(30,10)));
    }
}
