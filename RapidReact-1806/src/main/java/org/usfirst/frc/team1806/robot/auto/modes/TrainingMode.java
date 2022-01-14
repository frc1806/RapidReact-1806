package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.RightFeed2HCR2;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class TrainingMode extends AutoModeBase {
    /*
    So guys:
    This is gonna maybe be an auto if we don't frick it up. So let's not do that!
    In this auto, we're pre-loaded with a hatch and we're gonna patch up the first level of the right rocket.
    Let's win!
    ~CCP
    */

    @Override
    public void routine() throws AutoModeEndedException {
        PathContainer trainingDealio = new RightFeed2HCR2();//TrainingPath(); //I de-capitalized the variable name. Coding standards. -Dillon
        runAction(new ResetPoseFromPathAction(trainingDealio));
        runAction(new DrivePathAction(trainingDealio));
        runAction(new TurnTowardsPoint(new Translation2d(284,140)));
        runAction(new WaitAction(15));

    }
}
