package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.LiftActions.LiftToHeight;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.WaitAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.subsystems.LiftSubsystem;

/**
 * Lift Tester auto will be used to test our lift very quickly whenever we get to an event
 *
 * Self-Tester
 */
public class LiftTester extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new LiftToHeight(100, false));
        runAction(new WaitAction(1));
        runAction(new LiftToHeight(2, false));
        runAction(new WaitAction(1));
        runAction(new LiftToHeight(5, false));
        runAction(new WaitAction(1));
        runAction(new LiftToHeight(0, false));
        runAction(new WaitAction(15));
    }
}
