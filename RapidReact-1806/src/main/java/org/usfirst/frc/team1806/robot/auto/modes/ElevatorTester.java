package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.ElevatorActions.ElevatorToHeight;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.WaitAction;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;

/**
 * Lift Tester auto will be used to test our lift very quickly whenever we get to an event
 *
 * Self-Tester
 */
public class ElevatorTester extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ElevatorToHeight(100, false));
        runAction(new WaitAction(1));
        runAction(new ElevatorToHeight(2, false));
        runAction(new WaitAction(1));
        runAction(new ElevatorToHeight(5, false));
        runAction(new WaitAction(1));
        runAction(new ElevatorToHeight(0, false));
        runAction(new WaitAction(15));
    }
}
