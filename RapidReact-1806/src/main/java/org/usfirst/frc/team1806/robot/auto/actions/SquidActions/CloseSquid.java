package org.usfirst.frc.team1806.robot.auto.actions.SquidActions;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.SquidSubsystem;

public class CloseSquid implements Action {
    SquidSubsystem mSquidSubsystem = SquidSubsystem.getInstance();

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mSquidSubsystem.closeSquid();
    }
}
