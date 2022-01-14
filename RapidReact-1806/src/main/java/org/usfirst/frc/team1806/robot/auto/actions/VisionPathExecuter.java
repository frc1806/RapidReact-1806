package org.usfirst.frc.team1806.robot.auto.actions;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.auto.modes.VisionMode;

public class VisionPathExecuter implements Action {
    VisionMode visionMode;
    boolean started = false;
    @Override
    public boolean isFinished() {
        return started && visionMode.getIsDone();
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
        visionMode.stop();
    }

    @Override
    public void start() {
        visionMode = new VisionMode();
        visionMode.run();
        started = true;
    }
}
