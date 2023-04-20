package org.usfirst.frc.team1806.robot.auto.actions.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.util.XboxController;


public class VibrateControllerForTime implements Action {
    Timer timer;
    double wantedTime;
    XboxController controller;
    public VibrateControllerForTime(double time, XboxController contoller){
        timer = new Timer();
        wantedTime = time;
        this.controller = contoller;
    }
    @Override
    public boolean isFinished() {
        return timer.get() >= wantedTime;
    }

    @Override
    public void update() {
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        controller.setRumble(GenericHID.RumbleType.kRightRumble, 1);
    }

    @Override
    public void done() {
        System.out.println("stopping vibration");
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }

    @Override
    public void start() {
        timer.reset();
        System.out.println("starting vibration");
        timer.start();
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        controller.setRumble(GenericHID.RumbleType.kRightRumble, 1);
    }
}
