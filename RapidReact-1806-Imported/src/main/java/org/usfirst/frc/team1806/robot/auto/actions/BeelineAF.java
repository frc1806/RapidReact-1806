package org.usfirst.frc.team1806.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;

public class BeelineAF implements Action {
    DriveTrainSubsystem mDriveTrain = DriveTrainSubsystem.getInstance();
    double startTime = -1;
    double power;
    double time;

    public BeelineAF(){
        power = .5;
        time = .3;
    }

    public BeelineAF(double power, double time){
        this.power = power;
        this.time = time;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > time;
    }

    @Override
    public void update() {
        mDriveTrain.rightDrive(power);
        mDriveTrain.leftDrive(power);
    }

    @Override
    public void done() {
        mDriveTrain.rightDrive(0);
        mDriveTrain.leftDrive(0);
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }
}
