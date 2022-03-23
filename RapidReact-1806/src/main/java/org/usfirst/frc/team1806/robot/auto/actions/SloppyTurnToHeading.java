package org.usfirst.frc.team1806.robot.auto.actions;

import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;
import org.usfirst.frc.team1806.robot.util.DriveSignal;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import edu.wpi.first.wpilibj.Timer;

public class SloppyTurnToHeading implements Action{

    private Rotation2d mTargetHeading;
    private DriveTrainSubsystem mDrive = DriveTrainSubsystem.getInstance();
    double startTime;
    double timeout;

    public SloppyTurnToHeading(Rotation2d heading, double timeout) {
        mTargetHeading = heading;
        System.out.println("target heading " + mTargetHeading.getDegrees());
        this.timeout = timeout;
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return Math.abs(mDrive.getGyroYaw().getDegrees() - mTargetHeading.getDegrees()) < 5.0 || Timer.getFPGATimestamp() > startTime + timeout;
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        double error = -(mTargetHeading.getDegrees() - mDrive.getGyroYaw().getDegrees());
        mDrive.setOpenLoop(new DriveSignal(error * 0.015, -error * 0.015));
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }
    
}
