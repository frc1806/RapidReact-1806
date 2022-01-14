package org.usfirst.frc.team1806.robot.auto.actions.actionUtil;

import java.awt.Robot;
import java.sql.Driver;

import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;
import org.usfirst.frc.team1806.robot.util.DriveSignal;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;

public class TurnTowardsPoint implements Action {

    private Rotation2d mTargetHeading;
    private DriveTrainSubsystem mDrive = DriveTrainSubsystem.getInstance();
    
    
    public TurnTowardsPoint(Translation2d point) {
            mTargetHeading = new Rotation2d(point.subtract(RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation()), true);
            System.out.println("target heading " + mTargetHeading.getDegrees());
        }

        @Override
        public boolean isFinished() {
            return mDrive.isDoneWithTurn();
        }

        @Override
        public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
        }

        @Override
        public void done() {
            mDrive.setCoastMode();
            mDrive.setOpenLoop(new DriveSignal(0, 0));
            System.out.println("Finished turning to point.");
        }

        @Override
        public void start() {
            mDrive.setBrakeMode();
            System.out.println("Turning to point.");
        mDrive.setWantTurnToHeading(mTargetHeading);
    }
}
