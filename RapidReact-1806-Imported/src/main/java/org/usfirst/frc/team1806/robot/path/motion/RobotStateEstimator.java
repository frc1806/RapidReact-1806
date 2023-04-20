package org.usfirst.frc.team1806.robot.path.motion;

import org.usfirst.frc.team1806.robot.Kinematics;
import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Twist2d;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator implements Loop {
    static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {
    }

    RobotState robot_state_ = RobotState.getInstance();
    DriveTrainSubsystem drive_ = DriveTrainSubsystem.getInstance();
    double left_encoder_prev_distance_ = 0;
    double right_encoder_prev_distance_ = 0;

    @Override
    public synchronized void onStart(double timestamp) {
        left_encoder_prev_distance_ = drive_.getLeftDistanceInches();
        right_encoder_prev_distance_ = drive_.getRightDistanceInches();
    }

    @Override
    public synchronized void onLoop(double timestamp) {
        final double left_distance = drive_.getLeftDistanceInches();
        final double right_distance = drive_.getRightDistanceInches();
        final Rotation2d gyro_angle = drive_.getGyroYaw();
        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(drive_.getLeftVelocityInchesPerSec(),
                drive_.getRightVelocityInchesPerSec());
        robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = 
        		right_distance;
    }

    @Override
    public void onStop(double timestamp) {
        // no-op
    }

}