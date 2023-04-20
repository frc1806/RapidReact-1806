package org.usfirst.frc.team1806.robot.util;

public class TargetInfo {
    public TargetInfo(double distanceToTarget, double angleToTarget) {
        this.distanceToTarget = distanceToTarget;
        this.angleToTarget = angleToTarget;
    }
    public double getDistanceToTarget() {
        return distanceToTarget;
    }
    public double getAngleToTarget() {
        return angleToTarget;
    }
    double distanceToTarget;
    double angleToTarget;

    
}
