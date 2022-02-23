package org.usfirst.frc.team1806.robot.game;

public class PolarCoordinate {
    public PolarCoordinate(double angle, double distance) {
        this.angle = angle;
        this.distance = distance;
    }
    public double getAngle() {
        return angle;
    }
    public double getDistance() {
        return distance;
    }
    private double angle;
    private double distance;

    
}
