package org.usfirst.frc.team1806.robot.game;

public class shot {
    Double liftHeight, launcherAngle, baseSpeed, baseSpin;

    public Double getLiftHeight() {
        return liftHeight;
    }

    public Double getLauncherAngle() {
        return launcherAngle;
    }

    public Double getBaseSpeed() {
        return baseSpeed;
    }

    public Double getBaseSpin() {
        return baseSpin;
    }


    public Boolean getIsPreciseShot() {
        return isPreciseShot;
    }

    Boolean isPreciseShot;

    public shot(Double liftHeight, Double launcherAngle, Double baseSpeed, Double baseSpin, Boolean isPreciseShot) {
        this.liftHeight = liftHeight;
        this.launcherAngle = launcherAngle;
        this.baseSpeed = baseSpeed;
        this.baseSpin = baseSpin;
        this.isPreciseShot = isPreciseShot;
    }
    
}
