package org.usfirst.frc.team1806.robot.game;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Shot {

    Double MAX_LEGAL_HEIGHT = 52.0;
    Double MIN_PIVOT_HEIGHT = 32.0;
    Double CORNER_ANGLE_OFFSET = 2.0;
    Double CORNER_TO_PIVOT_DISTANCE = 20.0;
    Double launcherAngle, topSpeed, bottomSpeed;
    Boolean isPreciseShot;
    Boolean isFlipped;

    
    public static Shot CLOSE_SHOT = new Shot(175.0, 2500.0, 2300.0, false, false);


    public Shot(Double launcherAngle, Double topSpeed, Double bottomSpeed, Boolean isPreciseShot, Boolean isFlipped) {
        this.launcherAngle = launcherAngle;
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
        this.isPreciseShot = isPreciseShot;
        this.isFlipped = isFlipped;
    }

    public Double getLiftHeight() {
        return (MAX_LEGAL_HEIGHT - MIN_PIVOT_HEIGHT) - Math.asin(launcherAngle - 90 + CORNER_ANGLE_OFFSET) * CORNER_TO_PIVOT_DISTANCE;
    }

    public Double getLauncherAngle() {
        return isFlipped? -launcherAngle : launcherAngle;
    }

    public Double getTopSpeed() {
        return isFlipped? bottomSpeed : topSpeed;
    }

    public Double getBottomSpeed() {
        return isFlipped? topSpeed : bottomSpeed;
    }


    public Boolean getIsPreciseShot() {
        return isPreciseShot;
    }

    public Boolean getIsFlipped(){
        return isFlipped;
    }   
}
