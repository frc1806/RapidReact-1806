package org.usfirst.frc.team1806.robot.game;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Shot {

    public static ShuffleboardTab shotTab = Shuffleboard.getTab("Shot Adjuster");

    private static Map<String, Object> SPEED_SELECT_PROPS = new HashMap<>();

    static {
        SPEED_SELECT_PROPS.put("Min", 0.0d);
        SPEED_SELECT_PROPS.put("Max", 3000.0d);
    }

    private static NetworkTableEntry customTopRollerSpeedEntry = shotTab
        .addPersistent("Custom Top Roller Speed", 0).withWidget(BuiltInWidgets.kDial).withProperties(SPEED_SELECT_PROPS)
        .withSize(2,1).withPosition(0,0).getEntry();
    private static NetworkTableEntry customBottomRollerEntry = shotTab
        .addPersistent("Custom Bottom Roller Speed", 0).withWidget(BuiltInWidgets.kDial).withProperties(SPEED_SELECT_PROPS)
        .withSize(2,1).withPosition(2,0).getEntry();
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
