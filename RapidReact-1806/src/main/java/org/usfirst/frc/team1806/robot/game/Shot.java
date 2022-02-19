package org.usfirst.frc.team1806.robot.game;

import org.usfirst.frc.team1806.robot.util.InterpolatingDouble;
import org.usfirst.frc.team1806.robot.util.InterpolatingTreeMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Shot {

    Double MAX_LEGAL_HEIGHT = 52.0;
    Double MIN_PIVOT_HEIGHT = 32.0;
    Double CORNER_ANGLE_OFFSET = 2.0;
    Double CORNER_TO_PIVOT_DISTANCE = 20.0;
    Double launcherAngle, topSpeed, bottomSpeed;
    Boolean isPreciseShot;
    Boolean isFlipped;

    //A tab on the dashboard where we can enter in any shot params we want for tuning... or for extreme in-match emergencies.
    public static class ShotDashboard {

        private static ShuffleboardTab ShotTuningTab = Shuffleboard.getTab("Shot Tuning");
        private static NetworkTableEntry TopSpeedEntry = ShotTuningTab.addPersistent("Top Speed", 1500).withWidget(BuiltInWidgets.kField).getEntry();
        private static NetworkTableEntry BottomSpeedEntry = ShotTuningTab.addPersistent("Bottom Speed", 1500).withWidget(BuiltInWidgets.kField).getEntry();
        private static NetworkTableEntry Angle = ShotTuningTab.addPersistent("Angle", 175).withWidget(BuiltInWidgets.kField).getEntry();
        private static NetworkTableEntry IsPreciseShot = ShotTuningTab.addPersistent("Is Precise Shot?", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        private static NetworkTableEntry IsFlipped = ShotTuningTab.addPersistent("Flipped?", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        
        public static Shot getDashboardShot(){
            return new Shot(Angle.getDouble(85), TopSpeedEntry.getDouble(1500), BottomSpeedEntry.getDouble(1500), IsPreciseShot.getBoolean(false), IsFlipped.getBoolean(false));
        }
    }

    //A map from inches to top wheel speed
    private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> TOP_SPEED_FROM_DISTANCE_MAP = new InterpolatingTreeMap<>(30);
    static{
        TOP_SPEED_FROM_DISTANCE_MAP.put(new InterpolatingDouble(30.0), new InterpolatingDouble(1500.0)); //untested default
    }

    //A map from inches to bottom wheel speed
    private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> BOTTOM_SPEED_FROM_DISTANCE_MAP = new InterpolatingTreeMap<>(30);
    static{
            BOTTOM_SPEED_FROM_DISTANCE_MAP.put(new InterpolatingDouble(30.0), new InterpolatingDouble(1500.0)); //untested default
     }

    //A map from inches to launcher angle
    private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> LAUNCHER_ANGLE_FROM_DISTANCE_MAP = new InterpolatingTreeMap<>(30);
    static{
        LAUNCHER_ANGLE_FROM_DISTANCE_MAP.put(new InterpolatingDouble(30.0), new InterpolatingDouble(175.0)); //untested default
    }

    /**
     * Create a shot from vision
     * @param visionDistance    Distance to target, inches
     * @param isFlipped         is this shot out the back of the robot?
     */
    public static Shot createShotFromVision(Double visionDistance, Boolean isFlipped){
        InterpolatingDouble interpolableVisionDistance = new InterpolatingDouble(visionDistance);
        return new Shot(LAUNCHER_ANGLE_FROM_DISTANCE_MAP.getInterpolated(interpolableVisionDistance).value, TOP_SPEED_FROM_DISTANCE_MAP.getInterpolated(interpolableVisionDistance).value, BOTTOM_SPEED_FROM_DISTANCE_MAP.getInterpolated(interpolableVisionDistance).value, false, isFlipped);
    }
    
    public static Shot CLOSE_SHOT = new Shot(175.0, 2500.0, 2300.0, false, false);


    /**
     * Declare a preset/ dead reckoning shot
     * @param launcherAngle Angle of the shot from 0. Should always be positive. Use the isFlipped param for shooting out the back
     * @param topSpeed The speed of the top flywheel in RPM
     * @param bottomSpeed The speed of the bottom flywheel in RPM
     * @param isPreciseShot is this shot precise
     * @param isFlipped is this shot out the back of the robot?
     */
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
