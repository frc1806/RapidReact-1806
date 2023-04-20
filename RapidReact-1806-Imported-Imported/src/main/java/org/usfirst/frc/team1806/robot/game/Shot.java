package org.usfirst.frc.team1806.robot.game;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team1806.robot.util.InterpolatingDouble;
import org.usfirst.frc.team1806.robot.util.InterpolatingTreeMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Shot {

    private static ArrayList<PolarCoordinate> POINTS_OF_INTEREST_ON_LAUNCHBOX = new ArrayList<>();

    static{
        //TODO: Actually mesaure these.
        POINTS_OF_INTEREST_ON_LAUNCHBOX.add(new PolarCoordinate(0.0, 12.75)); //FRONT OF BOX
        POINTS_OF_INTEREST_ON_LAUNCHBOX.add(new PolarCoordinate(38.10, 16.203)); //FCorner
        POINTS_OF_INTEREST_ON_LAUNCHBOX.add(new PolarCoordinate(67.75, 11.885)); //FCorner
        //POINTS_OF_INTEREST_ON_LAUNCHBOX.add(new PolarCoordinate(4.0, 18.0)); //EXAMPLE FLYWHEEL POINT
    }

    Double MAX_LEGAL_HEIGHT = 52.0;
    Double CORNER_ANGLE_OFFSET = 2.0;
    Double CORNER_TO_PIVOT_DISTANCE = 20.0;
    Double launcherAngle, topSpeed, bottomSpeed;
    Boolean isPreciseShot;
    Boolean isFlipped;
    public static volatile ShotDashboard TheShotDashboard;

    public static void initializeShotDashboard(){
        TheShotDashboard = new ShotDashboard();
    }

    //A tab on the dashboard where we can enter in any shot params we want for tuning... or for extreme in-match emergencies.
    public static class ShotDashboard {

        private static Map<String, Object> FLYWHEEL_SPEED_SLIDER_PROPS;
        private static Map<String, Object> ANGLE_SLIDER_PROPS;

        private ShuffleboardTab ShotTuningTab;
        private GenericEntry TopSpeedEntry;
        private GenericEntry BottomSpeedEntry;
        private GenericEntry Angle;
        private GenericEntry IsPreciseShot;
        private GenericEntry IsFlipped;

        public void initialize(){
            FLYWHEEL_SPEED_SLIDER_PROPS = new HashMap<>();
            FLYWHEEL_SPEED_SLIDER_PROPS.put("Min", 0.0d);
            FLYWHEEL_SPEED_SLIDER_PROPS.put("Max", 6000.0d);
            FLYWHEEL_SPEED_SLIDER_PROPS.put("Block increment", 25d);

            ANGLE_SLIDER_PROPS = new HashMap<>();
            ANGLE_SLIDER_PROPS.put("Min", -200.0d);
            ANGLE_SLIDER_PROPS.put("Max", 200.0d);
            ANGLE_SLIDER_PROPS.put("Block increment", 0.25d);


        }


        public ShotDashboard(){
            ShotTuningTab = Shuffleboard.getTab("Shot Tuning");
            initialize();
            TopSpeedEntry = ShotTuningTab.addPersistent("Top Speed", 1500).withWidget(BuiltInWidgets.kNumberSlider).withProperties(FLYWHEEL_SPEED_SLIDER_PROPS).withPosition(1, 1).withSize(1,4).getEntry();
            BottomSpeedEntry = ShotTuningTab.addPersistent("Bottom Speed", 1500).withWidget(BuiltInWidgets.kNumberSlider).withProperties(FLYWHEEL_SPEED_SLIDER_PROPS).withPosition(1, 2).withSize(1,4).getEntry();
            Angle =  ShotTuningTab.addPersistent("Angle", 175).withWidget(BuiltInWidgets.kNumberSlider).withProperties(ANGLE_SLIDER_PROPS).withPosition(5,1).withSize(2, 2).getEntry();
            IsPreciseShot = ShotTuningTab.addPersistent("Is Precise Shot?", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(6, 1).withSize(2, 2).getEntry();
            IsFlipped = ShotTuningTab.addPersistent("Flipped?", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(8,1).withSize(2,2).getEntry();
        }
        
        public Shot getDashboardShot(){
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
        return new Shot(LAUNCHER_ANGLE_FROM_DISTANCE_MAP.getInterpolated(interpolableVisionDistance).value, TOP_SPEED_FROM_DISTANCE_MAP.getInterpolated(interpolableVisionDistance).value, BOTTOM_SPEED_FROM_DISTANCE_MAP.getInterpolated(interpolableVisionDistance).value, true, isFlipped);
    }
    
    public static Shot LOW_GOAL = new Shot (140.0, 1200.0, 850.0, false, false);
    public static Shot LOW_GOAL_FLIPPED = LOW_GOAL.getThisShotButFlipped();
    public static Shot CLOSE_SHOT = new Shot(166.0, 2500.0, 2500.0, true, false);
    public static Shot FLIPPED_CLOSE_SHOT = CLOSE_SHOT.getThisShotButFlipped();
    public static Shot TARMAC_EDGE_SHOT = new Shot(150.0, 2600.0, 3200.0, true, false);
    public static Shot TARMAC_EDGE_SHOT_FLIPPED = TARMAC_EDGE_SHOT.getThisShotButFlipped();
    public static Shot AUTO_FAR_TARMAC_EDGE = new Shot (152.0, 3000.0, 3650.0, true, false);
    public static Shot BIG_SHOT = new Shot(145.0, 4437.0, 2172.0, true, false);
    public static Shot BIG_SHOT_FLIPPED = BIG_SHOT.getThisShotButFlipped();
    public static Shot ROLL_SHOT = new Shot(110.0, 4500.0, 2000.0, false, false);
    public static Shot ROLL_SHOT_FLIPPED = ROLL_SHOT.getThisShotButFlipped();

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

    public Shot getThisShotButFlipped(){
        return new Shot(this.getLauncherAngle(), this.getTopSpeed(), this.getBottomSpeed(), this.getIsPreciseShot(), !this.getIsFlipped());
    }

}
