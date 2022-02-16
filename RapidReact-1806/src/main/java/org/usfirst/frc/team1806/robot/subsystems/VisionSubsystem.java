package org.usfirst.frc.team1806.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;
import org.usfirst.frc.team1806.robot.util.LED.CompositeLEDPattern;
import org.usfirst.frc.team1806.robot.util.LED.LEDPattern;
import org.usfirst.frc.team1806.robot.util.LED.LEDPatternSegment;
import org.usfirst.frc.team1806.robot.util.LED.ScrollingLEDPattern;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class VisionSubsystem implements Subsystem {

    private static VisionSubsystem VISION_SUBSYSTEM = new VisionSubsystem();

    public static VisionSubsystem getInstance(){
        return VISION_SUBSYSTEM;
    }
    private PhotonCamera frontCamera = new PhotonCamera("frontGoalPhoton");
    private PhotonCamera backCamera = new PhotonCamera("backGoalPhoton");
    private PhotonCamera frontDriverCam = new PhotonCamera("frontDriverCam");
    private PhotonCamera backDriverCam = new PhotonCamera("backDriverCamm");



    private Loop mLoop = new Loop(){

        @Override
        public void onStart(double timestamp) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onLoop(double timestamp) {

            
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }
        
    };

    private VisionSubsystem(){

        PortForwarder.add(5800, "frontGoalPhoton.local", 5800);
        PortForwarder.add(5800, "rearGoalPhoton.local", 5800);
    }

    @Override
    public void writeToLog() {
        // TODO Auto-generated method stub
    }

    @Override
    public void outputToSmartDashboard() {
        // TODO Auto-generated method stub
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub

    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    @Override
    public void setDebug(boolean _debug) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void retractAll() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        Robot.getMainDriverTab().addCamera("Driver Front Camera", "frontDriverCam", "frontGoalPhoton.local:1183").withPosition(4,1).withSize(4,2);
        Robot.getMainDriverTab().addCamera("Driver Rear Camera", "rearDriverCam", "frontGoalPhoton.local:1185").withPosition(4, 3).withSize(4,2); 
    }

    public boolean isBestTargetInFront(){
        return true;
    }

    private double getFrontDistanceToTarget(){
        PhotonPipelineResult result = frontCamera.getLatestResult();
        if(result.hasTargets()) return getDistanceToTargetForResult(result);
        return -1;
    }

    private double getFrontAngleToTarget(){
        PhotonPipelineResult result = frontCamera.getLatestResult();
        if(result.hasTargets()) return result.getBestTarget().getYaw();
        return Double.MAX_VALUE;
    }

    private double getRearDistanceToTarget(){
        PhotonPipelineResult result = backCamera.getLatestResult();
        if(result.hasTargets()) return getDistanceToTargetForResult(result);
        return -1;
    }

    private double getRearAngleTotarget(){
        PhotonPipelineResult result = backCamera.getLatestResult();
        if(result.hasTargets()) return result.getBestTarget().getYaw();
        return Double.MAX_VALUE;
    }

    private double getDistanceToTargetForResult(PhotonPipelineResult result){
        return Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(Constants.kCameraHeightMeters, Constants.kTargetHeightMeters, Constants.kCameraPitchRadians, Units.degreesToRadians(result.getBestTarget().getPitch())));
    }



}
