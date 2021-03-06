package org.usfirst.frc.team1806.robot.subsystems;

import java.lang.annotation.Target;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.game.Shot;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;
import org.usfirst.frc.team1806.robot.util.TargetInfo;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.net.PortForwarder;

public class VisionSubsystem implements Subsystem {

    private static VisionSubsystem VISION_SUBSYSTEM = new VisionSubsystem();

    private double FRONT_X_OFFSET = 4.0;
    private double FRONT_Y_OFFSET = 10.0;
    private double REAR_X_OFFSET = 4.0;
    private double REAR_Y_OFFSET = 10.0;

    public static VisionSubsystem getInstance(){
        return VISION_SUBSYSTEM;
    }

    enum TargetLocation{
        kFront,
        kBack,
        kUnknown
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
        Robot.getMainDriverTab().addCamera("Driver Front Camera", "frontDriverCam", "http://10.18.6.13:1182/stream.mjpg").withPosition(1,2).withSize(4,4);
        Robot.getMainDriverTab().addCamera("Driver Rear Camera", "rearDriverCam", "http://10.18.6.13:1184/stream.mjpg").withPosition(3, 2).withSize(4,4); 
    }

    public TargetLocation getBestTargetLocation(){
        if(frontCamera.getLatestResult().hasTargets()){
            return TargetLocation.kFront;
        }
        else if (backCamera.getLatestResult().hasTargets()){
            return TargetLocation.kBack;
        }
        else{
            return TargetLocation.kUnknown;
        }
    }

    private double getFrontDistanceToTarget(){
        //TODO: Correct for camera placement
        PhotonPipelineResult result = frontCamera.getLatestResult();
        if(result.hasTargets()) return getDistanceToTargetForResult(result);
        return -1;
    }

    private double getFrontAngleToTarget(){
        //TODO: Correct for camera placement
        PhotonPipelineResult result = frontCamera.getLatestResult();
        if(result.hasTargets()) return result.getBestTarget().getYaw();
        return Double.MAX_VALUE;
    }

    private double getRearDistanceToTarget(){
        //TODO: Correct for camera placement
        PhotonPipelineResult result = backCamera.getLatestResult();
        if(result.hasTargets()) return getDistanceToTargetForResult(result);
        return -1;
    }

    private double getRearAngleTotarget(){
        //TODO: Correct for camera placement
        PhotonPipelineResult result = backCamera.getLatestResult();
        if(result.hasTargets()) return result.getBestTarget().getYaw();
        return Double.MAX_VALUE;
    }

    private double getDistanceToTargetForResult(PhotonPipelineResult result){
        return Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(Constants.kCameraHeightMeters, Constants.kTargetHeightMeters, Constants.kCameraPitchRadians, Units.degreesToRadians(result.getBestTarget().getPitch())));
    }

    /**
     * Get a shot object from vision
     * @return  a shot object representing the optimal shot for the current distance from testing.
     * If there is no target, will return {@code null}
     */
    public Shot getShotFromVision(){
        switch(getBestTargetLocation()){
            case kBack:
                return Shot.createShotFromVision(getRearDistanceToTarget(), true);
            case kFront:
                return Shot.createShotFromVision(getFrontDistanceToTarget(), false);
            default:
            case kUnknown:
                return null;
            
        }
    }

    /**
     * Get a double representing the angle offset to the target in degrees.
     * @return   {@link double} representing the angle offset to the target in degrees.
     * If there is no target, will retur {@code Double.MAX_VALUE}.
     */
    public double getAngleOffsetToTarget(){
        switch(getBestTargetLocation()){
            case kBack:
                return applyCameraOffset(getRearAngleTotarget(), getRearDistanceToTarget(), REAR_X_OFFSET, REAR_Y_OFFSET).getAngleToTarget();
            case kFront:
                return applyCameraOffset(getFrontAngleToTarget(), getFrontDistanceToTarget(), FRONT_X_OFFSET, FRONT_Y_OFFSET).getAngleToTarget();
            default:
            case kUnknown:
                return Double.MAX_VALUE;
        }
    }

    public TargetInfo applyCameraOffset(Double cameraToTargetAngle, Double cameraToTargetDistance, Double cameraOffsetX, Double cameraOffsetY){
        Double X;
        Double Y;
        Double angle;

        X = cameraOffsetX + (Math.cos(cameraToTargetAngle) * cameraToTargetDistance);
        Y = cameraOffsetY + (Math.sin(cameraToTargetAngle) * cameraToTargetDistance);

        TargetInfo target = new TargetInfo(X, Y);

        angle = Math.atan(Y/X);

        return target;
    }
}