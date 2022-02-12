package org.usfirst.frc.team1806.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class VisionSubsystem implements Subsystem {

    private AddressableLED ringLEDs;
    private AddressableLEDBuffer onBuffer;
    private AddressableLEDBuffer offBuffer;
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
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }
        
    };

    public VisionSubsystem(){
        PortForwarder.add(5800, "frontGoalPhoton.local", 5800);
        PortForwarder.add(5800, "rearGoalPhoton.local", 5800);
        ringLEDs = new AddressableLED(24);
        onBuffer = new AddressableLEDBuffer(24);
        for(int i = 0; i < 24; i++)
        {
            onBuffer.setLED(i, Color.kGreen);
        }
        offBuffer = new AddressableLEDBuffer(24);
        for(int i = 0; i < 24; i++)
        {
            offBuffer.setLED(i, Color.kBlack);
        }
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
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setDebug(boolean _debug) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void retractAll() {
        // TODO Auto-generated method stub
        
    }
    
    public void turnLEDOn(){
        ringLEDs.setData(onBuffer);
    }

    public void turnLEDOff(){
        ringLEDs.setData(offBuffer);
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


}
