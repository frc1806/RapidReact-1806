package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class VisionSubsystem implements Subsystem {

    private AddressableLED leds;
    private AddressableLEDBuffer onBuffer;
    private AddressableLEDBuffer offBuffer;

    public VisionSubsystem(){
        leds = new AddressableLED(24);
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
        leds.setData(onBuffer);
    }

    public void turnLEDOff(){
        leds.setData(offBuffer);
    }

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }
}
