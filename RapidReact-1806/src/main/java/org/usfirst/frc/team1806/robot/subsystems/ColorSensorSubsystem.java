package org.usfirst.frc.team1806.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorSubsystem implements Subsystem {

    private ColorSensorSubsystem mColorSensorSubsystem;

    private final ColorSensorV3 mColorSensor;
    private final I2C.Port i2cPort;
    
    public enum ColorChoices{
        RED, BLUE, OTHER, NONE
    }

    private ColorChoices mMatchedColor;
    private ColorChoices mAllianceColor;

    public ColorSensorSubsystem(){
        mColorSensorSubsystem = new ColorSensorSubsystem();
        i2cPort = I2C.Port.kOnboard;
        mColorSensor = new ColorSensorV3(i2cPort);
        
        mMatchedColor = ColorChoices.NONE;
        
    }

    public ColorSensorSubsystem getInstance(){
        return mColorSensorSubsystem;
    }

    public boolean hasCorrectColor(){
        return mAllianceColor == mMatchedColor;
    }

    public boolean hasOppisiteColor(){
        return !hasCorrectColor();
    }

    public void updateAllianceColor() {
        if (DriverStation.isDSAttached()) {
            if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Red) {
                mAllianceColor = ColorChoices.RED;
            } else if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Blue){
                mAllianceColor = ColorChoices.BLUE;
            }
        } else {
            mAllianceColor = ColorChoices.NONE;
            DriverStation.reportError("No Alliance Color Detected", true);
        }
    }

    @Override
    public void writeToLog() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void outputToSmartDashboard() {
        Color detectedColor = mColorSensor.getColor();
        double IR = mColorSensor.getIR();
    
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
        int proximity = mColorSensor.getProximity();
    
        SmartDashboard.putNumber("Proximity", proximity);
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

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }
    
}
