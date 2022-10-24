package org.usfirst.frc.team1806.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorSubsystem implements Subsystem {

    private static ColorSensorSubsystem mColorSensorSubsystem;

    private final ColorSensorV3 mColorSensor;
    private final I2C.Port i2cPort;
    
    public enum ColorChoices{
        RED, BLUE, OTHER, NONE
    }

    private ColorChoices mMatchedColor;
    private ColorChoices mAllianceColor;
    private RawColor raw_color;
    private int color_offset;
    private int adjusted_blue;
    private int adjusted_red;
    private double color_ratio;

    public ColorSensorSubsystem(){
        mColorSensorSubsystem = new ColorSensorSubsystem();
        i2cPort = I2C.Port.kOnboard;
        mColorSensor = new ColorSensorV3(i2cPort);
        
        mMatchedColor = ColorChoices.NONE;
        raw_color = mColorSensor.getRawColor();
        adjusted_blue = raw_color.blue;
        adjusted_red = raw_color.red + color_offset;
        color_offset = adjusted_blue - adjusted_red;
        color_ratio = (double) adjusted_red / (double) adjusted_blue;
        updateAllianceColor();
    }

    public static ColorSensorSubsystem getInstance(){
        return mColorSensorSubsystem;
    }

    public boolean hasCorrectColor(){
        return mAllianceColor == mMatchedColor;
    }

    public boolean hasOppositeColor() {
        return !hasCorrectColor()
                    && (mMatchedColor != ColorChoices.OTHER)
                    && (mMatchedColor != ColorChoices.NONE);
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

    public void updateMatchedColor(){
        if (color_ratio > 1.0) {
            mMatchedColor = ColorChoices.RED;
        } else if (color_ratio < 1.0) {
            mMatchedColor = ColorChoices.BLUE;
        } else {
            mMatchedColor = ColorChoices.OTHER;
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
        SmartDashboard.putString("Guess Color", mMatchedColor.toString());
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
