package org.usfirst.frc.team1806.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

import java.util.regex.Pattern;
import java.util.stream.IntStream;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;
import org.usfirst.frc.team1806.robot.util.LED.CompositeLEDPattern;
import org.usfirst.frc.team1806.robot.util.LED.GlitchyLEDPattern;
import org.usfirst.frc.team1806.robot.util.LED.LEDPattern;
import org.usfirst.frc.team1806.robot.util.LED.LEDPatternSegment;
import org.usfirst.frc.team1806.robot.util.LED.ScrollingLEDPattern;

public class LEDStringSubsystem implements Subsystem {

    private AddressableLED mLED;
    private AddressableLEDBuffer mLEDBuffer;
    private int mNumOfLEDs;
    private boolean stopOnDisable;
    private double lastUpdateTime;
    private LEDPattern currentPattern;

    //2022 Specific
    private LEDPattern visionRingsPattern, leftDriveBasePatern, middleDriveBasePattern, rightDriveBasePattern, leftLaunchboxPattern, rightLaunchBoxPattern;

    private static LEDStringSubsystem LED_STRING_SUBSYSTEM = new LEDStringSubsystem(0, 174, false);

    public static LEDStringSubsystem getInstance(){
        return LED_STRING_SUBSYSTEM;
    }

    double lastCheckedDS;
    boolean isDSAttached;
    Alliance currentAlliance;

    private DriveBaseLEDMode driveBaseLEDMode;
    private LaunchBoxLEDMode launchBoxLEDMode;
    private VisionRingLEDMode visionRingLEDMode;
    
    enum VisionRingLEDMode{
        kVision,
        kOff,
        kVisionError
    }
    enum DriveBaseLEDMode{
        kVision,
        kAlliance
    }

    enum LaunchBoxLEDMode{
        kAlliance,
        kMotion
    }

    //LED string constructor is private for 2022
    private LEDStringSubsystem(int port, int mNumOfLEDs, boolean stopOnDisable) {
        lastUpdateTime = 0;
        lastCheckedDS = 0;
        this.mLED = new AddressableLED(port);
        this.mNumOfLEDs = mNumOfLEDs;
        this.mLEDBuffer = new AddressableLEDBuffer(mNumOfLEDs);
        mLED.setLength(mLEDBuffer.getLength());
        mLED.setData(mLEDBuffer);
        currentPattern = ScrollingLEDPattern.BLACK_AND_WHITE;
        StartLED();
        this.stopOnDisable = stopOnDisable;

        //2022 Specific
        isDSAttached = false;
        currentAlliance = Alliance.Invalid;

        driveBaseLEDMode = DriveBaseLEDMode.kAlliance;
        launchBoxLEDMode = LaunchBoxLEDMode.kAlliance;
        visionRingLEDMode = VisionRingLEDMode.kOff;

        //TODO: Replace thse with the methods to update the modes.
        visionRingsPattern = ScrollingLEDPattern.VISION_GREEN;
        leftDriveBasePatern = ScrollingLEDPattern.NO_ALLIANCE;
        middleDriveBasePattern = ScrollingLEDPattern.NO_ALLIANCE;
        rightDriveBasePattern = ScrollingLEDPattern.NO_ALLIANCE;
        leftLaunchboxPattern = new GlitchyLEDPattern(ScrollingLEDPattern.NO_ALLIANCE, 30, 1);
        rightLaunchBoxPattern = new GlitchyLEDPattern(ScrollingLEDPattern.NO_ALLIANCE, 30, 1);//ScrollingLEDPattern.NO_ALLIANCE;
        buildLEDPattern();
    }

    private void buildLEDPattern(){
        this.setPattern(new CompositeLEDPattern(new LEDPatternSegment(24, visionRingsPattern, false), new LEDPatternSegment(30, leftDriveBasePatern, false), new LEDPatternSegment(30, middleDriveBasePattern, true), new LEDPatternSegment(30, rightDriveBasePattern, false), new LEDPatternSegment(30, leftLaunchboxPattern, false), new LEDPatternSegment(30, rightLaunchBoxPattern, true)));
    }

    private Loop mEnabledLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            StartLED();
        }

        @Override
        public void onLoop(double timestamp) {
            if(currentPattern.getFPS() == 0)
            {
                return;
            }
            if(timestamp - lastUpdateTime > 1.0 / currentPattern.getFPS()) {
                currentPattern.updateAnimation();
                IntStream.range(0, mLEDBuffer.getLength()).forEach(i -> mLEDBuffer.setLED(i, currentPattern.getColorForPositionInString(i)));
                mLED.setData(mLEDBuffer);
                lastUpdateTime = timestamp;
            }

            //Check DS every second. Do not do this too often due to joystick deadlock
            if(timestamp - lastCheckedDS > 1.0)
            {
                boolean dsAttach = DriverStation.isDSAttached();
                Alliance alliance = DriverStation.getAlliance();
                if((isDSAttached != dsAttach || currentAlliance != alliance) && driveBaseLEDMode == DriveBaseLEDMode.kAlliance) updateDriveBaseLEDPatternForAlliance(dsAttach, alliance);
                isDSAttached = DriverStation.isDSAttached();
                currentAlliance = DriverStation.getAlliance();
                lastCheckedDS = timestamp;
            }

        }

        @Override
        public void onStop(double timestamp) {
            if(stopOnDisable)
            {
                IntStream.range(0, mLEDBuffer.getLength()).forEach(i -> mLEDBuffer.setLED(i, Color.kBlack));
                mLED.setData(mLEDBuffer);
            }
            StopLED();
        }
    };

    @Override
    public void registerEnabledLoops(Looper mEnabledLooper) {
        mEnabledLooper.register(mEnabledLoop);
    }

    public void registerDisabledLoops(Looper mDisabledLooper)
    {
        mDisabledLooper.register(new Loop(){

            @Override
            public void onStart(double timestamp) {
                // TODO Auto-generated method stub
                if(!stopOnDisable)
                {
                    mEnabledLoop.onStart(timestamp);
                }

                //2022Specific
                visionRingsPattern = ScrollingLEDPattern.LIGHTS_OUT;
            }

            @Override
            public void onLoop(double timestamp) {
                // TODO Auto-generated method stub
                if(!stopOnDisable)
                {
                    mEnabledLoop.onLoop(timestamp);
                }
            }

            @Override
            public void onStop(double timestamp) {
                // TODO Auto-generated method stub
                if(!stopOnDisable)
                {
                    mEnabledLoop.onStop(timestamp);
                }

                //2022 Specific
                visionRingsPattern = ScrollingLEDPattern.VISION_GREEN;
            }
            
        });
    }



    public void StartLED() {
        mLED.start();
    }

    public void StopLED() {
        mLED.stop();
    }

    @Override
    public void stop() {
        mLED.stop();
    }

    public void setPattern(LEDPattern currentPattern) {
        this.currentPattern = currentPattern;
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
    public void zeroSensors() {
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

    private void updateDriveBaseLEDPatternForAlliance(boolean isDSAttached, Alliance alliance){
        if(!isDSAttached)
        {
            leftDriveBasePatern = ScrollingLEDPattern.NO_ALLIANCE;
            middleDriveBasePattern = ScrollingLEDPattern.NO_ALLIANCE;
            rightDriveBasePattern = ScrollingLEDPattern.NO_ALLIANCE;
        }
        else{
            switch(alliance){
                case Blue:
                    leftDriveBasePatern = ScrollingLEDPattern.BLUE_ALLIANCE;
                    middleDriveBasePattern = ScrollingLEDPattern.BLUE_ALLIANCE;
                    rightDriveBasePattern = ScrollingLEDPattern.BLUE_ALLIANCE;
                    break;
                default:
                case Invalid:
                    leftDriveBasePatern = ScrollingLEDPattern.NO_ALLIANCE;
                    middleDriveBasePattern = ScrollingLEDPattern.NO_ALLIANCE;
                    rightDriveBasePattern = ScrollingLEDPattern.NO_ALLIANCE;
                    break;
                case Red:
                    leftDriveBasePatern = ScrollingLEDPattern.RED_ALLIANCE;
                    middleDriveBasePattern = ScrollingLEDPattern.RED_ALLIANCE;
                    rightDriveBasePattern = ScrollingLEDPattern.RED_ALLIANCE;
                    break;
                
            }
        }
        buildLEDPattern();
    }

    private void updateDriveBaseLEDPatternForVision()
    {
        //TODO: Not implemented yet
        buildLEDPattern();
    }

    private void setDriveBaseLEDsToVision(){
        driveBaseLEDMode = DriveBaseLEDMode.kVision;
        updateDriveBaseLEDPatternForVision();
    }

    private void setDriveBaseLEDsToAllianceMode()
    {
        driveBaseLEDMode = DriveBaseLEDMode.kAlliance;
        updateDriveBaseLEDPatternForAlliance(isDSAttached, currentAlliance);
    }
    
}
