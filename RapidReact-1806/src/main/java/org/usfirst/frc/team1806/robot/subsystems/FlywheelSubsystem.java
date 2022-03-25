package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FlywheelSubsystem implements Subsystem {

    private CANSparkMax mFlywheelMotor;
    private Double mKp, mKi, mKd, mKf, mIzone, mWantedSpeed, mks, mkv, mKa;
    private PIDController mFlywheelPIDController;
    private SimpleMotorFeedforward mFeedforwardController;
    private double withinLeniency = 250.0;
    private double withinLeniencyImprecise = 600.0;
    private Encoder mEncoder;
    private int currentLimit;
    private double rampRate;
    private boolean mPreciseShot = true;
    private CircularBuffer averagingBuffer;


    private Loop mLoop = new Loop(){
        @Override
        public void onStart(double timestamp) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onLoop(double timestamp) {
            averagingBuffer.addFirst(getCurrentRPM());
            switch(mFlywheelStates){
                case kIdle:
                    stop();
                    return;
                case kVelocityControl:
                if(mWantedSpeed> 0){
                    if(currentLimit != 30)
                    {
                        currentLimit = 30;
                        mFlywheelMotor.setSmartCurrentLimit(currentLimit);
                    }
                    if(rampRate != 0.05){
                        rampRate = 0.5;
                        mFlywheelMotor.setOpenLoopRampRate(rampRate);
                    }
                    if (mWantedSpeed * 0.66 > getCurrentRPM()){
                        mFlywheelMotor.setVoltage(12.0);
                    }
                    else if(mWantedSpeed > getCurrentRPM()){
                        mFlywheelMotor.setVoltage(mKf * mWantedSpeed * 1.1);
                    }
                    else{
                        mFlywheelMotor.setVoltage(mKf * mWantedSpeed * 0.9);
                    }
                }
                else{
                    if(currentLimit != 25){
                        currentLimit = 25;
                        mFlywheelMotor.setSmartCurrentLimit(currentLimit);
                    }
                    if(rampRate != 1.0){
                        rampRate = 1.0;
                        mFlywheelMotor.setOpenLoopRampRate(1.0);
                    }
                    
                    if(mWantedSpeed < getCurrentRPM()){
                        mFlywheelMotor.setVoltage(mKf * mWantedSpeed * 1.2);
                    }
                    else{
                        mFlywheelMotor.setVoltage(mKf * mWantedSpeed * 0.8);
                    }
                }
                //mFlywheelMotor.setVoltage((mKf * mWantedSpeed) + mFlywheelPIDController.calculate(getCurrentRPM(), mWantedSpeed) );
                    return;
            }
            
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            
        }

    };
    private enum FlywheelStates{
        kIdle,
        kVelocityControl
    };

    private FlywheelStates mFlywheelStates;

    public FlywheelSubsystem(Integer canID, Double kp, Double ki, Double kd, Double kf, Double izone, Boolean isInverted, Double ks, Double kv, Double ka,  Integer quadA, Integer quadB, boolean isEncoderInverted){
        mFlywheelMotor = new CANSparkMax(canID, MotorType.kBrushless);
        mFlywheelMotor.setInverted(isInverted);
        mFlywheelMotor.setIdleMode(IdleMode.kCoast);
        mFlywheelMotor.setSmartCurrentLimit(30);
        mFlywheelMotor.setOpenLoopRampRate(0.0);
        currentLimit = 30;
        mKp = kp;
        mKi = ki;
        mKd = kd;
        mKf = kf;
        mks = ks;
        mkv = kv;
        mKa = ka;
        mIzone = izone;
        mWantedSpeed = 0.0;
        mFlywheelStates = FlywheelStates.kIdle;
        mEncoder = new Encoder(quadA, quadB);
        mEncoder.setDistancePerPulse((1.0/2048.0) * 60); //2048 CPR encoder, change RPS to RPM
        mEncoder.setSamplesToAverage(12);
        mEncoder.setReverseDirection(isEncoderInverted);
        averagingBuffer = new CircularBuffer(30);
        reloadGames();
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
        mWantedSpeed = 0.0;
        mFlywheelStates = FlywheelStates.kIdle;
        mFlywheelMotor.setVoltage(0.0);
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
    
    public void reloadGames() {

        mFlywheelPIDController = new PIDController(mKp, mKi, mKd);
        mFeedforwardController = new SimpleMotorFeedforward(mks, mkv, mKa);
    
    }

    public void setWantedSpeed(Double speed){
        if(speed == 0.0){
            mFlywheelStates = FlywheelStates.kIdle;
            return;
        }
        mWantedSpeed = speed;
        mFlywheelStates = FlywheelStates.kVelocityControl;
        
    }

    public void setReverseSpeed(Double speed){
        if(speed == 0.0){
            mFlywheelStates = FlywheelStates.kIdle;
            return;
        }
        mWantedSpeed = -speed;
        mFlywheelStates = FlywheelStates.kVelocityControl;
    }

    public Double getWantedRPM(){
        return mWantedSpeed;
    }

    public Double getCurrentRPM(){
        return mEncoder.getRate();
    }

    public Boolean isSpeedInRange(){
       return isAtSpeed(mWantedSpeed);
    }

    public Boolean isAtSpeed(double Speed){
        double runningTotal = 0;
        
        for(int i = 0; i < 30; i++){
            runningTotal += averagingBuffer.get(i);
        }
        //SmartDashboard.putNumber("Average", (runningTotal / 30.0));
        return Math.abs((runningTotal / 30.0) - mWantedSpeed) < (mPreciseShot?withinLeniency:withinLeniencyImprecise);
    }

    public double getOutputPower(){
        return mFlywheelMotor.getAppliedOutput();
    }

    public void setPreciseShot(boolean preciseShot){
        
    }

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }
}