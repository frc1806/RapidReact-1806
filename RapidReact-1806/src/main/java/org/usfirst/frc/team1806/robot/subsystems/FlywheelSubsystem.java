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
import edu.wpi.first.wpilibj.Encoder;

public class FlywheelSubsystem implements Subsystem {

    private CANSparkMax mFlywheelMotor;
    private Double mKp, mKi, mKd, mKf, mIzone, mWantedSpeed, mks, mkv, mKa;
    private PIDController mFlywheelPIDController;
    private SimpleMotorFeedforward mFeedforwardController;
    private Integer withinLeniency = 50;
    private Encoder mEncoder;


    private Loop mLoop = new Loop(){
        @Override
        public void onStart(double timestamp) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onLoop(double timestamp) {
            switch(mFlywheelStates){
                case kIdle:
                    stop();
                    return;
                case kVelocityControl:
                mFlywheelMotor.setVoltage((Constants.kTopFlywheelKf * mWantedSpeed) + mFlywheelPIDController.calculate(getCurrentRPM(), mWantedSpeed));
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
        mEncoder.setDistancePerPulse((4.0/8192.0) * 60); //8192 CPR encoder, change RPS to RPM
        mEncoder.setReverseDirection(isEncoderInverted);
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
        return true;
        //return !(getCurrentRPM() >= mWantedSpeed + withinLeniency && getCurrentRPM() >= mWantedSpeed - withinLeniency);
    }

    public double getOutputPower(){
        return mFlywheelMotor.getAppliedOutput();
    }

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }
}