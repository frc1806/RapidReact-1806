package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
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
                case kPositionControl:
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
        kPositionControl
    };

    private FlywheelStates mFlywheelStates;

    public FlywheelSubsystem(Integer canID, Double kp, Double ki, Double kd, Double kf, Double izone, Boolean isInverted, Double ks, Double kv, Double ka,  Integer quadA, Integer quadB){
        mFlywheelMotor = new CANSparkMax(canID, MotorType.kBrushless);
        mFlywheelMotor.setInverted(isInverted);
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
        mEncoder.setDistancePerPulse((1.0/8192.0) * 60); //8192 CPR encoder, change RPS to RPM
        mEncoder.setReverseDirection(isInverted);
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
        mFlywheelStates = FlywheelStates.kPositionControl;
        mFlywheelMotor.setVoltage(mFeedforwardController.calculate(rpmToCounts(mWantedSpeed)) + mFlywheelPIDController.calculate(mEncoder.getRate(), rpmToCounts(mWantedSpeed)));
    }

    public void setReverseSpeed(Double speed){
        if(speed == 0.0){
            mFlywheelStates = FlywheelStates.kIdle;
            return;
        }
        mWantedSpeed = -speed;
        mFlywheelStates = FlywheelStates.kPositionControl;
        mFlywheelMotor.setVoltage(mFeedforwardController.calculate(rpmToCounts(mWantedSpeed)) + mFlywheelPIDController.calculate(mEncoder.getRate(), rpmToCounts(mWantedSpeed)));
    }

    public Double getWantedRPM(){
        return mWantedSpeed;
    }

    public Double getCurrentRPM(){
        return mEncoder.getRate();
    }

    private Double rpmToCounts(Double rpm){
        return rpm * Constants.kRPMToCounts;
    }

    private Double countsToRPM(Double counts){
        return counts / Constants.kRPMToCounts;
    }

    public Boolean isSpeedInRange(){
        return !(getCurrentRPM() >= mWantedSpeed + withinLeniency && getCurrentRPM() >= mWantedSpeed - withinLeniency);
    }

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }
}