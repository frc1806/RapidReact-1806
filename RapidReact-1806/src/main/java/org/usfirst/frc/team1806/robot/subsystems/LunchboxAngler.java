package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;


public class LunchboxAngler implements Subsystem {

    private TalonSRX mLaunchMotor;
    private DutyCycleEncoder mEncoder;
    private PIDController mPIDController;
    private static LunchboxAngler LUNCH_BOX_ANGLER;
    private Double mKp, mKi, mKd, mWantedSetPoint;
    private enum LunchboxStates{
        Idle,
        GoingToPosition,
        AtPosition
    }
    private LunchboxStates mLunchboxStates;

    private Loop mLoop = new Loop(){

        @Override
        public void onStart(double timestamp) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onLoop(double timestamp) {
            switch(mLunchboxStates){
                case Idle:
                    mLaunchMotor.set(ControlMode.PercentOutput, 0.0);
                    return;
                case GoingToPosition:
                    mLaunchMotor.set(ControlMode.PercentOutput, mPIDController.calculate(mEncoder.getDistance(), mWantedSetPoint));
                    return;
                case AtPosition:
                    return;
            }
            
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }
        
    };

    public LunchboxAngler(TalonSRX LaunchMotor, Double kp, Double ki, Double kd){
        mEncoder.setConnectedFrequencyThreshold(975);
        mEncoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);
        mEncoder.setDistancePerRotation(360);

        mKp = kp;
        mKi = ki;
        mKd = kd;
        mLaunchMotor = LaunchMotor;
        mPIDController.setPID(mKp, mKi, mKd);;
        
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
        mLaunchMotor.set(ControlMode.PercentOutput, 0.0);
        
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

    public void goToAngle(Double Angle){
        
    }
    
    public Boolean isAtAngle(){
        //TODO
        return false;
    }

    public static LunchboxAngler getInstance(){
        return LUNCH_BOX_ANGLER;
    }

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }
}
