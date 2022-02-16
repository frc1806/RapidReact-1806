package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;


public class LaunchBoxAngler implements Subsystem {

    private TalonSRX mLaunchMotor;
    private DutyCycleEncoder mEncoder;
    private PIDController mPIDController;
    private static LaunchBoxAngler LUNCH_BOX_ANGLER = new LaunchBoxAngler();
    private Double mKp, mKi, mKd, mWantedSetPoint;
    private Double angleLeniency = 0.75;
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

    private LaunchBoxAngler(){
        mEncoder = new DutyCycleEncoder(RobotMap.launchBoxAngleEncoder);
        mEncoder.setConnectedFrequencyThreshold(975);
        mEncoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);
        mEncoder.setDistancePerRotation(360);

        mKp = Constants.kLaunchBoxAnglerKp;
        mKi = Constants.kLaunchBoxAnglerKi;
        mKd = Constants.kLaunchBoxAnglerKd;
        mLaunchMotor = new TalonSRX(RobotMap.launchBoxAngler);
        mPIDController = new PIDController(mKp, mKi, mKd);
        
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
        mLunchboxStates = LunchboxStates.Idle;
        
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
        mWantedSetPoint = Angle;
        mLunchboxStates = LunchboxStates.GoingToPosition;
    }
    
    public Boolean isAtAngle(){
        //TODO
        return false;
    }

    public double getCurrentAngle(){
        return mEncoder.getDistance();
    }

    public Boolean angleToCheck(Double angle){
        if (angle >= mWantedSetPoint + angleLeniency && angle >= mWantedSetPoint - angleLeniency) return false;
        return true;
    }

    public static LaunchBoxAngler getInstance(){
        return LUNCH_BOX_ANGLER;
    }

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }
}
