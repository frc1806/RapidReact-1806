package org.usfirst.frc.team1806.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

public class FlywheelSubsystem implements Subsystem {
    
    private CANSparkMax mFlywheelMotor;
    private Double mKp, mKi, mKd, mKf, mIzone, mConversionFactor, mWantedSpeed;
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

    public FlywheelSubsystem(Integer canID, Double kp, Double ki, Double kd, Double kf, Double izone, Boolean isInverted, Double conversionFactor){
        mFlywheelMotor = new CANSparkMax(canID, MotorType.kBrushless);
        mKp = kp;
        mKi = ki;
        mKd = kd;
        mKf = kf;
        mIzone = izone;
        mConversionFactor = conversionFactor;
        mWantedSpeed = 0.0;
        mFlywheelStates = FlywheelStates.kIdle;

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
        mFlywheelMotor.getPIDController().setReference(0.0, CANSparkMax.ControlType.kDutyCycle);
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
    public void goToHatchMode() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void goToCargoMode() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void retractAll() {
        // TODO Auto-generated method stub
        
    }
    
    public void reloadGames() {
        mFlywheelMotor.getPIDController().setP(mKp);
        mFlywheelMotor.getPIDController().setI(mKi);
        mFlywheelMotor.getPIDController().setD(mKd);
        mFlywheelMotor.getPIDController().setFF(mKf);
        mFlywheelMotor.getPIDController().setIZone(mIzone);
        mFlywheelMotor.getEncoder().setVelocityConversionFactor(mConversionFactor);
    }

    public void setWantedSpeed(Double speed){
        if(speed == 0.0){
            stop();
            return;
        }
        mWantedSpeed = speed;
        mFlywheelStates = FlywheelStates.kPositionControl;
        mFlywheelMotor.getPIDController().setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    public Double getWantedRPM(){
        return mWantedSpeed;
    }

    public Double getCurrentRPM(){
        return mFlywheelMotor.getEncoder().getVelocity();
    }
}
