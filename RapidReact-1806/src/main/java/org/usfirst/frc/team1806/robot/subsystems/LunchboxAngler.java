package org.usfirst.frc.team1806.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;

import org.usfirst.frc.team1806.robot.loop.Looper;

public class LunchboxAngler implements Subsystem {

    private CANSparkMax mLaunchMotor;
    private SparkMaxAlternateEncoder mEncoder;
    private SparkMaxPIDController mPIDController;
    private static LunchboxAngler LUNCH_BOX_ANGLER;

    public LunchboxAngler(CANSparkMax LaunchMotor){
        mLaunchMotor = LaunchMotor;
        mPIDController = mLaunchMotor.getPIDController();
        mPIDController.setFeedbackDevice(mEncoder);
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
        mLaunchMotor.set(0.0);
        
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
}
