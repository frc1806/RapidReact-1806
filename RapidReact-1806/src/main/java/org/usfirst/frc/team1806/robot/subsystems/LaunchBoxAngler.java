package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LaunchBoxAngler implements Subsystem {

    private TalonSRX mLaunchMotor;
    private DutyCycleEncoder mEncoder;
    private PIDController mPIDController;
    private static LaunchBoxAngler LUNCH_BOX_ANGLER = new LaunchBoxAngler();
    private Double mKp, mKi, mKd, mWantedSetPoint;
    private Double angleLeniency = 0.75;
    private final double ROBOT_OFFSET = Constants.kIsCompBot?-200:-51;
    private double currentOffset;
    private boolean hasOffsetBeenSet;
    private boolean hasOffsetBeenLoopChecked;
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
            if(!Constants.kIsCompBot &&!hasOffsetBeenSet && !hasOffsetBeenLoopChecked && mEncoder.getDistance() >= 180.0)
            {
                currentOffset = currentOffset - 360.0;
                hasOffsetBeenSet = true;
                
            }
            hasOffsetBeenLoopChecked = true;
        }

        @Override
        public void onLoop(double timestamp) {
            switch(mLunchboxStates){
                case Idle:
                    mLaunchMotor.set(ControlMode.PercentOutput, 0.0);
                    return;
                case GoingToPosition:
                    double power = mPIDController.calculate(getCurrentAngle(), mWantedSetPoint);
                    if(Math.abs(getCurrentAngle()) < 160 && Math.abs(getCurrentAngle()) >80){
                        power  += getCurrentAngle()>0? 0.10 : -0.10;
                    }  
                    power = MathUtil.clamp(power, -0.45, 0.45);
                    if(mWantedSetPoint == 0.0)
                    {   
                        if(Math.abs(getCurrentAngle()) < 30){
                            power =MathUtil.clamp(power, -0.25, 0.25);
                        } 
                    }

                    mLaunchMotor.set(ControlMode.PercentOutput, power);
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
        currentOffset = 0.0;
        currentOffset += ROBOT_OFFSET;
        hasOffsetBeenLoopChecked=false;
        if(!Constants.kIsCompBot && mEncoder.getDistance() >= 180.0)
        {
            currentOffset = currentOffset- 360.0;
            hasOffsetBeenSet = true;
        }
        else
        {
            hasOffsetBeenSet = false;
        }

        mKp = Constants.kLaunchBoxAnglerKp;
        mKi = Constants.kLaunchBoxAnglerKi;
        mKd = Constants.kLaunchBoxAnglerKd;
        mLaunchMotor = new TalonSRX(RobotMap.launchBoxAngler);
        mLaunchMotor.setNeutralMode(NeutralMode.Brake);
        mPIDController = new PIDController(mKp, mKi, mKd);
        mLunchboxStates = LunchboxStates.Idle;
        mWantedSetPoint = 0.0;
   }

    @Override
    public void writeToLog() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void outputToSmartDashboard() {
        // TODO Auto-generated method stub
        SmartDashboard.putString("Launchbox State", mLunchboxStates.name());
        SmartDashboard.putNumber("Launchbox Encoder Angle", mEncoder.getDistance());
        SmartDashboard.putNumber("Launchbox Wanted Angle", mWantedSetPoint);
        SmartDashboard.putNumber("Launchbox Angle Motor Output", mLaunchMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Launchbox Angle", getCurrentAngle());
        SmartDashboard.putBoolean("hasOffsetBeenSet", hasOffsetBeenSet);
        SmartDashboard.putBoolean("hasOffsetBeenLoopChecked", hasOffsetBeenLoopChecked);
        
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

    public void goToAngle(Double Angle){
        mWantedSetPoint = Angle;
        mLunchboxStates = LunchboxStates.GoingToPosition;
    }
    
    public Boolean isAtAngle(){
        return checkIfAtArbitraryAngle(mWantedSetPoint);
    }

    public double getCurrentAngle(){
        return mEncoder.getDistance() + currentOffset;
    }

    public Boolean checkIfAtArbitraryAngle(Double angle){
        return true;
        //if (Math.abs(angle - getCurrentAngle()) > angleLeniency) return false;
        //return true;
    }

    public static LaunchBoxAngler getInstance(){
        return LUNCH_BOX_ANGLER;
    }

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }
}
