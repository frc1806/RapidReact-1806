package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Conveyor implements Subsystem {

    

    private enum ConveyorStates{
        kIdle,
        kConveying,
        kReverse,
        kLoading,
        kLaunch,
        kPrepareForLaunch,
    };

    private ConveyorStates mConveyorStates;

    private Loop mLoop = new Loop(){

        @Override
        public void onStart(double timestamp) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onLoop(double timestamp) {
            switch(mConveyorStates){
                case kIdle:
                    mTalonMotor.set(ControlMode.PercentOutput, 0.0);
                    return;
                case kConveying:
                    return;
                case kReverse:
                    return;
                case kLoading:
                    return;
                case kPrepareForLaunch:
                    return;
            }            
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }

    };
    private TalonSRX mTalonMotor;

    public Conveyor(CANifier canifier){
        canifier = canifier;
        mConveyorStates = ConveyorStates.kIdle;
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
        mConveyorStates = ConveyorStates.kIdle;
        
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

    public void loadConveyor(){
        mConveyorStates = ConveyorStates.kLoading;
    }

    public void startConveyor(Double percent){
        mTalonMotor.set(ControlMode.PercentOutput, percent);
        mConveyorStates = ConveyorStates.kConveying;
    }

    public void prepareForLaunch(){
        mConveyorStates = ConveyorStates.kPrepareForLaunch;
    }

    public void launch(){
        mConveyorStates = ConveyorStates.kLaunch;
    }

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }
    
}