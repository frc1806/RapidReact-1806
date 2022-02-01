package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;


public class DualRollerSubsystem implements Subsystem{

    TalonSRX rollerOne;
    TalonSRX rollerTwo;
      
    public DualRollerSubsystem(){
        rollerOne = new TalonSRX(RobotMap.rollerOne);
        rollerTwo = new TalonSRX(RobotMap.rollerTwo);
        rollerTwo.setInverted(true);
    }

    public void startRoller() {

        rollerOne.set(ControlMode.PercentOutput, 1.00);
        rollerTwo.set(ControlMode.PercentOutput, 1.00);
    }


    public void stopRoller() {

        rollerOne.set(ControlMode.PercentOutput, 0);
        rollerTwo.set(ControlMode.PercentOutput, 0);

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
        // TODO Auto-generated method stub
        
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


    
}