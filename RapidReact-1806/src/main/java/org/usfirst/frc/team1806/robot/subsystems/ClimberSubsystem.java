package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ClimberSubsystem implements Subsystem {


    private TalonSRX bottomMotor1;
    private TalonSRX bottomMotor2;
    private TalonSRX topMotor1;
    private TalonSRX topMotor2;
    private DoubleSolenoid leftClaw;
    private DoubleSolenoid rightClaw;


    private ClimberSubsystem(){
        bottomMotor1 = new TalonSRX(RobotMap.bottomMotor1);
        bottomMotor2 = new TalonSRX(RobotMap.bottomMotor2);
        topMotor1 = new TalonSRX(RobotMap.topMotor1);
        topMotor2 = new TalonSRX(RobotMap.topMotor2);
        leftClaw = new DoubleSolenoid(RobotMap.module2Number, PneumaticsModuleType.CTREPCM, RobotMap.gripLeftClaw, RobotMap.releaseLeftClaw);
        rightClaw = new DoubleSolenoid(RobotMap.module2Number, PneumaticsModuleType.CTREPCM, RobotMap.gripRightClaw, RobotMap.releaseRightClaw);
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

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }

    
}