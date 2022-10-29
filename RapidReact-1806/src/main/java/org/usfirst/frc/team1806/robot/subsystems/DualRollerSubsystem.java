package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;


public class DualRollerSubsystem implements Subsystem{

    private TalonSRX frontRoller;
    private TalonSRX backRoller;
    private static DualRollerSubsystem DUAL_ROLLER_SUBSYSTEM = new DualRollerSubsystem();

    private DualRollerSubsystem(){
        frontRoller = new TalonSRX(RobotMap.frontRoller);
        frontRoller.configVoltageCompSaturation(9);
        frontRoller.enableVoltageCompensation(true);

        backRoller = new TalonSRX(RobotMap.rearRoller);
        backRoller.configVoltageCompSaturation(9);
        backRoller.enableVoltageCompensation(true);
        backRoller.setInverted(true);

        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration(true, 40.0, 50.0, .1);
        frontRoller.configSupplyCurrentLimit(config);
        backRoller.configSupplyCurrentLimit(config);
        frontRoller.configOpenloopRamp(0.2);
        backRoller.configOpenloopRamp(0.2);

    }



    public void startRoller() {

        frontRoller.set(ControlMode.PercentOutput, 1.0);
        backRoller.set(ControlMode.PercentOutput, 1.0);
    }


    public void stopRoller() {

        frontRoller.set(ControlMode.PercentOutput, 0);
        backRoller.set(ControlMode.PercentOutput, 0);

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
        stopRoller();
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

    public void feedForward(){
        frontRoller.set(ControlMode.PercentOutput, -1);
        backRoller.set(ControlMode.PercentOutput, 0.6);
    }

    public void feedBackwards(){
        frontRoller.set(ControlMode.PercentOutput, 0.6);
        backRoller.set(ControlMode.PercentOutput, -1);
    }

    public static DualRollerSubsystem getInstance(){
        return DUAL_ROLLER_SUBSYSTEM;
    }



    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }
    
}
