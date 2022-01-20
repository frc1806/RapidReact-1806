package org.usfirst.frc.team1806.robot.subsystems;

import com.revrobotics.CANSparkMax;

import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;

public class IntakeSubsystem implements Subsystem {

    public enum IntakeStates{
        IDLE,
        INTAKING,
        SWEEP
    };
    private DoubleSolenoid extendSolenoid;
    private CANSparkMax intakeMotor;
    private Double mIntakeSpeed = 0.0;

    public IntakeSubsystem(Double intakeSpeed){
        mIntakeSpeed = intakeSpeed;
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
        intakeMotor.set(0.0);
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
    
}
