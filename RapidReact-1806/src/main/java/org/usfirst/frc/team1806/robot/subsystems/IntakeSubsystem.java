package org.usfirst.frc.team1806.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeSubsystem implements Subsystem {

    public enum IntakeStates{
        IDLE,
        INTAKING,
        SWEEP
    };
    private DoubleSolenoid mExtendSolenoid;
    private CANSparkMax intakeMotor;
    private Double mIntakeSpeed = 0.0;
    private IntakeStates mIntakeState;
    private Loop mLoop = new Loop(){

        @Override
        public void onStart(double timestamp) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onLoop(double timestamp) {
            switch(mIntakeState){
                case IDLE:
                    mExtendSolenoid.set(Value.kReverse);
                    intakeMotor.set(0.0);
                    return;
                case INTAKING:
                    mExtendSolenoid.set(Value.kForward);
                    intakeMotor.set(Constants.kIntakeSpeed);
                    return;
                case SWEEP:
                    mExtendSolenoid.set(Value.kForward);
                    intakeMotor.set(-Constants.kSweep);
                    return;
            }
            
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }

    };

    public IntakeSubsystem(int canID, int extendSolenoidPort, int retractSolenoidPort){
        mExtendSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, extendSolenoidPort, retractSolenoidPort);
        intakeMotor = new CANSparkMax(canID, MotorType.kBrushless);
        mIntakeState = IntakeStates.IDLE;
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
        mIntakeState = IntakeStates.IDLE;
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
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

    public void wantIntaking(){
        mIntakeState = IntakeStates.INTAKING;

    }

    public void wantIdle(){
        mIntakeState = IntakeStates.IDLE;
    }

    public void wantSweep(){
        mIntakeState = IntakeStates.SWEEP;
    }

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }
}
