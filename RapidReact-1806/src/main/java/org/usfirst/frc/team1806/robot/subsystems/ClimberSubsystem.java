package org.usfirst.frc.team1806.robot.subsystems;

import java.security.Timestamp;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ClimberSubsystem implements Subsystem {


    private TalonSRX climbMotor1;
    private TalonSRX climbMotor2;
    private TalonSRX climbMotor3;
    private TalonSRX climbMotor4;
    private DoubleSolenoid leftClaw;
    private DoubleSolenoid rightClaw;
    private DigitalInput leftClawLimitSwitch;
    private DigitalInput rightClawLimitSwitch;
    private boolean wasLeftLimitSwitchPressed;
    private boolean wasRightLimitSwitchPressed;
    private boolean isLeftLimitSwitchOn;
    private boolean isRightLimitSwitchOn;
    private int releaseCounter;
    private double leftClawReleaseTime;
    private double rightClawReleaseTime;
    private double climbPower;


    private ClimberSubsystem(){
        
        climbMotor1 = new TalonSRX(RobotMap.climbMotor1);
        climbMotor2 = new TalonSRX(RobotMap.climbMotor2);
        climbMotor3 = new TalonSRX(RobotMap.climbMotor3);
        climbMotor4 = new TalonSRX(RobotMap.climbMotor4);
        climbMotor2.follow(climbMotor1);
        climbMotor3.follow(climbMotor1);
        climbMotor4.follow(climbMotor1);
        leftClaw = new DoubleSolenoid(RobotMap.module2Number, PneumaticsModuleType.CTREPCM, RobotMap.gripLeftClaw, RobotMap.releaseLeftClaw);
        rightClaw = new DoubleSolenoid(RobotMap.module2Number, PneumaticsModuleType.CTREPCM, RobotMap.gripRightClaw, RobotMap.releaseRightClaw);
        leftClawLimitSwitch = new DigitalInput(RobotMap.leftClawLimitSwitch);
        rightClawLimitSwitch = new DigitalInput(RobotMap.rightClawLimitSwitch);
        isLeftLimitSwitchOn = true;
        isRightLimitSwitchOn = true;
        wasLeftLimitSwitchPressed = true;
        wasRightLimitSwitchPressed = true;
        releaseCounter = 0;
        leftClawReleaseTime = -235;
        rightClawReleaseTime = -235;
        climbPower = 0;
    }


    private Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onLoop(double timestamp) {
            climbMotor1.set(ControlMode.PercentOutput, climbPower);
            isLeftLimitSwitchOn = leftClawLimitSwitch.get();
            isRightLimitSwitchOn = rightClawLimitSwitch.get();
            if(isLeftLimitSwitchOn && !wasLeftLimitSwitchPressed){
                rightClawReleaseTime = timestamp;
                releaseCounter ++;
            }

            if(isRightLimitSwitchOn && !wasRightLimitSwitchPressed){
                leftClawReleaseTime = timestamp;
                releaseCounter ++;
            }

            if(timestamp - leftClawReleaseTime < 0.25 && releaseCounter > 1){
                leftClaw.set(Value.kReverse);
            }

            else {
                leftClaw.set(Value.kForward);
            }


            
            if(timestamp - rightClawReleaseTime < 0.25 && releaseCounter > 1){
                rightClaw.set(Value.kReverse);
            }

            else {
                rightClaw.set(Value.kForward);
            }
        
            wasLeftLimitSwitchPressed = isLeftLimitSwitchOn;
            wasRightLimitSwitchPressed = isRightLimitSwitchOn;
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }
        
    };

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
        climbMotor1.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void zeroSensors() {
        releaseCounter = 0;
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

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }

    public void moveClimber(double power){
        climbPower = power;
    }

    
}