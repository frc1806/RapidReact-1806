package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FlywheelSubsystem implements Subsystem {

    private CANSparkMax mFlywheelMotor;
    private Double mKp, mKi, mKd, mKf, mIzone, mConversionFactor, mWantedSpeed, mks, mkv;
    private CANifier mCANifierFlywheelCANifiermFlywheelCANifier;
    private PIDController mFlywheelPIDController;
    private SimpleMotorFeedforward mFeedforwardController;


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

    public FlywheelSubsystem(Integer canID, Double kp, Double ki, Double kd, Double kf, Double izone, Boolean isInverted, Double conversionFactor, Integer CANifierID, Double ks, Double kv){
        mFlywheelMotor = new CANSparkMax(canID, MotorType.kBrushless);
        mKp = kp;
        mKi = ki;
        mKd = kd;
        mKf = kf;
        mks = ks;
        mkv = kv;
        mIzone = izone;
        mConversionFactor = conversionFactor;
        mWantedSpeed = 0.0;
        mFlywheelStates = FlywheelStates.kIdle;
        mCANifierFlywheelCANifiermFlywheelCANifier = new CANifier(CANifierID);

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
        mFlywheelMotor.setVoltage(0.0);
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
    
    public void reloadGames() {
        mFlywheelMotor.getPIDController().setP(mKp);
        mFlywheelMotor.getPIDController().setI(mKi);
        mFlywheelMotor.getPIDController().setD(mKd);
        mFlywheelMotor.getPIDController().setFF(mKf);
        mFlywheelMotor.getPIDController().setIZone(mIzone);

        mFlywheelPIDController = new PIDController(mKp, mKi, mKd);
        mFeedforwardController = new SimpleMotorFeedforward(mks, mkv);
    
    }

    public void setWantedSpeed(Double speed){
        if(speed == 0.0){
            stop();
            return;
        }
        mWantedSpeed = speed;
        mFlywheelStates = FlywheelStates.kPositionControl;
        mFlywheelMotor.setVoltage(mFeedforwardController.calculate(rpmToCounts(mWantedSpeed)) + mFlywheelPIDController.calculate(mCANifierFlywheelCANifiermFlywheelCANifier.getQuadratureVelocity(), rpmToCounts(mWantedSpeed)));
    }

    public Double getWantedRPM(){
        return mWantedSpeed;
    }

    public Double getCurrentRPM(){
        return mFlywheelMotor.getEncoder().getVelocity();
    }

    private Double rpmToCounts(Double rpm){
        return rpm * Constants.kRPMToCounts;
    }

    private Double countsToRPM(Double counts){
        return counts / Constants.kRPMToCounts;
    }
}