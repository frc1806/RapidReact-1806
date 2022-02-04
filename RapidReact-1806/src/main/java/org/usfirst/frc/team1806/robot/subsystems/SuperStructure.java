package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.CANifier;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.game.shot;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

public class SuperStructure implements Subsystem {

    public enum SuperStructureStates{
        IntakingFront,
        IntakingBack,
        PrepareLaunch,
        Launching,
        Idle,
        Climbing,
    };

    private Loop mLoop = new Loop(){

        @Override
        public void onStart(double timestamp) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onLoop(double timestamp) {
            switch(mSuperStructureStates){
                case IntakingFront:
                    return;
                case IntakingBack:
                    return;
                case PrepareLaunch:
                    return;
                case Launching:
                    return;
                default:
                case Idle:
                    return;
                case Climbing:
                    return;
            }
            
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }
        
    };

    private static SuperStructure SUPER_STRUCTURE = new SuperStructure();
    private IntakeSubsystem mFrontIntake;
    private IntakeSubsystem mBackIntake;
    private FlywheelSubsystem mUpFlywheel;
    private FlywheelSubsystem mDownFlywheel;
    private ElevatorSubsystem mElevator;
    private LunchboxAngler mLunchboxAngler;
    private SuperStructureStates mSuperStructureStates;
    private CANifier mCanifierUp = new CANifier(0);
    private CANifier mCanitiferDown = new CANifier(1);

    private SuperStructure(){
        mFrontIntake = new IntakeSubsystem(RobotMap.frontIntake, RobotMap.frontIntakeExtend, RobotMap.backIntakeExtend);
        mBackIntake = new IntakeSubsystem(RobotMap.rearIntake, RobotMap.backIntakeExtend, RobotMap.backIntakeRetract);
        mUpFlywheel = new FlywheelSubsystem(RobotMap.upFlywheel, Constants.kTopFlywheelKp, Constants.kTopFlywheelKi, Constants.kTopFlywheelKd, Constants.kTopFlywheelKf, Constants.kTopFlywheelIzone, false, Constants.kTopFlywheelConversionFactor, RobotMap.upFlywheel, Constants.kTopFlywheelKs, Constants.kTopFlywheelKv, mCanifierUp);
        mDownFlywheel =  new FlywheelSubsystem(RobotMap.downFlywheel, Constants.kTopFlywheelKp, Constants.kTopFlywheelKi, Constants.kTopFlywheelKd, Constants.kTopFlywheelKf, Constants.kTopFlywheelIzone, false, Constants.kTopFlywheelConversionFactor, RobotMap.upFlywheel, Constants.kTopFlywheelKs, Constants.kTopFlywheelKv, mCanitiferDown);
    }

    @Override
    public void writeToLog() {
        mBackIntake.writeToLog();
        mFrontIntake.writeToLog();
        mDownFlywheel.writeToLog();
        mUpFlywheel.writeToLog();
        mElevator.writeToLog();
        
    }

    @Override
    public void outputToSmartDashboard() {
        mBackIntake.outputToSmartDashboard();
        mFrontIntake.outputToSmartDashboard();
        mDownFlywheel.outputToSmartDashboard();
        mUpFlywheel.outputToSmartDashboard();
        mElevator.outputToSmartDashboard();
        
    }

    @Override
    public void stop() {
        mSuperStructureStates = SuperStructureStates.Idle;
        mFrontIntake.stop();
        mBackIntake.stop();
        mUpFlywheel.stop();
        mDownFlywheel.stop();
        mElevator.stop();
        
    }

    @Override
    public void zeroSensors() {
        mBackIntake.zeroSensors();
        mFrontIntake.zeroSensors();
        mDownFlywheel.zeroSensors();
        mUpFlywheel.zeroSensors();
        mElevator.zeroSensors();
        
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        mBackIntake.registerEnabledLoops(enabledLooper);
        mFrontIntake.registerEnabledLoops(enabledLooper);
        mDownFlywheel.registerEnabledLoops(enabledLooper);
        mUpFlywheel.registerEnabledLoops(enabledLooper);
        mElevator.registerEnabledLoops(enabledLooper);
        
    }

    @Override
    public void setDebug(boolean _debug) {
        mBackIntake.setDebug(_debug);
        mFrontIntake.setDebug(_debug);
        mDownFlywheel.setDebug(_debug);
        mUpFlywheel.setDebug(_debug);
        mElevator.setDebug(_debug);
        
    }

    @Override
    public void retractAll() {
        mBackIntake.retractAll();
        mFrontIntake.retractAll();
        mDownFlywheel.retractAll();
        mUpFlywheel.retractAll();
        mElevator.retractAll();
        
    }

    public void wantIntakeFront(){
        mFrontIntake.wantIntaking();
        mElevator.goToSetpointInches(0.0);

    };

    public void wantIntakeBack(){
        mBackIntake.wantIntaking();
        mElevator.goToSetpointInches(0.0);
    }

    public void wantPrepareShot(shot wantedShot){
        mSuperStructureStates = SuperStructureStates.PrepareLaunch;
    }

    public void wantConfirmLaunch(Boolean shouldShoot){
        // todo 
    }

    public static SuperStructure getInstance(){
        return SUPER_STRUCTURE;
    }
    
}
