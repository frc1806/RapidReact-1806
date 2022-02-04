package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.CANifier;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.game.shot;
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

    private IntakeSubsystem mFrontIntake;
    private IntakeSubsystem mBackIntake;
    private FlywheelSubsystem mUpFlywheel;
    private FlywheelSubsystem mDownFlywheel;
    private ElevatorSubsystem mElevator;
    private SuperStructureStates mSuperStructureStates;
    private CANifier mCanifierUp = new CANifier(0);
    private CANifier mCanitiferDown = new CANifier(1);

    public SuperStructure(){
        mFrontIntake = new IntakeSubsystem(RobotMap.frontIntake, RobotMap.frontIntakeExtend, RobotMap.backIntakeExtend);
        mBackIntake = new IntakeSubsystem(RobotMap.rearIntake, RobotMap.backIntakeExtend, RobotMap.backIntakeRetract);
        mUpFlywheel = new FlywheelSubsystem(RobotMap.upFlywheel, Constants.kTopFlywheelKp, Constants.kTopFlywheelKi, Constants.kTopFlywheelKd, Constants.kTopFlywheelKf, Constants.kTopFlywheelIzone, false, Constants.kTopFlywheelConversionFactor, RobotMap.upFlywheel, Constants.kTopFlywheelKs, Constants.kTopFlywheelKv, mCanifierUp);
        mDownFlywheel =  new FlywheelSubsystem(RobotMap.downFlywheel, Constants.kTopFlywheelKp, Constants.kTopFlywheelKi, Constants.kTopFlywheelKd, Constants.kTopFlywheelKf, Constants.kTopFlywheelIzone, false, Constants.kTopFlywheelConversionFactor, RobotMap.upFlywheel, Constants.kTopFlywheelKs, Constants.kTopFlywheelKv, mCanitiferDown);
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
        mSuperStructureStates = SuperStructureStates.Idle;
        mFrontIntake.stop();
        mBackIntake.stop();
        mUpFlywheel.stop();
        mDownFlywheel.stop();
        
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

    public void wantIntakeFront(){
        // do this later
    };

    public void wantIntakeBack(){
        // todo
    }

    public void wantPrepareShot(shot wantedShot){
        // todo
    }

    public void wantConfirmLaunch(Boolean shouldShoot){
        // todo 
    }
    
}
