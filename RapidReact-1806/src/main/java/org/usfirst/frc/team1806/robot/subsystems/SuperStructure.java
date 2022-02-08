package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.game.shot;
import org.usfirst.frc.team1806.robot.loop.Looper;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


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
    private CANifier mCanifier = new CANifier(0);
    private final I2C.Port i2cPort1 = I2C.Port.kOnboard;
    private final I2C.Port i2cPort2 = I2C.Port.kMXP;
    ColorSensorV3 m_colorSensorFront;
    ColorSensorV3 m_colorSensorRear;

    public SuperStructure(){
        mFrontIntake = new IntakeSubsystem(RobotMap.frontIntake, RobotMap.frontIntakeExtend, RobotMap.backIntakeExtend);
        mBackIntake = new IntakeSubsystem(RobotMap.rearIntake, RobotMap.backIntakeExtend, RobotMap.backIntakeRetract);
        mUpFlywheel = new FlywheelSubsystem(RobotMap.upFlywheel, Constants.kTopFlywheelKp, Constants.kTopFlywheelKi, Constants.kTopFlywheelKd, Constants.kTopFlywheelKf, Constants.kTopFlywheelIzone, false, Constants.kTopFlywheelConversionFactor, RobotMap.upFlywheel, Constants.kTopFlywheelKs, Constants.kTopFlywheelKv, mCanifier);
        m_colorSensorFront = new ColorSensorV3(i2cPort1);
        m_colorSensorFront = new ColorSensorV3(i2cPort2);
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
