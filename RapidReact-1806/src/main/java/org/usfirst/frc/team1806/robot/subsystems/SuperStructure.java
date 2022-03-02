package org.usfirst.frc.team1806.robot.subsystems;

import java.util.function.DoubleSupplier;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.game.Shot;
import org.usfirst.frc.team1806.robot.loop.Looper;
import org.usfirst.frc.team1806.robot.util.PicoColorSensor;
import org.usfirst.frc.team1806.robot.util.PicoColorSensor.RawColor;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class SuperStructure implements Subsystem {

    public enum SuperStructureStates {
        IntakingFront,
        IntakingBack,
        Launching,
        Idle,
        Climbing,
        FrontIntakeFeedThrough,
        BackIntakeFeedThrough
    };

    public enum LaunchingStates {
        kPreparingLaunch,
        kChangeShot,
        kLaunching
    }

    
    public enum IdleStates {
        GoingHome,
        AtHome
    }

    private Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            // TODO Auto-generated method stub

        }

        @Override
        public void onLoop(double timestamp) {

            if(timestamp - lastCheckedAllianceTime > 2.5){
                mCurrentAlliance = DriverStation.getAlliance();
                lastCheckedAllianceTime = timestamp;
            }
            switch (mSuperStructureStates) {
                case IntakingFront:
                    mElevator.goToSetpointInches(0);
                    mFrontIntake.wantIntaking();
                    mBackIntake.stop();
                    mDualRollerSubsystem.startRoller();
                    mConveyor.loadConveyor();
                    mUpFlywheel.setReverseSpeed(1500.0);
                    mDownFlywheel.setReverseSpeed(1500.0);
                    mLunchboxAngler.goToAngle(0.0);
                    return;
                case IntakingBack:
                    mElevator.goToSetpointInches(0);
                    mFrontIntake.stop();
                    mBackIntake.wantIntaking();
                    mDualRollerSubsystem.startRoller();
                    mConveyor.loadConveyor();
                    mUpFlywheel.setReverseSpeed(1500.0);
                    mDownFlywheel.setReverseSpeed(1500.0);
                    mLunchboxAngler.goToAngle(0.0);
                    return;
                case Launching:
                    switch (mLaunchingStates) {
                        default:
                        case kPreparingLaunch:
                            mElevator.goToSetpointInches(mWantedShot.getLiftHeight());
                            mLunchboxAngler.goToAngle(mWantedShot.getLauncherAngle());
                            mUpFlywheel.setWantedSpeed(mWantedShot.getTopSpeed());
                            mDownFlywheel.setWantedSpeed(mWantedShot.getBottomSpeed());
                            mFrontIntake.stop();
                            mBackIntake.stop();
                            mDualRollerSubsystem.stop();
                            mConveyor.prepareForLaunch();
                            if (mWantConfirmShot && mLunchboxAngler.isAtAngle() && mElevator.isAtPosition() && mUpFlywheel.isSpeedInRange() && mDownFlywheel.isSpeedInRange()) 
                            {
                                mLaunchingStates = LaunchingStates.kLaunching;
                            }
                            break;
                        case kChangeShot:
                            if (!isShotAngleIncreasing){
                                mLunchboxAngler.goToAngle(mWantedShot.getLauncherAngle());
                                if(mLunchboxAngler.isAtAngle()); mLaunchingStates = LaunchingStates.kLaunching;
                            } else {
                                mElevator.goToSetpointInches(mWantedShot.getLiftHeight());
                                if (mElevator.isAtPosition()); mLaunchingStates = LaunchingStates.kLaunching;
                            }
                            break;
                        case kLaunching:
                            mElevator.goToSetpointInches(mWantedShot.getLiftHeight());
                            mUpFlywheel.setWantedSpeed(mWantedShot.getTopSpeed());
                            mLunchboxAngler.goToAngle(mWantedShot.getLauncherAngle());
                            mDownFlywheel.setWantedSpeed(mWantedShot.getBottomSpeed());
                            mFrontIntake.stop();
                            mBackIntake.stop();
                            mConveyor.launch();
                            mDualRollerSubsystem.stop();
                            if (!mWantConfirmShot || !mLunchboxAngler.isAtAngle() || !mElevator.isAtPosition() || !mUpFlywheel.isSpeedInRange() || !mDownFlywheel.isSpeedInRange()) // TODO: And a bunch of other logic
                            {
                                mLaunchingStates = LaunchingStates.kPreparingLaunch;
                            }
                            break;
                    }
                    return;
                default:
                case Idle:
                    switch(mIdleStates){
                        default:
                        case GoingHome:
                            mUpFlywheel.stop();
                            mDownFlywheel.stop();
                            mElevator.goToSetpointInches(0.0);
                            mFrontIntake.stop();
                            mBackIntake.stop();
                            mDualRollerSubsystem.stop();
                            mConveyor.stop();
                            mIdleStates = IdleStates.AtHome;
                            mLunchboxAngler.goToAngle(0.0);
                            if(mElevator.isAtPosition() && mLunchboxAngler.isAtAngle())
                            {
                                mIdleStates = IdleStates.AtHome;
                            }
                            break;
                        case AtHome:
                            mUpFlywheel.stop();
                            mDownFlywheel.stop();
                            mElevator.goToSetpointInches(Constants.kLiftBottomPivotHeight);
                            mFrontIntake.stop();
                            mBackIntake.stop();
                            mDualRollerSubsystem.stop();
                            mConveyor.stop();
                            mLunchboxAngler.stop();

                            break;
                    }
                    return;
                case Climbing:
                    mUpFlywheel.stop();
                    mDownFlywheel.stop();
                    mElevator.stop();
                    mFrontIntake.stop();
                    mBackIntake.stop();
                    mDualRollerSubsystem.stop();
                    mConveyor.stop();
                    mLunchboxAngler.stop();
                    return;
                case FrontIntakeFeedThrough:
                    mUpFlywheel.stop();
                    mDownFlywheel.stop();
                    mElevator.stop();
                    mFrontIntake.wantIntaking();
                    mBackIntake.stop();
                    mDualRollerSubsystem.feedBackwards();
                    mConveyor.stop();
                    mLunchboxAngler.goToAngle(0.0);
                    return;
                case BackIntakeFeedThrough:
                    mUpFlywheel.stop();
                    mDownFlywheel.stop();
                    mElevator.stop();
                    mFrontIntake.stop();
                    mBackIntake.wantIntaking();
                    mDualRollerSubsystem.feedForward();
                    mConveyor.stop();
                    mLunchboxAngler.goToAngle(0.0);
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
    private DualRollerSubsystem mDualRollerSubsystem = DualRollerSubsystem.getInstance();
    private Conveyor mConveyor;
    private Boolean isShotAngleIncreasing = false;
    private LaunchBoxAngler mLunchboxAngler;
    private SuperStructureStates mSuperStructureStates;
    private PicoColorSensor mPicoColorSensor;
    private LaunchingStates mLaunchingStates;
    private Boolean mWantConfirmShot;
    private Shot mWantedShot; // make sure to null check this
    private IdleStates mIdleStates;
    private Alliance mCurrentAlliance;
    private double lastCheckedAllianceTime;

    private SuperStructure() {
        mElevator = ElevatorSubsystem.getInstance();
        mPicoColorSensor = new PicoColorSensor();
        mFrontIntake = new IntakeSubsystem(RobotMap.frontIntake, RobotMap.frontIntakeExtend, RobotMap.frontIntakeRetract);
        mBackIntake = new IntakeSubsystem(RobotMap.rearIntake, RobotMap.backIntakeExtend, RobotMap.backIntakeRetract);
        mUpFlywheel = new FlywheelSubsystem(RobotMap.upFlywheel, Constants.kTopFlywheelKp, Constants.kTopFlywheelKi,
                Constants.kTopFlywheelKd, Constants.kTopFlywheelKf, Constants.kTopFlywheelIzone, false,  Constants.kTopFlywheelKs,
                Constants.kTopFlywheelKv, RobotMap.upFlyWheelEncoderA, RobotMap.upFlywheelEncoderB);
        mDownFlywheel = new FlywheelSubsystem(RobotMap.downFlywheel, Constants.kTopFlywheelKp, Constants.kTopFlywheelKi,
                Constants.kTopFlywheelKd, Constants.kTopFlywheelKf, Constants.kTopFlywheelIzone, true,
                 Constants.kTopFlywheelKs,
                Constants.kTopFlywheelKv, RobotMap.downFlywheelEncoderA, RobotMap.downFlywheelEncoderB);
        mConveyor = new Conveyor();
        mLunchboxAngler = LaunchBoxAngler.getInstance();
        mSuperStructureStates = SuperStructureStates.Idle;
        mLaunchingStates = LaunchingStates.kPreparingLaunch;
        mIdleStates = IdleStates.GoingHome;
        mWantConfirmShot = false;
        mCurrentAlliance = Alliance.Invalid;
        lastCheckedAllianceTime = 0.0
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
        mConveyor.outputToSmartDashboard();
        SmartDashboard.putString("Superstructure State", mSuperStructureStates.name());
        SmartDashboard.putString("Superstructure Launching State", mLaunchingStates.name());
        SmartDashboard.putString("Superstructure Idle State", mIdleStates.name());
    }

    @Override
    public void stop() {
        mIdleStates = IdleStates.GoingHome;
        mSuperStructureStates = SuperStructureStates.Idle;

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
        mConveyor.registerEnabledLoops(enabledLooper);
        enabledLooper.register(mLoop);
    }

    @Override
    public void setDebug(boolean _debug) {
        mBackIntake.setDebug(_debug);
        mFrontIntake.setDebug(_debug);
        mDownFlywheel.setDebug(_debug);
        mUpFlywheel.setDebug(_debug);
        mElevator.setDebug(_debug);
        mConveyor.setDebug(_debug);

    }

    @Override
    public void retractAll() {
        mSuperStructureStates = SuperStructureStates.Idle;
        mBackIntake.retractAll();
        mFrontIntake.retractAll();
        mDownFlywheel.retractAll();
        mUpFlywheel.retractAll();
        mElevator.retractAll();
        mConveyor.retractAll();

    }

    public void wantIntakeFront() {
        mSuperStructureStates = SuperStructureStates.IntakingFront;

    };

    public void wantIntakeBack() {
        mSuperStructureStates = SuperStructureStates.IntakingBack;
    }

    public void wantPrepareShot(Shot wantedShot) {
        if (mSuperStructureStates != SuperStructureStates.Launching) {
            mLaunchingStates = LaunchingStates.kPreparingLaunch;
        }
        else
        {
            if (mWantedShot != null && Math.abs(mWantedShot.getLauncherAngle() -wantedShot.getLauncherAngle()) > 0.001){
                isShotAngleIncreasing = Math.abs(wantedShot.getLauncherAngle()) > Math.abs(mWantedShot.getLauncherAngle()); 
                mLaunchingStates = LaunchingStates.kChangeShot;
            }
        }
        mSuperStructureStates = SuperStructureStates.Launching;
        mWantedShot = wantedShot;
    }

    public void wantConfirmLaunch(Boolean shouldShoot) {
        mWantConfirmShot = shouldShoot;
    }
    
    public void wantFeedFrontIntake(){
        mSuperStructureStates = SuperStructureStates.FrontIntakeFeedThrough;
    }

    public void wantFeedBackIntake(){
        mSuperStructureStates = SuperStructureStates.BackIntakeFeedThrough;

    }

    public static SuperStructure getInstance() {
        return SUPER_STRUCTURE;
    }


    public boolean doesFrontColorSensorDetectWrongBall(){

        mPicoColorSensor.getRawColor0();
        
        switch(mCurrentAlliance){
            case Blue:
                //Do code for if we're on the Blue Alliance Here
                break;
            default:
            case Invalid:
                return false;
            case Red:
                //Do code for if we're on the Red Alliance Here
                break;
            
            
        }
        
        



    }

    public boolean doesBackColorSensorDetectWrongBall(){

        
        mPicoColorSensor.getRawColor0();

        if mCurrentAlliance
        

    }

    public boolean isColorBlue(RawColor color){
        if(color.red < Constants.kBlueBallMinimumValues.red || color.red > Constants.kBlueBallMaxValues.red) return false;
        if(color.green < Constants.kBlueBallMinimumValues.green || color.green > Constants.kBlueBallMaxValues.green) return false;
        if(color.blue < Constants.kBlueBallMinimumValues.blue || color.blue > Constants.kBlueBallMaxValues.blue) return false;
        return true;
    }

    public boolean isColorRed(RawColor color){
        if(color.red < Constants.kRedBallMinimumValues.red || color.red > Constants.kRedBallMaxValues.red) return false;
        if(color.green < Constants.kRedBallMinimumValues.green || color.green > Constants.kRedBallMaxValues.green) return false;
        if(color.blue < Constants.kRedBallMinimumValues.blue || color.blue > Constants.kBlueBallMaxValues.blue) return false;
        return true;
    }

    private static Map<String, Object> FLYWHEEL_SPEEDS = new HashMap<>();
    private static Map<String, Object> SHOOTER_ANGLE = new HashMap<>();

    static {
        FLYWHEEL_SPEEDS.put("Min", 0.0d);
        FLYWHEEL_SPEEDS.put("Max", 3000d);
        FLYWHEEL_SPEEDS.put("Block increment", 1.0d);
    }


    static {
        SHOOTER_ANGLE.put("Min", -180.0d);
        SHOOTER_ANGLE.put("Max", 180d);
        SHOOTER_ANGLE.put("Block increment", 0.1d);
    }

    @Override
    public void setupDriverTab() {

        Robot.getMainDriverTab().addNumber("Up Flywheel Speed", new DoubleSupplier() {

            @Override
			public double getAsDouble() {
				return mUpFlywheel.getCurrentRPM();
			}

        }).withWidget(BuiltInWidgets.kDial).withPosition(9,1).withSize(2,2).withProperties(FLYWHEEL_SPEEDS); //add .withProperties if neccesary


        Robot.getMainDriverTab().addNumber("Down Flywheel Speed", new DoubleSupplier() {

            @Override
			public double getAsDouble() {
				return mDownFlywheel.getCurrentRPM();
			}


        }).withWidget(BuiltInWidgets.kDial).withPosition(9,3).withSize(2,2).withProperties(FLYWHEEL_SPEEDS); //add .withProperties if neccesary




        Robot.getMainDriverTab().addNumber("Up Wanted Flywheel Speed", new DoubleSupplier() {

            @Override
			public double getAsDouble() {
				return mUpFlywheel.getWantedRPM();
			}

        }).withWidget(BuiltInWidgets.kDial).withPosition(7,1).withSize(2,2).withProperties(FLYWHEEL_SPEEDS); //add .withProperties if neccesary


        Robot.getMainDriverTab().addNumber("Down Wanted Flywheel Speed", new DoubleSupplier() {

            @Override
			public double getAsDouble() {
				return mDownFlywheel.getWantedRPM();
			}
        

        }).withWidget(BuiltInWidgets.kDial).withPosition(7,3).withSize(2,2).withProperties(FLYWHEEL_SPEEDS); //add .withProperties if neccesary

        

        Robot.getMainDriverTab().addNumber("Shooter Angle", new DoubleSupplier() {

            @Override
            public double getAsDouble() {
                return mLunchboxAngler.getCurrentAngle();
            }
        

        }).withWidget(BuiltInWidgets.kDial).withPosition(-1,-1).withSize(1,1).withProperties(SHOOTER_ANGLE); //add .withProperties if neccesary


        }
       
    







        // TODO Auto-generated method stub
        
    }

