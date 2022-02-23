package org.usfirst.frc.team1806.robot.subsystems;

import java.util.function.DoubleSupplier;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.game.Shot;
import org.usfirst.frc.team1806.robot.loop.Looper;
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
            switch (mSuperStructureStates) {
                case IntakingFront:
                    mElevator.goToSetpointInches(Constants.kLiftBottomPivotHeight);
                    mFrontIntake.wantIntaking();
                    mBackIntake.stop();
                    mDualRollerSubsystem.startRoller();
                    mConveyor.loadConveyor();
                    mUpFlywheel.setReverseSpeed(1500.0);
                    mDownFlywheel.setReverseSpeed(1500.0);
                    mLunchboxAngler.goToAngle(0.0);
                    return;
                case IntakingBack:
                    mElevator.goToSetpointInches(Constants.kLiftBottomPivotHeight);
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
                                mLaunchingStates = LaunchingStates.kChangeShot;
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
                            mElevator.goToSetpointInches(Constants.kLiftBottomPivotHeight);
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
                        case AtHome:
                            mUpFlywheel.stop();
                            mDownFlywheel.stop();
                            mElevator.stop();
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
    private final I2C.Port i2cPort1 = I2C.Port.kOnboard;
    private final I2C.Port i2cPort2 = I2C.Port.kMXP;
    ColorSensorV3 m_colorSensorFront;
    ColorSensorV3 m_colorSensorRear;
    private LaunchingStates mLaunchingStates;
    private Boolean mWantConfirmShot;
    private Shot mWantedShot; // make sure to null check this
    private IdleStates mIdleStates;

    private SuperStructure() {
        mElevator = ElevatorSubsystem.getInstance();
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

    }

    @Override
    public void stop() {
        mIdleStates = IdleStates.AtHome;
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
            if (mWantedShot != wantedShot){
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



    private static Map<String, Object> FLYWHEEL_SPEEDS = new HashMap<>();

    static {
        FLYWHEEL_SPEEDS.put("Min", 0.0d);
        FLYWHEEL_SPEEDS.put("Max", 3000d);
        FLYWHEEL_SPEEDS.put("Block increment", 1.0d);
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
        

        }).withWidget(BuiltInWidgets.kDial).withPosition(-1,-1).withSize(1,1); //add .withProperties if neccesary


        }

        // TODO Auto-generated method stub
        
    }


