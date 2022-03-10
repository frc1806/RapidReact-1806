package org.usfirst.frc.team1806.robot.subsystems;

import java.util.function.DoubleSupplier;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.game.Shot;
import org.usfirst.frc.team1806.robot.loop.Looper;
import org.usfirst.frc.team1806.robot.util.BisectedCircularBuffer;
import org.usfirst.frc.team1806.robot.util.PicoColorSensor;
import org.usfirst.frc.team1806.robot.util.PicoColorSensor.RawColor;
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

    public enum IntakeStates{
        Intaking,
        FeedThrough
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

            //Cargo counting
            flywheelAverageSpeedsBuffer.addLast((mUpFlywheel.getCurrentRPM() + mDownFlywheel.getCurrentRPM()) / 2.0);
            double olderAverage = flywheelAverageSpeedsBuffer.getOlderAverage();
            double recentAverage = flywheelAverageSpeedsBuffer.getMoreRecentAverage();
            boolean isDecreasing = false;

            //main loop
            switch (mSuperStructureStates) {
                case IntakingFront:
                    mElevator.goToSetpointInches(Constants.kLiftBottomPivotHeight);
                    mFrontIntake.wantIntaking();
                    mBackIntake.stop();
                    mDualRollerSubsystem.startRoller();
                    mConveyor.loadConveyor();
                    mUpFlywheel.setReverseSpeed(500.0);
                    mDownFlywheel.setReverseSpeed(500.0);
                    mLaunchboxAngler.goToAngle(0.0);
                    //cargo counting
                    if(olderAverage < 0 && recentAverage < 0)
                    {
                        if(recentAverage - olderAverage > (olderAverage* .05))
                        {
                            isDecreasing = true;
                            if(isDecreasing && !isFlywheelSpeedDecreasing)
                            {
                                ballCount ++;
                            }
                        }
                    }
                    break;
                case IntakingBack:
                    mElevator.goToSetpointInches(Constants.kLiftBottomPivotHeight);
                    mFrontIntake.stop();
                    mBackIntake.wantIntaking();
                    mDualRollerSubsystem.startRoller();
                    mConveyor.loadConveyor();
                    mUpFlywheel.setReverseSpeed(500.0);
                    mDownFlywheel.setReverseSpeed(500.0);
                    mLaunchboxAngler.goToAngle(0.0);
                    if(olderAverage < 0 && recentAverage < 0)
                    {
                        if(recentAverage - olderAverage > (olderAverage* .05))
                        {
                            isDecreasing = true;
                            if(isDecreasing && !isFlywheelSpeedDecreasing)
                            {
                                ballCount ++;
                            }
                        }
                    }
                    break;
                case Launching:
                    switch (mLaunchingStates) {
                        default:
                        case kPreparingLaunch:
                            mElevator.goToSetpointInches(mWantedShot.getLiftHeight());

                            if(mElevator.isAbovePosition(Constants.kLaunchBoxInchesToFreedom + Constants.kLiftBottomPivotHeight))
                            {
                                mLaunchboxAngler.goToAngle(mWantedShot.getLauncherAngle());
                            }
                            else {
                                mLaunchboxAngler.goToAngle(0.0);
                            }
                            mUpFlywheel.setWantedSpeed(mWantedShot.getTopSpeed());
                            mDownFlywheel.setWantedSpeed(mWantedShot.getBottomSpeed());
                            mFrontIntake.stop();
                            mBackIntake.stop();
                            mDualRollerSubsystem.stop();
                            mConveyor.prepareForLaunch();
                            if (mWantConfirmShot && mLaunchboxAngler.isAtAngle() && mElevator.isAtPosition() && mUpFlywheel.isSpeedInRange() && mDownFlywheel.isSpeedInRange()) 
                            {
                                mLaunchingStates = LaunchingStates.kLaunching;
                            }
                            break;
                        case kChangeShot:
                            if (!isShotAngleIncreasing){
                                if(mElevator.isAbovePosition(Constants.kLaunchBoxInchesToFreedom + Constants.kLiftBottomPivotHeight))
                                {
                                    mLaunchboxAngler.goToAngle(mWantedShot.getLauncherAngle());
                                }
                                else {
                                    mLaunchboxAngler.goToAngle(0.0);
                                }
                                if(mLaunchboxAngler.isAtAngle()); mLaunchingStates = LaunchingStates.kLaunching;
                            } else {
                                mElevator.goToSetpointInches(mWantedShot.getLiftHeight());
                                if (mElevator.isAtPosition()); mLaunchingStates = LaunchingStates.kLaunching;
                            }
                            break;
                        case kLaunching:
                            mElevator.goToSetpointInches(mWantedShot.getLiftHeight());
                            mUpFlywheel.setWantedSpeed(mWantedShot.getTopSpeed());
                            if(mElevator.isAbovePosition(Constants.kLaunchBoxInchesToFreedom + Constants.kLiftBottomPivotHeight))
                            {
                                mLaunchboxAngler.goToAngle(mWantedShot.getLauncherAngle());
                            }
                            else {
                                mLaunchboxAngler.goToAngle(0.0);
                            }
                            mDownFlywheel.setWantedSpeed(mWantedShot.getBottomSpeed());
                            mFrontIntake.stop();
                            mBackIntake.stop();
                            mConveyor.launch();
                            mDualRollerSubsystem.stop();
                            if(olderAverage > 0 && recentAverage > 0)
                            {
                                if(recentAverage - olderAverage < -(olderAverage* .05))
                                {
                                    isDecreasing = true;
                                    if(isDecreasing && !isFlywheelSpeedDecreasing)
                                    {
                                        ballCount --;
                                    }
                                }
                            }
                            if (!mWantConfirmShot || !mLaunchboxAngler.isAtAngle() || !mElevator.isAtPosition() || !mUpFlywheel.isSpeedInRange() || !mDownFlywheel.isSpeedInRange()) // TODO: And a bunch of other logic
                            {
                                mLaunchingStates = LaunchingStates.kPreparingLaunch;
                            }
                            break;
                    }
                    break;
                default:
                case Idle:
                    switch(mIdleStates){
                        default:
                        case GoingHome:
                            mUpFlywheel.stop();
                            mDownFlywheel.stop();
                            if(mLaunchboxAngler.checkIfAtArbitraryAngle(0.0))
                            {
                                mElevator.goToSetpointInches(Constants.kLiftBottomPivotHeight);
                            }
                            else
                            {
                                mElevator.goToSetpointInches(Constants.kLiftBottomPivotHeight + Constants.kLaunchBoxInchesToFreedom);
                            }
                            mFrontIntake.stop();
                            mBackIntake.stop();
                            mDualRollerSubsystem.stop();
                            mConveyor.stop();
                            mIdleStates = IdleStates.AtHome;
                            mLaunchboxAngler.goToAngle(0.0);
                            if(mElevator.isAtPosition() && mLaunchboxAngler.checkIfAtArbitraryAngle(0.0));
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
                            mLaunchboxAngler.goToAngle(0.0);

                            break;
                    }
                    break;
                case Climbing:
                    mUpFlywheel.stop();
                    mDownFlywheel.stop();
                    mElevator.stop();
                    mFrontIntake.stop();
                    mBackIntake.stop();
                    mDualRollerSubsystem.stop();
                    mConveyor.stop();
                    mLaunchboxAngler.goToAngle(0.0);
                    break;
                case FrontIntakeFeedThrough:
                    mUpFlywheel.stop();
                    mDownFlywheel.stop();
                    mElevator.stop();
                    mFrontIntake.wantIntaking();
                    mBackIntake.stop();
                    mDualRollerSubsystem.feedBackwards();
                    mConveyor.stop();
                    mLaunchboxAngler.goToAngle(0.0);
                    break;
                case BackIntakeFeedThrough:
                    mUpFlywheel.stop();
                    mDownFlywheel.stop();
                    mElevator.stop();
                    mFrontIntake.stop();
                    mBackIntake.wantIntaking();
                    mDualRollerSubsystem.feedForward();
                    mConveyor.stop();
                    mLaunchboxAngler.goToAngle(0.0);
                    break;
            }
            isFlywheelSpeedDecreasing = isDecreasing;
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
    private LaunchBoxAngler mLaunchboxAngler;
    private SuperStructureStates mSuperStructureStates;
    private PicoColorSensor mPicoColorSensor;
    private LaunchingStates mLaunchingStates;
    private Boolean mWantConfirmShot;
    private Shot mWantedShot; // make sure to null check this
    private IdleStates mIdleStates;
    private Alliance mCurrentAlliance;
    private double lastCheckedAllianceTime;

    //Cargo counting
    private BisectedCircularBuffer flywheelAverageSpeedsBuffer;
    private boolean isFlywheelSpeedDecreasing;
    private int ballCount;

    private SuperStructure() {
        mElevator = ElevatorSubsystem.getInstance();
        mPicoColorSensor = new PicoColorSensor();
        mFrontIntake = new IntakeSubsystem(RobotMap.frontIntake, RobotMap.frontIntakeExtend, RobotMap.frontIntakeRetract);
        mBackIntake = new IntakeSubsystem(RobotMap.rearIntake, RobotMap.backIntakeExtend, RobotMap.backIntakeRetract);
        mUpFlywheel = new FlywheelSubsystem(RobotMap.upFlywheel, Constants.kTopFlywheelKp, Constants.kTopFlywheelKi,
                Constants.kTopFlywheelKd, Constants.kTopFlywheelKf, Constants.kTopFlywheelIzone, true,  Constants.kTopFlywheelKs,
                Constants.kTopFlywheelKv, Constants.kTopFlywheelKa, RobotMap.upFlyWheelEncoderA, RobotMap.upFlywheelEncoderB, false);
        mDownFlywheel = new FlywheelSubsystem(RobotMap.downFlywheel, Constants.kTopFlywheelKp, Constants.kTopFlywheelKi,
                Constants.kTopFlywheelKd, Constants.kTopFlywheelKf, Constants.kTopFlywheelIzone, false,
                 Constants.kTopFlywheelKs,
                Constants.kTopFlywheelKv, Constants.kTopFlywheelKa, RobotMap.downFlywheelEncoderA, RobotMap.downFlywheelEncoderB, false);
        mConveyor = new Conveyor();
        mLaunchboxAngler = LaunchBoxAngler.getInstance();
        mSuperStructureStates = SuperStructureStates.Idle;
        mLaunchingStates = LaunchingStates.kPreparingLaunch;
        mIdleStates = IdleStates.GoingHome;
        mWantConfirmShot = false;
        mCurrentAlliance = Alliance.Invalid;
        lastCheckedAllianceTime = 0.0;
        flywheelAverageSpeedsBuffer = new BisectedCircularBuffer(10, 5);
        isFlywheelSpeedDecreasing = false;

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
        SmartDashboard.putNumber("Top Flywheel Output", mUpFlywheel.getOutputPower());
        SmartDashboard.putNumber("Bottom Flywheel Output", mDownFlywheel.getOutputPower());
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
    
    public void overwiteBallCount(int wantedCount){
        ballCount = wantedCount;
    }


    public boolean doesFrontColorSensorDetectWrongBall(){
        
        switch(mCurrentAlliance){
            case Blue:
                //Do code for if we're on the Blue Alliance Here
                return isColorRed(mPicoColorSensor.getRawColor0());
            default:
            case Invalid:
                return false;
            case Red:
                //Do code for if we're on the Red Alliance Here
                return isColorBlue(mPicoColorSensor.getRawColor0());   
        }
    }
        
        

    public boolean doesBackColorSensorDetectWrongBall(){

        
        mPicoColorSensor.getRawColor0();
       
        switch(mCurrentAlliance){
            case Blue:
            return isColorRed(mPicoColorSensor.getRawColor1());
            default:
            case Invalid:
                return false;
            case Red:
            return isColorBlue(mPicoColorSensor.getRawColor1());  
            
            
        }

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
    private static Map<String, Object> RGB_VALUE = new HashMap<>();
    private static Map<String, Object> BALL_COUNT = new HashMap<>();

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

    static {
        RGB_VALUE.put("Min", 0.0d);
        RGB_VALUE.put("Max", 255.0d);
        RGB_VALUE.put("Block increment", 1.0d);
    }

    static {
        BALL_COUNT.put("Min", 0);
        BALL_COUNT.put("Max", 2);
        BALL_COUNT.put("Block increment", 1d);
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
                return mLaunchboxAngler.getCurrentAngle();
            }
        

        }).withWidget(BuiltInWidgets.kDial).withPosition(-1,-1).withSize(1,1).withProperties(SHOOTER_ANGLE); //add .withProperties if neccesary

        //TODO: Move position
        Robot.getMainDriverTab().addNumber("Ball Count", new DoubleSupplier() {

            @Override
            public double getAsDouble() {
                return ballCount;
            }
        

        }).withWidget(BuiltInWidgets.kDial).withPosition(-1,-1).withSize(1,1).withProperties(BALL_COUNT); //add .withProperties if neccesary

        Robot.getMainDriverTab().addNumber("Red 0", new DoubleSupplier() {

            @Override
            public double getAsDouble() {
                return mPicoColorSensor.getRawColor0().red;
            }
        

        }).withWidget(BuiltInWidgets.kDial).withPosition(0,3).withSize(1,1).withProperties(RGB_VALUE); //add .withProperties if neccesary


        Robot.getMainDriverTab().addNumber("Green 0", new DoubleSupplier() {

            @Override
            public double getAsDouble() {
                return mPicoColorSensor.getRawColor0().green;
            }
        

        }).withWidget(BuiltInWidgets.kDial).withPosition(1,3).withSize(1,1).withProperties(RGB_VALUE); //add .withProperties if neccesary




        Robot.getMainDriverTab().addNumber("Blue 0", new DoubleSupplier() {

            @Override
            public double getAsDouble() {
                return mPicoColorSensor.getRawColor0().blue;
            }
        

        }).withWidget(BuiltInWidgets.kDial).withPosition(2,3).withSize(1,1).withProperties(RGB_VALUE); //add .withProperties if neccesary




        }
       
    







        // TODO Auto-generated method stub
        
    }

