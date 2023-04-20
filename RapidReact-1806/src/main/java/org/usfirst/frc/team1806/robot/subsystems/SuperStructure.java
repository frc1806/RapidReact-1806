package org.usfirst.frc.team1806.robot.subsystems;

import java.util.function.DoubleSupplier;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.OI;
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
        ClimbingMid,
        ClimbingLow,
        FrontIntakeFeedThrough,
        BackIntakeFeedThrough,
    };

    public enum LaunchingStates {
        kPreparingLaunch,
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

    public enum InnerBallPathModes{
        kIntake,
        kReverse,
        kIdle
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
                    mCompressor.disable();
                    mDriveTrainSubsystem.setCurrentLimitPerMotor(Constants.kDriveLimitedAmpLimit);
                    mDriveTrainSubsystem.setOpenLoopRampRate(Constants.kDriveLimitedRampRate);
                    if(timestamp - intakePowerManagementChangeTime > .15){
                        intakePowerManagementStage++;
                        intakePowerManagementChangeTime = timestamp;
                    }
                    switch(intakePowerManagementStage){
                        case 0:
                            mDualRollerSubsystem.stop();
                        case 1:
                            mUpFlywheel.stop();
                            mDownFlywheel.stop();
                        case 2:
                            mConveyor.stop();
                        case 3:
                        default:
                    }
                    switch(intakePowerManagementStage){
                        default:
                        case 3:
                            mConveyor.loadConveyor();
                        case 2:
                            mUpFlywheel.setReverseSpeed(1500.0);
                            mDownFlywheel.setReverseSpeed(1500.0);
                        case 1:
                            mDualRollerSubsystem.startRoller();
                        case 0:
                    }
                    mFrontIntake.wantIntaking();
                    mBackIntake.stop();
                    // mDualRollerSubsystem.startRoller();
                    // mConveyor.loadConveyor();
                    // mUpFlywheel.setReverseSpeed(1500.0);
                    // mDownFlywheel.setReverseSpeed(1500.0);
                    mLaunchboxAngler.goToAngle(0.0);
                    //cargo counting
                    if(olderAverage < 0 && recentAverage < 0)
                    {
                        if(recentAverage - olderAverage > (olderAverage* .3))
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
                    mCompressor.disable();
                    mDriveTrainSubsystem.setCurrentLimitPerMotor(Constants.kDriveLimitedAmpLimit);
                    mDriveTrainSubsystem.setOpenLoopRampRate(Constants.kDriveLimitedRampRate);
                    if(timestamp - intakePowerManagementChangeTime > .15){
                        intakePowerManagementStage++;
                        intakePowerManagementChangeTime = timestamp;
                    }
                    switch(intakePowerManagementStage){
                        case 0:
                            mDualRollerSubsystem.stop();
                        case 1:
                            mUpFlywheel.stop();
                            mDownFlywheel.stop();
                        case 2:
                            mConveyor.stop();
                        case 3:
                        default:
                    }
                    switch(intakePowerManagementStage){
                        default:
                        case 3:
                            mConveyor.loadConveyor();
                        case 2:
                            mUpFlywheel.setReverseSpeed(1500.0);
                            mDownFlywheel.setReverseSpeed(1500.0);
                        case 1:
                            mDualRollerSubsystem.startRoller();
                        case 0:
                    }
                    mFrontIntake.stop();
                    mBackIntake.wantIntaking();
                    mLaunchboxAngler.goToAngle(0.0);
                    if(olderAverage < 0 && recentAverage < 0)
                    {
                        if(recentAverage - olderAverage > (olderAverage* .3))
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
                    mCompressor.disable();
                    mDriveTrainSubsystem.setCurrentLimitPerMotor(Constants.kDriveLimitedAmpLimit);
                    mDriveTrainSubsystem.setOpenLoopRampRate(Constants.kDriveLimitedRampRate);
                    if(frontSweep){
                        mFrontIntake.wantSweep();
                    }else{
                        mFrontIntake.stop();
                    }
                    if(rearSweep){
                        mBackIntake.wantSweep();
                    }else{
                        mBackIntake.stop();
                    }
                    switch (mLaunchingStates) {
                        default:
                        case kPreparingLaunch:
                            mLaunchboxAngler.goToAngle(mWantedShot.getLauncherAngle());
                            if(Math.abs(mLaunchboxAngler.getCurrentAngle() - mWantedShot.getLauncherAngle()) < 25){
                                mUpFlywheel.setWantedSpeed(mWantedShot.getTopSpeed());
                                mDownFlywheel.setWantedSpeed(mWantedShot.getBottomSpeed());
                            }
                            else{
                                mUpFlywheel.stop();
                                mDownFlywheel.stop();
                            }
                            mDualRollerSubsystem.stop();
                            mConveyor.prepareForLaunch();
                            if (mWantConfirmShot && mLaunchboxAngler.checkIfAtArbitraryAngle(mWantedShot.getLauncherAngle())&& mUpFlywheel.isAtSpeed(mWantedShot.getTopSpeed()) && mDownFlywheel.isAtSpeed(mWantedShot.getBottomSpeed())) 
                            {
                                mLaunchingStates = LaunchingStates.kLaunching;
                            }
                            break;
                        case kLaunching:
                            mUpFlywheel.setWantedSpeed(mWantedShot.getTopSpeed());
                            mDownFlywheel.setWantedSpeed(mWantedShot.getBottomSpeed());
                            mLaunchboxAngler.goToAngle(mWantedShot.getLauncherAngle());
                            mConveyor.launch();
                            mDualRollerSubsystem.stop();
                            if(olderAverage > 0 && recentAverage > 0)
                            {
                                if(recentAverage - olderAverage < -(olderAverage* .3))
                                {
                                    isDecreasing = true;
                                    if(isDecreasing && !isFlywheelSpeedDecreasing)
                                    {
                                        ballCount --;
                                    }
                                }
                            }
                            if (!mWantConfirmShot || (mWantedShot.getIsPreciseShot() &&(!mLaunchboxAngler.checkIfAtArbitraryAngle(mWantedShot.getLauncherAngle()))|| !mUpFlywheel.isSpeedInRange() || !mDownFlywheel.isSpeedInRange())) // TODO: And a bunch of other logic
                            {
                                mLaunchingStates = LaunchingStates.kPreparingLaunch;
                            }
                            break;
                        }
                    break;
                default:
                case Idle:
                    if(!mDebugController.getButtonB()){
                        mCompressor.enableDigital();
                    }
                    mDriveTrainSubsystem.setCurrentLimitPerMotor(Constants.kDriveNormalAmpLimit);
                    mDriveTrainSubsystem.setOpenLoopRampRate(Constants.kDriveNormalRampRate);
                    if(frontSweep){
                        mFrontIntake.wantSweep();
                    }else{
                        mFrontIntake.stop();
                    }
                    if(rearSweep){
                        mBackIntake.wantSweep();
                    }else{
                        mBackIntake.stop();
                    }
                    switch(mIdleStates){
                        default:
                        case GoingHome:
                            mIdleStates = IdleStates.AtHome;
                            mLaunchboxAngler.goToAngle(0.0);
                            if(mLaunchboxAngler.checkIfAtArbitraryAngle(0.0));
                            {
                                mIdleStates = IdleStates.AtHome;
                            }
                            break;
                        case AtHome:
                            mLaunchboxAngler.goToAngle(0.0);
                            break;
                    }

                    switch(mInnerBallPathMode){
                        default:
                        case kIdle:
                            mDualRollerSubsystem.stop();
                            mConveyor.stop();
                            mUpFlywheel.stop();
                            mDownFlywheel.stop();
                            break;
                        case kIntake:
                            mDualRollerSubsystem.startRoller();
                            mConveyor.loadConveyor();
                            mUpFlywheel.setReverseSpeed(1500.0);
                            mDownFlywheel.setReverseSpeed(1500.0);
                            break;
                        case kReverse:
                            mDualRollerSubsystem.feedBackwards();
                            mConveyor.launch();
                            mUpFlywheel.setWantedSpeed(1500.0);
                            mDownFlywheel.setWantedSpeed(1500.0);
                            break;
                    }
                    break;
                case ClimbingMid:
                    mCompressor.disable();
                    mDriveTrainSubsystem.setCurrentLimitPerMotor(Constants.kDriveNormalAmpLimit);
                    mDriveTrainSubsystem.setOpenLoopRampRate(Constants.kDriveNormalRampRate);
                    mUpFlywheel.stop();
                    mDownFlywheel.stop();
                    if(frontSweep){
                        mFrontIntake.wantSweep();
                    }else{
                        mFrontIntake.stop();
                    }
                    if(rearSweep){
                        mBackIntake.wantSweep();
                    }else{
                        mBackIntake.stop();
                    }
                    mDualRollerSubsystem.stop();
                    mConveyor.stop();
                    mLaunchboxAngler.goToAngle(77.7); //Hi 987
                    break;

                case ClimbingLow:
                    mCompressor.disable();
                    mDriveTrainSubsystem.setCurrentLimitPerMotor(Constants.kDriveNormalAmpLimit);
                    mDriveTrainSubsystem.setOpenLoopRampRate(Constants.kDriveNormalRampRate);
                    mUpFlywheel.stop();
                    mDownFlywheel.stop();
                    if(frontSweep){
                        mFrontIntake.wantSweep();
                    }else{
                        mFrontIntake.stop();
                    }
                    if(rearSweep){
                        mBackIntake.wantSweep();
                    }else{
                        mBackIntake.stop();
                    }
                    mDualRollerSubsystem.stop();
                    mConveyor.stop();
                    mLaunchboxAngler.goToAngle(-66.6);
                    break;

                case FrontIntakeFeedThrough:
                    mDriveTrainSubsystem.setCurrentLimitPerMotor(Constants.kDriveLimitedAmpLimit);
                    mDriveTrainSubsystem.setOpenLoopRampRate(Constants.kDriveLimitedRampRate);
                    mCompressor.disable();
                    mUpFlywheel.stop();
                    mDownFlywheel.stop();
                    mFrontIntake.wantIntaking();
                    mBackIntake.stop();
                    mDualRollerSubsystem.feedBackwards();
                    mConveyor.stop();
                    mLaunchboxAngler.goToAngle(0.0);
                    break;
                case BackIntakeFeedThrough:
                    mDriveTrainSubsystem.setCurrentLimitPerMotor(Constants.kDriveLimitedAmpLimit);
                    mDriveTrainSubsystem.setOpenLoopRampRate(Constants.kDriveLimitedRampRate);
                    mCompressor.disable();
                    mUpFlywheel.stop();
                    mDownFlywheel.stop();
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
    private DualRollerSubsystem mDualRollerSubsystem = DualRollerSubsystem.getInstance();
    private Conveyor mConveyor;
    private LaunchBoxAngler mLaunchboxAngler;
    private SuperStructureStates mSuperStructureStates;
    private PicoColorSensor mPicoColorSensor;
    private LaunchingStates mLaunchingStates;
    private InnerBallPathModes mInnerBallPathMode;
    private Boolean mWantConfirmShot;
    private Shot mWantedShot; // make sure to null check this
    private IdleStates mIdleStates;
    private Alliance mCurrentAlliance;
    private double lastCheckedAllianceTime;
    private int intakePowerManagementStage;
    private double intakePowerManagementChangeTime;
    Compressor mCompressor;
    DriveTrainSubsystem mDriveTrainSubsystem;

    private boolean frontSweep;
    private boolean rearSweep;

    //Cargo counting
    private BisectedCircularBuffer flywheelAverageSpeedsBuffer;
    private boolean isFlywheelSpeedDecreasing;
    private int ballCount;
    private org.usfirst.frc.team1806.robot.util.XboxController mDebugController;

    private SuperStructure() {
        frontSweep = false;
        rearSweep = false;
        intakePowerManagementStage = 0;
        mInnerBallPathMode = InnerBallPathModes.kIdle;
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
        mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
        mDriveTrainSubsystem = DriveTrainSubsystem.getInstance();
        mDebugController = OI.debugController;
    }

    public Compressor getCompressor(){
        return mCompressor;
    }

    @Override
    public void writeToLog() {
        mBackIntake.writeToLog();
        mFrontIntake.writeToLog();
        mDownFlywheel.writeToLog();
        mUpFlywheel.writeToLog();

    }

    @Override
    public void outputToSmartDashboard() {
        mBackIntake.outputToSmartDashboard();
        mFrontIntake.outputToSmartDashboard();
        mDownFlywheel.outputToSmartDashboard();
        mUpFlywheel.outputToSmartDashboard();
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

    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        mBackIntake.registerEnabledLoops(enabledLooper);
        mFrontIntake.registerEnabledLoops(enabledLooper);
        mDownFlywheel.registerEnabledLoops(enabledLooper);
        mUpFlywheel.registerEnabledLoops(enabledLooper);
        mConveyor.registerEnabledLoops(enabledLooper);
        enabledLooper.register(mLoop);
    }

    @Override
    public void setDebug(boolean _debug) {
        mBackIntake.setDebug(_debug);
        mFrontIntake.setDebug(_debug);
        mDownFlywheel.setDebug(_debug);
        mUpFlywheel.setDebug(_debug);
        mConveyor.setDebug(_debug);

    }

    @Override
    public void retractAll() {
        mSuperStructureStates = SuperStructureStates.Idle;
        mBackIntake.retractAll();
        mFrontIntake.retractAll();
        mDownFlywheel.retractAll();
        mUpFlywheel.retractAll();
        mConveyor.retractAll();

    }

    public synchronized void wantIntakeFront() {
        if(mSuperStructureStates!= SuperStructureStates.IntakingFront){
            mSuperStructureStates = SuperStructureStates.IntakingFront;
            intakePowerManagementStage = 0;
            intakePowerManagementChangeTime = Timer.getFPGATimestamp();
        }
        

    };

    public synchronized void wantIntakeBack() {
        if(mSuperStructureStates != SuperStructureStates.IntakingBack){
            mSuperStructureStates = SuperStructureStates.IntakingBack;
            intakePowerManagementStage = 0;
            intakePowerManagementChangeTime = Timer.getFPGATimestamp();
        }
        
    }

    public synchronized void wantInnerBallPathIntake(){
        mInnerBallPathMode = InnerBallPathModes.kIntake;
    }

    public synchronized void wantInnerBallPathReverse(){
        mInnerBallPathMode = InnerBallPathModes.kReverse;
    }

    public synchronized void wantInnerBallPathStop(){
        mInnerBallPathMode = InnerBallPathModes.kIdle;
    }

    public synchronized void wantPrepareShot(Shot wantedShot) {
        if (mSuperStructureStates != SuperStructureStates.Launching) {
            mLaunchingStates = LaunchingStates.kPreparingLaunch;
        }
        mWantedShot = wantedShot;
        mUpFlywheel.setPreciseShot(mWantedShot.getIsPreciseShot());
        mDownFlywheel.setPreciseShot(mWantedShot.getIsPreciseShot());
        mLaunchboxAngler.setIsPreciseShot(mWantedShot.getIsPreciseShot());
        mSuperStructureStates = SuperStructureStates.Launching;
        
    }

    public synchronized void wantConfirmLaunch(Boolean shouldShoot) {
        mWantConfirmShot = shouldShoot;
    }
    
    public synchronized void wantFeedFrontIntake(){
        if(mSuperStructureStates != SuperStructureStates.FrontIntakeFeedThrough){
            mSuperStructureStates = SuperStructureStates.FrontIntakeFeedThrough;
            intakePowerManagementStage = 0;
            intakePowerManagementChangeTime = Timer.getFPGATimestamp();
        }
        
    }

    public synchronized void wantFeedBackIntake(){
        if(mSuperStructureStates != SuperStructureStates.BackIntakeFeedThrough){
            mSuperStructureStates = SuperStructureStates.BackIntakeFeedThrough;
            intakePowerManagementStage = 0;
            intakePowerManagementChangeTime = Timer.getFPGATimestamp();
        }
        

    }

    public static SuperStructure getInstance() {
        return SUPER_STRUCTURE;
    }
    
    public synchronized void overwiteBallCount(int wantedCount){
        ballCount = wantedCount;
    }


    public synchronized boolean doesFrontColorSensorDetectWrongBall(){
        
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
        
        

    public synchronized boolean doesBackColorSensorDetectWrongBall(){

        
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

    public void setWantFrontSweep(boolean sweep){
        frontSweep = sweep;
    }

    public void setWantRearSwee(boolean sweep){
        rearSweep = sweep;
    }

    public void setWantClimbMid(){
        mSuperStructureStates = SuperStructureStates.ClimbingMid;
    }

    public void setWantClimbLow(){
        mSuperStructureStates = SuperStructureStates.ClimbingLow;
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
        

        Robot.getMainDriverTab().addNumber("Shooter Angle", new DoubleSupplier() {

            @Override
            public double getAsDouble() {
                return mLaunchboxAngler.getCurrentAngle();
            }
        

        }).withWidget(BuiltInWidgets.kDial).withPosition(-1,-1).withSize(1,1).withProperties(SHOOTER_ANGLE); //add .withProperties if neccesary
        
    }
}
