package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LaunchBoxAngler implements Subsystem {
    
    private TalonSRX mLaunchMotor;
    private DutyCycleEncoder mEncoder;
    private Encoder mQuadEncoder;
    private PIDController mPIDController;
    private static LaunchBoxAngler LUNCH_BOX_ANGLER = new LaunchBoxAngler();
    private Double mKp, mKi, mKd, mWantedSetPoint;
    private Double angleLeniency = 1.5;
    private Double angleLeniencyImprecise = 6.0;
    private final double ROBOT_OFFSET = Constants.kIsCompBot?-142.6:-51;
    private double currentOffset;
    private double quadOffset;
    private boolean hasOffsetBeenSet;
    private boolean hasOffsetBeenLoopChecked;
    private boolean hasQuadOffsetBeenSet;
    private double wantedManualPower;
    private double batteryVoltage;
    private boolean mIsPreciseShot;
    
    private enum LunchboxStates{
        Idle,
        GoingToPosition,
        AtPosition,
        Manual
    }
    private LunchboxStates mLunchboxStates;

    private Loop mLoop = new Loop(){

        @Override
        public void onStart(double timestamp) {
            //Use
            if(!hasOffsetBeenSet && !hasOffsetBeenLoopChecked && mEncoder.getDistance() >= 300.0)
            {
                currentOffset = currentOffset - 360.0;
                hasOffsetBeenSet = true;
            }
            if(!hasQuadOffsetBeenSet)
            {
                quadOffset = getAbsoluteEncoderAngle();
                mQuadEncoder.reset();   
                hasQuadOffsetBeenSet = true;
            }
            hasOffsetBeenLoopChecked = true;
        }

        @Override
        public void onLoop(double timestamp) {
            
            batteryVoltage = Robot.powerDistributionPanel.getVoltage();
            switch(mLunchboxStates){
                case Idle:
                    mLaunchMotor.set(ControlMode.PercentOutput, 0.0);
                    if(getCurrentAngle() != getAbsoluteEncoderAnglePointing()){
                        quadOffset += getAbsoluteEncoderAnglePointing() - getCurrentAngle();
                    }
                    return;
                case GoingToPosition:
                    
                    double power = mPIDController.calculate(getCurrentAngle(), mWantedSetPoint);
                    /*
                    if(Math.abs(getCurrentAngle()) < 160 && Math.abs(getCurrentAngle()) >50){
                        power  += getCurrentAngle()>0? 0.3 : -0.3;
                    }  
                    power = MathUtil.clamp(power, -0.80, 0.80);
                    */
                   /*
                    if(mWantedSetPoint == 0.0)
                    {   
                        if(Math.abs(getCurrentAngle()) < 30){
                            power =MathUtil.clamp(power, -0.5, 0.55);
                        } 
                    }
                    */
                    double minimumPower = Constants.kLaunchBoxMinimumMovePowerVert + ((Constants.kLaunchBoxMinimumMovePowerHoriz - Constants.kLaunchBoxMinimumMovePowerVert)* Math.abs(Math.sin( Units.degreesToRadians(getCurrentAngle()))));
                    if(batteryVoltage > 10.0){
                        if(isAtAngle())
                        {
                            power = 0;
                            mPIDController.reset();
                            if(getCurrentAngle() != getAbsoluteEncoderAnglePointing()){
                                quadOffset += getAbsoluteEncoderAnglePointing() - getCurrentAngle();
                            }
                        }
                        else{
                            if(power > 0)
                            {
                                power += minimumPower; 
                            }
                            if(power < 0)
                            {
                                power -= minimumPower;
                            }
                        }
                    }
                    else{
                        power = 0;
                    }


                    /*
                    double power =  mWantedSetPoint - getCurrentAngle() > 0? .35 :-.35;

                    if(isAtAngle()){
                        power = 0.0;
                    }
                    */
                    mLaunchMotor.set(ControlMode.PercentOutput, power);

                    
                    return;
                case AtPosition:
                    return;
                case Manual:
                    mLaunchMotor.set(ControlMode.PercentOutput, wantedManualPower);
                    return;
            }
            mLaunchMotor.set(ControlMode.PercentOutput, 0.0);
            
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }
        
    };

    private LaunchBoxAngler(){
        mEncoder = new DutyCycleEncoder(RobotMap.launchBoxAngleEncoder);
        mEncoder.setConnectedFrequencyThreshold(975);
        mEncoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);
        mEncoder.setDistancePerRotation(360); //Rotations to degrees.
        mQuadEncoder = new Encoder(RobotMap.launchBoxAngleQuadA, RobotMap.launchBoxAngleQuadB);
        mQuadEncoder.setDistancePerPulse(360.0/2048.0); // 2048CPR, change to degrees.
        mQuadEncoder.setReverseDirection(false);
        wantedManualPower = 0.0;
        currentOffset = 0.0;
        currentOffset += ROBOT_OFFSET;
        hasOffsetBeenLoopChecked=false;
        hasQuadOffsetBeenSet = false;
        if(mEncoder.getDistance() >= 300.0)
        {
            currentOffset = currentOffset- 360.0;
            hasOffsetBeenSet = true;
            quadOffset = getAbsoluteEncoderAngle();
            hasQuadOffsetBeenSet = true;
        }
        else
        {
            quadOffset = getAbsoluteEncoderAngle();
            hasOffsetBeenSet = false;
        }

        mKp = Constants.kLaunchBoxAnglerKp;
        mKi = Constants.kLaunchBoxAnglerKi;
        mKd = Constants.kLaunchBoxAnglerKd;
        mLaunchMotor = new TalonSRX(RobotMap.launchBoxAngler);
        mLaunchMotor.setInverted(true);
        mLaunchMotor.setNeutralMode(NeutralMode.Brake);
        mLaunchMotor.configPeakCurrentLimit(0);
        mLaunchMotor.configContinuousCurrentLimit(100);
        mLaunchMotor.configVoltageCompSaturation(9);
        mLaunchMotor.enableVoltageCompensation(true);
        mIsPreciseShot = true;
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration(true, 40.0, 50.0, .2);
        mLaunchMotor.configSupplyCurrentLimit(config);

        mPIDController = new PIDController(mKp, mKi, mKd);
        mLunchboxStates = LunchboxStates.Idle;
        mWantedSetPoint = 0.0;
   }

    @Override
    public void writeToLog() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void outputToSmartDashboard() {
        // TODO Auto-generated method stub
        SmartDashboard.putString("Launchbox State", mLunchboxStates.name());
        SmartDashboard.putNumber("Launchbox Encoder Angle", mEncoder.getDistance());
        SmartDashboard.putNumber("Launchbox Wanted Angle", mWantedSetPoint);
        SmartDashboard.putNumber("Launchbox Angle Motor Output", mLaunchMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Launchbox Angle", getCurrentAngle());
        SmartDashboard.putBoolean("hasOffsetBeenSet", hasOffsetBeenSet);
        SmartDashboard.putBoolean("hasOffsetBeenLoopChecked", hasOffsetBeenLoopChecked);
        
    }

    @Override
    public void stop() {
        mLunchboxStates = LunchboxStates.Idle;
        
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        // TODO Auto-generated method stub
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

    public void goToAngle(Double Angle){
        if(mLunchboxStates != LunchboxStates.Manual){
            mWantedSetPoint = Angle;
            mPIDController.setP(Constants.kLaunchBoxAnglerKpVert + ((Constants.kLaunchBoxAnglerKpHoriz - Constants.kLaunchBoxAnglerKpVert) * Math.abs(Math.sin(Units.degreesToRadians(Angle)))));
            mLunchboxStates = LunchboxStates.GoingToPosition;
        }

    }
    
    public Boolean isAtAngle(){
        return checkIfAtArbitraryAngle(mWantedSetPoint);
    }

    public double getCurrentAngle(){
        return mQuadEncoder.getDistance() + quadOffset;
    }

    private double getAbsoluteEncoderAngle(){
        return -(mEncoder.getDistance() + currentOffset);
    }

    private double getAbsoluteEncoderAnglePointing(){
        double currentRotAngle = -(mEncoder.getDistance() + currentOffset) % 360.0;
        if (currentRotAngle > 180){
            currentRotAngle -= 360;
        }
        else if (currentRotAngle < -180){
            currentRotAngle += 360;
        }
       
        return currentRotAngle;
    }

    public Boolean checkIfAtArbitraryAngle(Double angle){
        if(mLunchboxStates == LunchboxStates.Manual){
            return true;
        }
        if(angle == 0.0){
            return Math.abs(angle - getCurrentAngle()) < 2.0;
        }
        if (Math.abs(angle - getCurrentAngle()) > (mIsPreciseShot?angleLeniency:angleLeniencyImprecise)) return false;
        return true;
    }

    public static LaunchBoxAngler getInstance(){
        return LUNCH_BOX_ANGLER;
    }
    
    public void runManualMode(double power){
        if(power != 0 && mLunchboxStates!= LunchboxStates.Manual){
            mLunchboxStates = LunchboxStates.Manual;
        }
        wantedManualPower = power;

    }

    public void setIsPreciseShot(boolean isPreciseShot){
        mIsPreciseShot = isPreciseShot;
    }

    public void reEnable(){
        mLunchboxStates = LunchboxStates.Idle;
    }

    public void resetEncoder(){
        mEncoder.reset();
        mQuadEncoder.reset();
        currentOffset = 0.0;
        quadOffset = 0.0;
    }

    @Override
    public void setupDriverTab() {
        // TODO Auto-generated method stub
        
    }
}
